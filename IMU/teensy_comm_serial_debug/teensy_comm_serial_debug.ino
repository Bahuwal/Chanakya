//Upload this code to teensy if you want to see the data with serial monitor

// #define DEBUG
//  modified 8/11
#define PACKETIZER_USE_INDEX_AS_DEFAULT
#define PACKETIZER_USE_CRC_AS_DEFAULT

// must be before the import of Packetizer
#include <Packetizer.h> // serial with computer
#include <TCA9548.h>    // I2C multiplexer
#include <Wire.h>       // I2C
#include <math.h>

// I2C addresses
#define LOADCELL_ADDR 0x28
#define MULTIPLEXER_ADDR 0x70

constexpr float GRAVITY = 9.80665;

// CONSTANTS
// IMU
const int bufferSize = 256;
byte buffer[bufferSize];
int bufferIndex = 0;
bool ackReceived = false; // IMU acknowledgment for commands
unsigned long commandTimeout = 1000;

// Filter and sensor sample rates
const uint16_t sensor_sample_rate = 500; // Hz
const uint16_t filter_sample_rate = 500; // Hz

//*--------------force
// sensor---------------------------------------------------------
//
bool sampleForceSensors = true;

// multiplexer
PCA9548 multiplexer(MULTIPLEXER_ADDR);

// sensor timing and data collection.
unsigned long previousForceSensorCollectionTime = 0; // ms
const long forceSensorCollectionInterval = 5;        // ms //typical 5 ms
//*---------------------------------------------------------------------------

// Packetizer
// send and recv index
const uint8_t recv_index = 0x12;
const uint8_t send_index = 0x34;

// Finite state machine
enum State {
  WAIT_FOR_U,
  WAIT_FOR_E,
  WAIT_FOR_DESCRIPTOR,
  WAIT_FOR_LENGTH,
  READ_PACKET
};

union FloatBytes {
  float f;
  byte b[4];
} __attribute__((packed));

// Data stream types
enum DescriptorSet {
  BASE_COMMAND = 0x01,
  _3DM_COMMAND = 0x0C,
  FILTER_COMMAND = 0x0D,
  SENSOR_DATA = 0x80,
  FILTER_DATA = 0x82,
  SYSTEM_DATA = 0xA0,
  SHARED_DATA = 0xFF
};

enum DataFieldDescriptors {
  FILTER_FILTER_STATUS = 0x10,
  FILTER_ATT_QUATERNION = 0x03,
  FILTER_ATT_EULER_ANGLES = 0x05,
  FILTER_ATT_DCM = 0x04,
  FILTER_LINEAR_ACCEL = 0x0D,
  FILTER_GRAVITY_VECTOR = 0x13,
  FILTER_ANGULAR_RATE = 0x0E,
  SENSOR_SCALED_ACCEL = 0x04,
  SENSOR_SCALED_GYRO = 0x05,
};

enum CommandSendDescriptors {
  COMMAND_PING = 0x01,
  COMMAND_BUILT_IN_TEST = 0x05,
  COMMAND_GET_BASE_DATA_RATE = 0x0E,
  COMMAND_SET_TO_IDLE = 0x02,
  COMMAND_DEVICE_SETTINGS = 0x30,
  COMMAND_DATASTREAM_CONTROL = 0x11,
  COMMAND_DEFAULT_SETTINGS = 0x30,
  COMMAND_MESSAGE_FORMAT = 0x0F,
  COMMAND_RESUME_SAMPLING = 0x06,
  COMMAND_RESET_NAV_FILTER = 0x01,
  COMMAND_SET_HEADING = 0x03,
  COMMAND_AIDING_MEASUREMENT = 0x50,
  COMMAND_VEHICLE_TRANSFORM_EULER = 0x31,
};

enum FunctionSelectors {
  SELECT_WRITE = 0x01,
  SELECT_READ = 0x02,
  SELECT_SAVE = 0x03,
  SELECT_LOAD = 0x04,
  SELECT_RESET = 0x05,
};

enum CommandReceiveDescriptors {
  RECV_BUILT_IN_TEST = 0x83,
  RECV_AIDING_MEASURMENT = 0xD0,
  RECV_GET_BASE_DATA_RATE = 0x8E,
  RECV_DATASTEAM_CONTROL = 0x85,
};

// Structs for storing packet information.
struct PacketField {
  byte descriptor;
  byte length;
  std::vector<byte> data;
};

struct IMUPacket {
  byte descriptorSet;
  byte length;
  std::vector<PacketField> fields;
};

// for collecting data in one place and sending
struct CombinedData {
  // load cell measurements
  int16_t force_measurement[4]; // hhhh

  // accelerometer data
  float lin_acc_raw[3]; // fff

  // gyroscope data
  float ang_vel_raw[3]; // fff

  // gravity vector estimation filter
  float gravity_vector[3]; // fff

  // quaternion attitude estimation filter
  float quaternion[4]; // ffff

  // gravity-compensated linear acceleration
  float lin_acc_filtered[3]; // fff

  // gravity-compensated angular velocity
  float ang_vel_filtered[3]; // fff

  // filter status, check domumentation
  uint16_t filter_status; // h
  // dynamic mode. check documentation
  uint16_t filter_dynamics_mode; // h
  // status flags. processed in python code.
  uint16_t filter_status_flags; // h
  // check which are fresh.
  uint16_t fresh; // h
} __attribute__((packed));

// Packetizer combined data object
CombinedData combinedData;

// IMU current packet information
State currentState = WAIT_FOR_U;
byte descriptor;
int payload_length;

////////////////////////////////////
// IMU variables (updated by IMU) //
////////////////////////////////////

uint16_t filter_base_rate = 0;
uint16_t sensor_base_rate = 0;

// verifying channels are activate when enabling datastream
bool sensor_channel_active = false;
bool filter_channel_active = false;
bool aiding_measurements_active = true;

// for timing
unsigned long startTime;
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100; // Print every 100ms

void setup() {
  Serial.begin(115200); // Changed to 115200 for debug output
  Serial1.begin(921600);
  Wire.begin();
  Wire.setClock(921600UL);

  delay(2000); // Give time for Serial Monitor to open
  Serial.println("\n========================================");
  Serial.println("IMU DEBUG MODE - Human Readable Output");
  Serial.println("========================================\n");

  // start all data as not fresh
  combinedData.fresh = 0;

  // Initialize the IMU with default settings.
  if (setToIdle()) {
    Serial.println("✓ Successfully set to idle.");
  } else {
    Serial.println("✗ Failed to set to idle.");
  }

  Serial.println();

  if (loadDefaultSettings()) {
    Serial.println("✓ Successfully loaded default settings.");
  } else {
    Serial.println("✗ Failed to load default settings.");
  }

  Serial.println();

  // Configure sensor data.
  if (getDataBaseRate(SENSOR_DATA)) {
    Serial.print("Sensor base data rate: ");
    Serial.print(sensor_base_rate);
    Serial.println(" Hz");
  } else {
    Serial.println("✗ Failed to get base data rate.");
  }

  Serial.println();

  const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

  // Set Message Format
  std::vector<std::pair<byte, uint16_t>> sensorDescriptorsAndDecimators = {
      {SENSOR_SCALED_ACCEL, sensor_decimation}, // Accelerometer
      {SENSOR_SCALED_GYRO, sensor_decimation},  // Gyroscope
  };

  if (setMessageFormat(SENSOR_DATA, sensorDescriptorsAndDecimators)) {
    Serial.println("✓ Sensor data message format successfully updated.");
  } else {
    Serial.println("✗ Failed to set message format.");
  }

  Serial.println();

  if (enableDatastream(SENSOR_DATA)) {
    Serial.println("✓ Successfully enabled sensor data.");
  } else {
    Serial.println("✗ Failed to enable sensor datastream.");
  }

  Serial.println();

  if (verifyDatastream(SENSOR_DATA)) {
    Serial.println("✓ Verified: Sensor datastream enabled.");
  } else {
    Serial.println("✗ Verification failed: Sensor data not available.");
  }

  Serial.println();

  // Configure filter data.
  if (resetNavigationFilter()) {
    Serial.println("✓ Navigation filter reset successfully.");
  } else {
    Serial.println("✗ Failed to reset navigation filter.");
  }

  if (setHeadingControl()) {
    Serial.println("✓ Heading control set successfully.");
  } else {
    Serial.println("✗ Failed to set heading control.");
  }

  if (loadAllAidingMeasurement()) {
    Serial.println("✓ Loaded all aiding mesurements successfully.");
  } else {
    Serial.println("✗ Failed to load all aiding measurements.");
  }

  Serial.println();

  if (sensorToVehicleTransform(0.0, 0.0, 0.0)) {
    Serial.println("✓ Sensor to vehicle transform successfully updated.");
  } else {
    Serial.println("✗ Failed to updated sensor to vehicle transform.");
  }

  Serial.println();

  if (readSensorToVehicleTransform()) {
    Serial.println("✓ Reading sensor to vehicle transform.");
  } else {
    Serial.println("✗ Failed to updated sensor to vehicle transform.");
  }

  if (getDataBaseRate(FILTER_DATA)) {
    Serial.print("Filter base data rate: ");
    Serial.print(filter_base_rate);
    Serial.println(" Hz");
  } else {
    Serial.println("✗ Failed to get base data rate.");
  }

  Serial.println();

  const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

  // Set Message Format
  std::vector<std::pair<byte, uint16_t>> filterDescriptorsAndDecimators = {
      {FILTER_ATT_QUATERNION, filter_decimation}, // Attitude quaternion
      {FILTER_FILTER_STATUS, filter_decimation},  // Filter Status
      {FILTER_GRAVITY_VECTOR, filter_decimation}, // Gravity vector
      {FILTER_LINEAR_ACCEL,
       filter_decimation}, // Gravity-compensated linear acceleration,
      {FILTER_ANGULAR_RATE, filter_decimation}, // Filter-compensated gyro
  };

  if (setMessageFormat(FILTER_DATA, filterDescriptorsAndDecimators)) {
    Serial.println("✓ Filter data message format successfully updated.");
  } else {
    Serial.println("✗ Failed to set message format.");
  }

  Serial.println();

  if (enableDatastream(FILTER_DATA)) {
    Serial.println("✓ Successfully enabled filter data.");
  } else {
    Serial.println("✗ Failed to enable filter datastream.");
  }

  Serial.println();

  if (verifyDatastream(FILTER_DATA)) {
    Serial.println("✓ Verified: Filter datastream enabled.");
  } else {
    Serial.println("✗ Verification failed: Filter data not available.");
  }

  Serial.println();

  startTime = millis();

  if (resumeSampling()) {
    Serial.println("✓ IMU sampling resumed.\n");
    Serial.println("========================================");
    Serial.println("Starting IMU Data Stream...");
    Serial.println("========================================\n");
  } else {
    Serial.println("✗ Failed to resume IMU sampling.");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // Read force sensor data at regular intervals
  if ((currentMillis - previousForceSensorCollectionTime >=
       forceSensorCollectionInterval) &&
      sampleForceSensors) {
    previousForceSensorCollectionTime = currentMillis;
    for (int i = 0; i < 4; i++) {
      readLoadCell(i);
    }
  }

  // SERIAL MONITOR DEBUG MODE: Print human-readable data
  // For GUI mode with Python, use teensy_comm_gui.ino instead
  if (currentMillis - lastPrintTime >= printInterval) {
    lastPrintTime = currentMillis;
    printIMUData();
  }

  // Binary packet sending is DISABLED in debug mode
  // (Uncomment below if you need both, but GUI won't work properly)
  /*
  noInterrupts();
  if (shouldSendPacket()) {
    uint8_t *data = reinterpret_cast<uint8_t *>(&combinedData);
    Packetizer::send(Serial, send_index, data, sizeof(combinedData));
    combinedData.fresh = 0;
  }
  interrupts();
  */
}

// NEW FUNCTION: Print human-readable IMU data
void printIMUData() {
  Serial.println("─────────────────────────────────────────");
  Serial.print("Time: ");
  Serial.print((millis() - startTime) / 1000.0, 3);
  Serial.println(" s");

  // Print Accelerometer (Raw)
  Serial.println("\n[Raw Accelerometer] (m/s²)");
  Serial.print("  X: ");
  Serial.print(combinedData.lin_acc_raw[0], 4);
  Serial.print("  Y: ");
  Serial.print(combinedData.lin_acc_raw[1], 4);
  Serial.print("  Z: ");
  Serial.println(combinedData.lin_acc_raw[2], 4);

  // Print Gyroscope (Raw)
  Serial.println("\n[Raw Gyroscope] (rad/s)");
  Serial.print("  X: ");
  Serial.print(combinedData.ang_vel_raw[0], 4);
  Serial.print("  Y: ");
  Serial.print(combinedData.ang_vel_raw[1], 4);
  Serial.print("  Z: ");
  Serial.println(combinedData.ang_vel_raw[2], 4);

  // Print Quaternion
  Serial.println("\n[Quaternion] (x, y, z, w)");
  Serial.print("  X: ");
  Serial.print(combinedData.quaternion[0], 4);
  Serial.print("  Y: ");
  Serial.print(combinedData.quaternion[1], 4);
  Serial.print("  Z: ");
  Serial.print(combinedData.quaternion[2], 4);
  Serial.print("  W: ");
  Serial.println(combinedData.quaternion[3], 4);

  // Print Gravity Vector
  Serial.println("\n[Gravity Vector] (g)");
  Serial.print("  X: ");
  Serial.print(combinedData.gravity_vector[0], 4);
  Serial.print("  Y: ");
  Serial.print(combinedData.gravity_vector[1], 4);
  Serial.print("  Z: ");
  Serial.println(combinedData.gravity_vector[2], 4);

  // Print Linear Acceleration (Filtered)
  Serial.println("\n[Filtered Linear Acceleration] (m/s²)");
  Serial.print("  X: ");
  Serial.print(combinedData.lin_acc_filtered[0], 4);
  Serial.print("  Y: ");
  Serial.print(combinedData.lin_acc_filtered[1], 4);
  Serial.print("  Z: ");
  Serial.println(combinedData.lin_acc_filtered[2], 4);

  // Print Angular Velocity (Filtered)
  Serial.println("\n[Filtered Angular Velocity] (rad/s)");
  Serial.print("  X: ");
  Serial.print(combinedData.ang_vel_filtered[0], 4);
  Serial.print("  Y: ");
  Serial.print(combinedData.ang_vel_filtered[1], 4);
  Serial.print("  Z: ");
  Serial.println(combinedData.ang_vel_filtered[2], 4);

  // Print Filter Status
  Serial.println("\n[Filter Status]");
  Serial.print("  Status: 0x");
  Serial.print(combinedData.filter_status, HEX);
  Serial.print("  Dynamics Mode: 0x");
  Serial.print(combinedData.filter_dynamics_mode, HEX);
  Serial.print("  Flags: 0x");
  Serial.println(combinedData.filter_status_flags, HEX);

  // Print Load Cells
  Serial.println("\n[Load Cells]");
  for (int i = 0; i < 4; i++) {
    Serial.print("  Cell ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(combinedData.force_measurement[i]);
    if (i < 3)
      Serial.print(", ");
  }
  Serial.println();

  // Print Fresh Data Indicator
  Serial.print("\nFresh Data Bits: 0b");
  Serial.println(combinedData.fresh, BIN);

  Serial.println();
}

// [ALL THE REST OF THE FUNCTIONS REMAIN UNCHANGED - Including serialEvent1,
// processPacket, etc.]

// Interrupt on receiving data with finite state machine.
void serialEvent1() {
  while (Serial1.available()) {
    byte incomingByte = Serial1.read();

    switch (currentState) {
    case WAIT_FOR_U:
      if (incomingByte == 'u') {
        bufferIndex = 0;
        buffer[bufferIndex++] = incomingByte;
        currentState = WAIT_FOR_E;
      }
      break;

    case WAIT_FOR_E:
      if (incomingByte == 'e') {
        buffer[bufferIndex++] = incomingByte;
        currentState = WAIT_FOR_DESCRIPTOR;
      } else {
        currentState = WAIT_FOR_U;
        bufferIndex = 0;
      }
      break;

    case WAIT_FOR_DESCRIPTOR:
      buffer[bufferIndex++] = incomingByte;
      descriptor = incomingByte;
      currentState = WAIT_FOR_LENGTH;
      break;

    case WAIT_FOR_LENGTH:
      buffer[bufferIndex++] = incomingByte;
      payload_length = (int)incomingByte;
      if (payload_length > bufferSize - 6) {
        Serial.println("Invalid packet length");
        currentState = WAIT_FOR_U;
        bufferIndex = 0;
      } else {
        currentState = READ_PACKET;
      }
      break;

    case READ_PACKET:
      if (bufferIndex < bufferSize) {
        buffer[bufferIndex++] = incomingByte;
        if (bufferIndex >= payload_length + 6) {
          processPacket(buffer, payload_length + 6);
          currentState = WAIT_FOR_U;
        }
      } else {
        Serial.println("Buffer overflow detected");
        currentState = WAIT_FOR_U;
      }
      break;
    }
  }
}

void processPacket(byte *packet, int packet_length) {
  if (!verifyChecksum(packet, packet_length)) {
    Serial.println("Bad checksum.");
    ackReceived = false;
    return;
  }

  IMUPacket response_packet = parsePacket(packet, packet_length);

  if (response_packet.descriptorSet == BASE_COMMAND) {
    handleBaseCommand(response_packet);
  } else if (response_packet.descriptorSet == _3DM_COMMAND) {
    handle3DMCommand(response_packet);
  } else if (response_packet.descriptorSet == SENSOR_DATA) {
    handleSensorData(response_packet);
  } else if (response_packet.descriptorSet == FILTER_COMMAND) {
    handleFilterCommand(response_packet);
  } else if (response_packet.descriptorSet == FILTER_DATA) {
    handleFilterData(response_packet);
  } else {
    Serial.print("ERROR - Unrecognized packet descriptor set: ");
    Serial.println(response_packet.descriptorSet, HEX);
  }
}

void handleBaseCommand(IMUPacket packet) {
  ackReceived = fieldsContainACK(packet);
  for (const auto &field : packet.fields) {
    if (field.descriptor == COMMAND_BUILT_IN_TEST && field.data.size() == 6) {
      for (byte data : field.data) {
        printBinaryWithLeadingZeros(data);
      }
      Serial.println();
    }
  }
}

void handle3DMCommand(IMUPacket packet) {
  ackReceived = fieldsContainACK(packet);

  if (!ackReceived) {
    return;
  }

  for (const auto &field : packet.fields) {
    if (field.descriptor == RECV_GET_BASE_DATA_RATE && field.data.size() == 3) {
      if (field.data[0] == FILTER_DATA) {
        filter_base_rate = (field.data[1] << 8) | field.data[2];
      } else if (field.data[0] == SENSOR_DATA) {
        sensor_base_rate = (field.data[1] << 8) | field.data[2];
      }
    } else if (field.descriptor == RECV_DATASTEAM_CONTROL) {
      if (field.data[0] == SENSOR_DATA && field.data.size() == 2) {
        sensor_channel_active = field.data[1];
      } else if (field.data[1] == FILTER_DATA && field.data.size() == 2) {
        filter_channel_active = field.data[1];
      }
    }
  }
}

void handleFilterCommand(IMUPacket packet) {
  ackReceived = fieldsContainACK(packet);

  if (!ackReceived) {
    return;
  }

  for (const auto &field : packet.fields) {
    if (field.descriptor == RECV_AIDING_MEASURMENT && field.data.size() == 3) {
      if (field.data[0] == 0xFF && field.data[1] == 0xFF) {
        if (field.data[2] == 0x01) {
          aiding_measurements_active = true;
        } else {
          aiding_measurements_active = false;
        }
      }
    }
  }
}

void handleFilterData(IMUPacket packet) {
  for (const auto &field : packet.fields) {
    if (field.descriptor == FILTER_LINEAR_ACCEL && field.length == 0x10) {
      bool valid = (bool)((field.data[12] << 8) | field.data[13]);
      if (valid) {
        FloatBytes *fb =
            reinterpret_cast<FloatBytes *>(combinedData.lin_acc_filtered);
        for (int i = 0; i < 3; ++i) {
          fb[i].b[0] = field.data[i * 4 + 3];
          fb[i].b[1] = field.data[i * 4 + 2];
          fb[i].b[2] = field.data[i * 4 + 1];
          fb[i].b[3] = field.data[i * 4];
        }
        combinedData.lin_acc_filtered[1] = -combinedData.lin_acc_filtered[1];
        combinedData.lin_acc_filtered[2] = -combinedData.lin_acc_filtered[2];
        combinedData.fresh |= (1 << 5);
      }
    } else if (field.descriptor == FILTER_GRAVITY_VECTOR &&
               field.length == 0x10) {
      bool valid = (bool)((field.data[12] << 8) | field.data[13]);
      if (valid) {
        FloatBytes *fb =
            reinterpret_cast<FloatBytes *>(combinedData.gravity_vector);
        for (int i = 0; i < 3; ++i) {
          fb[i].b[0] = field.data[i * 4 + 3];
          fb[i].b[1] = field.data[i * 4 + 2];
          fb[i].b[2] = field.data[i * 4 + 1];
          fb[i].b[3] = field.data[i * 4];
        }
        combinedData.gravity_vector[0] =
            -combinedData.gravity_vector[0] / GRAVITY;
        combinedData.gravity_vector[1] =
            combinedData.gravity_vector[1] / GRAVITY;
        combinedData.gravity_vector[2] =
            combinedData.gravity_vector[2] / GRAVITY;
        combinedData.fresh |= (1 << 8);
      }
    } else if (field.descriptor == FILTER_ATT_QUATERNION &&
               field.length == 0x14) {
      bool valid = (bool)((field.data[16] << 8) | field.data[17]);
      if (valid) {
        FloatBytes fb[4];
        for (int i = 0; i < 4; ++i) {
          fb[i].b[0] = field.data[i * 4 + 3];
          fb[i].b[1] = field.data[i * 4 + 2];
          fb[i].b[2] = field.data[i * 4 + 1];
          fb[i].b[3] = field.data[i * 4];
        }
        combinedData.quaternion[0] = fb[1].f;
        combinedData.quaternion[1] = -fb[2].f;
        combinedData.quaternion[2] = -fb[3].f;
        combinedData.quaternion[3] = fb[0].f;
        combinedData.fresh |= (1 << 6);
      }
    } else if (field.descriptor == FILTER_FILTER_STATUS &&
               field.data.size() == 6) {
      combinedData.filter_status = (field.data[0] << 8) | field.data[1];
      combinedData.filter_dynamics_mode = (field.data[2] << 8) | field.data[3];
      combinedData.filter_status_flags = (field.data[4] << 8) | field.data[5];
      combinedData.fresh |= (1 << 4);
    } else if (field.descriptor == FILTER_ANGULAR_RATE &&
               field.length == 0x10) {
      FloatBytes *fb =
          reinterpret_cast<FloatBytes *>(combinedData.ang_vel_filtered);
      for (int i = 0; i < 3; ++i) {
        fb[i].b[0] = field.data[i * 4 + 3];
        fb[i].b[1] = field.data[i * 4 + 2];
        fb[i].b[2] = field.data[i * 4 + 1];
        fb[i].b[3] = field.data[i * 4];
      }
      combinedData.ang_vel_filtered[1] = -combinedData.ang_vel_filtered[1];
      combinedData.ang_vel_filtered[2] = -combinedData.ang_vel_filtered[2];
      combinedData.fresh |= (1 << 10);
    } else {
      Serial.print("Unrecognized data type! ");
      Serial.print("Descriptor: 0x");
      Serial.print(field.descriptor, HEX);
      Serial.print(", Length: 0x");
      Serial.println(field.length, HEX);
    }
  }
}

void handleSensorData(IMUPacket packet) {
  for (const auto &field : packet.fields) {
    if (field.descriptor == SENSOR_SCALED_ACCEL && field.data.size() == 12) {
      FloatBytes *fb = reinterpret_cast<FloatBytes *>(combinedData.lin_acc_raw);
      for (int i = 0; i < 3; ++i) {
        fb[i].b[0] = field.data[i * 4 + 3];
        fb[i].b[1] = field.data[i * 4 + 2];
        fb[i].b[2] = field.data[i * 4 + 1];
        fb[i].b[3] = field.data[i * 4];
      }
      combinedData.lin_acc_raw[0] = combinedData.lin_acc_raw[0] * GRAVITY;
      combinedData.lin_acc_raw[1] = -combinedData.lin_acc_raw[1] * GRAVITY;
      combinedData.lin_acc_raw[2] = -combinedData.lin_acc_raw[2] * GRAVITY;
      combinedData.fresh |= (1 << 10);
    } else if (field.descriptor == SENSOR_SCALED_GYRO &&
               field.data.size() == 12) {
      FloatBytes *fb = reinterpret_cast<FloatBytes *>(combinedData.ang_vel_raw);
      for (int i = 0; i < 3; ++i) {
        fb[i].b[0] = field.data[i * 4 + 3];
        fb[i].b[1] = field.data[i * 4 + 2];
        fb[i].b[2] = field.data[i * 4 + 1];
        fb[i].b[3] = field.data[i * 4];
      }
      combinedData.ang_vel_raw[1] = -combinedData.ang_vel_raw[1];
      combinedData.ang_vel_raw[2] = -combinedData.ang_vel_raw[2];
      combinedData.fresh |= (1 << 9);
    } else {
      Serial.println("Unrecognized data type.");
    }
  }
}

IMUPacket parsePacket(byte *buffer, int length) {
  IMUPacket packet;
  packet.descriptorSet = buffer[2];
  packet.length = buffer[3];
  int index = 4;

  while (index < length - 2) {
    PacketField field;
    field.length = buffer[index++];
    field.descriptor = buffer[index++];
    field.data.assign(buffer + index, buffer + index + field.length - 2);
    index += field.length - 2;
    packet.fields.push_back(field);
  }
  return packet;
}

bool verifyChecksum(byte *packet, int packet_length) {
  uint16_t packet_checksum =
      (packet[packet_length - 2] << 8) | packet[packet_length - 1];
  uint16_t calculated_checksum = fletcher_checksum(packet, packet_length);
  return packet_checksum == calculated_checksum;
}

uint16_t fletcher_checksum(const uint8_t *packet, int packet_length) {
  const int checksum_length = packet_length - 2;
  uint8_t checksum_MSB = 0;
  uint8_t checksum_LSB = 0;
  for (int i = 0; i < checksum_length; i++) {
    checksum_MSB += packet[i];
    checksum_LSB += checksum_MSB;
  }
  return ((uint16_t)checksum_MSB << 8) | (uint16_t)checksum_LSB;
}

bool fieldsContainACK(IMUPacket packet) {
  for (const auto &field : packet.fields) {
    if (field.descriptor == 0xF1) {
      if (field.data[1] == 0x00) {
        return true;
      } else if (field.data[1] == 0x03) {
        Serial.println("Invalid parameter!");
      }
    }
  }
  return false;
}

bool checkACK() {
  unsigned long startTime = millis();
  while (millis() - startTime < commandTimeout) {
    noInterrupts();
    if (ackReceived) {
      ackReceived = false;
      interrupts();
      return true;
    }
    interrupts();
    yield();
  }

  Serial.println("Timeout occurred!");
  return false;
}

void addChecksum(byte *command, int command_length) {
  uint16_t checksum = fletcher_checksum(command, command_length);
  command[command_length - 2] = (byte)(checksum >> 8);
  command[command_length - 1] = (byte)(checksum & 0xFF);
}

bool ping() {
  int command_length = 8;
  byte command[command_length] = {0x75, 0x65,         BASE_COMMAND, 0x02,
                                  0x02, COMMAND_PING, 0x00,         0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool setToIdle() {
  int command_length = 8;
  byte command[] = {0x75, 0x65, BASE_COMMAND, 0x02, 0x02, COMMAND_SET_TO_IDLE,
                    0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool loadDefaultSettings() {
  int command_length = 9;
  byte command[] = {0x75,        0x65, _3DM_COMMAND,
                    0x03,        0x03, COMMAND_DEFAULT_SETTINGS,
                    SELECT_LOAD, 0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool getDataBaseRate(byte dataDescriptorSet) {
  int command_length = 9;
  byte command[] = {0x75,
                    0x65,
                    _3DM_COMMAND,
                    0x03,
                    0x03,
                    COMMAND_GET_BASE_DATA_RATE,
                    dataDescriptorSet,
                    0x00,
                    0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool setMessageFormat(
    byte descriptorSet,
    const std::vector<std::pair<byte, uint16_t>> &descriptorsAndDecimators) {
  int numDescriptors = descriptorsAndDecimators.size();
  int command_length = 11 + numDescriptors * 3;
  std::vector<byte> command(command_length, 0);
  command[0] = 0x75;
  command[1] = 0x65;
  command[2] = _3DM_COMMAND;
  command[3] = 5 + numDescriptors * 3;
  command[4] = 5 + numDescriptors * 3;
  command[5] = COMMAND_MESSAGE_FORMAT;
  command[6] = 0x01;
  command[7] = descriptorSet;
  command[8] = numDescriptors;
  for (int i = 0; i < numDescriptors; ++i) {
    command[9 + i * 3] = descriptorsAndDecimators[i].first;
    command[10 + i * 3] = (byte)(descriptorsAndDecimators[i].second >> 8);
    command[11 + i * 3] = (byte)(descriptorsAndDecimators[i].second & 0xFF);
  }
  command[12 + numDescriptors * 3] = 0x00;
  command[13 + numDescriptors * 3] = 0x00;
  addChecksum(command.data(), command_length);
  Serial1.write(command.data(), command_length);
  return checkACK();
}

bool verifyDatastream(byte dataDescriptorSet) {
  int command_length = 10;
  byte command[] = {0x75,         0x65,
                    _3DM_COMMAND, 0x04,
                    0x04,         COMMAND_DATASTREAM_CONTROL,
                    SELECT_READ,  dataDescriptorSet,
                    0x00,         0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool enableDatastream(byte dataDescriptorSet) {
  int command_length = 11;
  byte command[] = {0x75,         0x65,
                    _3DM_COMMAND, 0x05,
                    0x05,         COMMAND_DATASTREAM_CONTROL,
                    SELECT_WRITE, dataDescriptorSet,
                    0x01,         0x00,
                    0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  if (checkACK()) {
    if (dataDescriptorSet == SENSOR_DATA) {
      sensor_channel_active = true;
      return true;
    } else if (dataDescriptorSet == FILTER_DATA) {
      filter_channel_active = true;
      return true;
    } else {
      Serial.println("Unrecognized data descriptor set.");
      return false;
    }
  } else {
    return false;
  }
}

bool disableDatastream(byte dataDescriptorSet) {
  int command_length = 11;
  byte command[] = {0x75,         0x65,
                    _3DM_COMMAND, 0x05,
                    0x05,         COMMAND_DATASTREAM_CONTROL,
                    SELECT_WRITE, dataDescriptorSet,
                    0x00,         0x00,
                    0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  if (checkACK()) {
    if (dataDescriptorSet == SENSOR_DATA) {
      sensor_channel_active = false;
      return true;
    } else if (dataDescriptorSet == FILTER_DATA) {
      filter_channel_active = false;
      return true;
    } else {
      Serial.println("Unrecognized data descriptor set.");
      return false;
    }
  } else {
    return false;
  }
}

bool resumeSampling() {
  int command_length = 8;
  byte command[] = {0x75, 0x65, BASE_COMMAND,
                    0x02, 0x02, COMMAND_RESUME_SAMPLING,
                    0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool resetNavigationFilter() {
  int command_length = 8;
  byte command[] = {0x75, 0x65, FILTER_COMMAND,
                    0x02, 0x02, COMMAND_RESET_NAV_FILTER,
                    0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool setHeadingControl() {
  int command_length = 12;
  byte command[] = {0x75, 0x65, FILTER_COMMAND, 0x06, 0x06, COMMAND_SET_HEADING,
                    0x00, 0x00, 0x00,           0x00, 0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool loadAllAidingMeasurement() {
  int command_length = 12;
  byte command[] = {
      0x75,        0x65, FILTER_COMMAND, 0x06, 0x06, COMMAND_AIDING_MEASUREMENT,
      SELECT_LOAD, 0xFF, 0xFF,           0x01, 0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool sensorToVehicleTransform(float x, float y, float z) {
  byte *xBytes = (byte *)&x;
  byte *yBytes = (byte *)&y;
  byte *zBytes = (byte *)&z;
  int command_length = 21;
  byte command[] = {0x75,         0x65,      _3DM_COMMAND,
                    0x0F,         0x0F,      COMMAND_VEHICLE_TRANSFORM_EULER,
                    SELECT_WRITE, xBytes[3], xBytes[2],
                    xBytes[1],    xBytes[0], yBytes[3],
                    yBytes[2],    yBytes[1], yBytes[0],
                    zBytes[3],    zBytes[2], zBytes[1],
                    zBytes[0],    0x00,      0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool readSensorToVehicleTransform() {
  int command_length = 9;
  byte command[] = {0x75,        0x65, _3DM_COMMAND,
                    0x03,        0x03, COMMAND_VEHICLE_TRANSFORM_EULER,
                    SELECT_READ, 0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

bool runBuiltInTest() {
  int command_length = 8;
  byte command[] = {0x75, 0x65, BASE_COMMAND, 0x02, 0x02, COMMAND_BUILT_IN_TEST,
                    0x00, 0x00};
  addChecksum(command, command_length);
  Serial1.write(command, command_length);
  return checkACK();
}

void readLoadCell(int index) {
  multiplexer.selectChannel(index);
  Wire.requestFrom(LOADCELL_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t data[2];
    data[0] = Wire.read();
    data[1] = Wire.read();
    uint8_t bridge_status = data[0] >> 6;
    combinedData.force_measurement[index] =
        (int16_t)(((data[0] & 0x3F) << 8) | data[1]);
    if (bridge_status == 2) {
      // Data is stale
    } else {
      combinedData.fresh |= (1 << index);
    }
  } else {
    Serial.print("Loadcell no data: ");
    Serial.println(index);
  }
}

bool shouldSendPacket() {
  uint16_t load_cell_mask = 0b0000000000001111;
  if ((load_cell_mask & combinedData.fresh) == load_cell_mask)
    return true;
  uint16_t filter_and_sensor_mask = 0b0000011101110000;
  if ((filter_and_sensor_mask & combinedData.fresh) == filter_and_sensor_mask)
    return true;
  return false;
}

void printBinaryWithLeadingZeros(byte value) {
  for (int i = 15; i >= 0; i--) {
    Serial.print(bitRead(value, i));
  }
  Serial.println();
}
