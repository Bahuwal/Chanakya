# Humanoid Robot Control - Duke + CAN Implementation

This repository contains Duke University's bipedal RL controller with CAN bus motor support added.

## 📁 Folder Structure

### `/` (Root) - Original Duke Implementation
- EtherCAT-based motor control (C++)
- Original RL policies and configurations  
- Requires compilation
- **For reference or EtherCAT hardware**

### `bipedCANController/` - Pure Python CAN Version
- 100% Python - NO compilation!
- CAN bus support (Motorevo protocol)
- Same RL policies
- Manual control scripts included
- **Ready to use with CAN motors**

---

## 🚀 Quick Start

### Using CAN Motors:
```bash
cd bipedCANController
python3 biped_controller_can.py
```

### Using EtherCAT (Original):
```bash
mkdir build && cd build
cmake .. && make && cd ..
python3 biped_controller.py
```

---

## 📚 Documentation

- `bipedCANController/README.md` - Complete CAN guide
- `bipedCANController/QUICK_START.md` - Quick reference
- `README.md` (this file) - Overview

---

## ✨ What's New in CAN Version

1. ✅ Pure Python - no compilation
2. ✅ Manual control scripts (`can_trajectory_PD_test.py`, `sshkeyboard_pd_test.py`)
3. ✅ Duke's exact joint limits  
4. ✅ Default standing pose
5. ✅ Enhanced safety (limit checking)
6. ✅ Loop counter for RL timing
7. ✅ Comprehensive documentation

---

See `bipedCANController/README.md` for complete details!
