# Utility functions for CAN motor control
# Float/Uint conversion for Motorevo protocol

import numpy as np


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """
    Convert a float value to an unsigned integer for CAN transmission.
    
    Args:
        x: Float value to convert
        x_min: Minimum value of the float range
        x_max: Maximum value of the float range
        bits: Number of bits for the unsigned integer
        
    Returns:
        Unsigned integer representation
    """
    span = x_max - x_min
    offset = x_min
    x = np.clip(x, x_min, x_max)
    return int((x - offset) * ((1 << bits) - 1) / span)


def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """
    Convert an unsigned integer from CAN to a float value.
    
    Args:
        x_int: Unsigned integer value
        x_min: Minimum value of the float range
        x_max: Maximum value of the float range
        bits: Number of bits of the unsigned integer
        
    Returns:
        Float representation
    """
    span = x_max - x_min
    offset = x_min
    return x_int * span / ((1 << bits) - 1) + offset
