def limit_min_max(x, min, max):
    if x <= min:
        x = min
    elif x > max:
        x = max
    return x

def uint_to_float(value, min_val, max_val, bits):
    # value = limit_min_max(value, min_val, max_val)
    span = max_val - min_val
    return (value) * span / ((1 << bits) - 1) + min_val


def float_to_uint(value, min_val, max_val, bits):
    # value = limit_min_max(value, min_val, max_val)
    span = max_val - min_val
    return int((value-min_val) * ((1 << bits) - 1) / span)
