import numpy as np

def RemapRange(values, min_in, max_in, min_out, max_out):
    values = np.clip(values, min_in, max_in)
    proportion = (values - min_in) / (max_in - min_in)
    return proportion * (max_out-min_out) + min_out