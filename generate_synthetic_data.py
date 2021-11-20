import numpy as np

packet_delimiter = '|'
value_delimiter = '_'

def get_sample_points():
    return np.random.choice(np.arange(-10, 11), 10)

def sigmoid(x, shift, coeff):
    return 1 / (1 + np.exp(-coeff*(x - shift)))

def generate_synthetic_data():
    return packet_delimiter.join([get_thumb_value(), get_index_value(), get_middle_value(), get_ring_value()])

def get_thumb_value():
    return np.array2string(sigmoid(get_sample_points(), 0, 2) * 200,
        separator='_',
        formatter={'float_kind':lambda x: "%.2f" % x},
        suppress_small = True)[1:-1]

def get_index_value():
    return np.array2string(sigmoid(get_sample_points(), 0, 1) * 1500,
        separator='_',
        formatter={'float_kind':lambda x: "%.2f" % x},
        suppress_small = True)[1:-1]

def get_middle_value():
    return np.array2string(sigmoid(get_sample_points(), 0, 0.8) * 3000,
        separator='_',
        formatter={'float_kind':lambda x: "%.2f" % x},
        suppress_small = True)[1:-1]

def get_ring_value():
    return np.array2string(sigmoid(get_sample_points(), 0, 1.5) * 500,
        separator='_',
        formatter={'float_kind':lambda x: "%.2f" % x},
        suppress_small = True)[1:-1]