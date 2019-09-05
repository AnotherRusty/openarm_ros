'''
    untilities for python 3.x compatible
'''

# convert bytes to integer
def bytes_to_int(c):
    if bytes == str:    # python2
        return ord(c)
    return int(c)   # python3

# convert int to bytes
def int_to_bytes(d):
    if bytes == str:    # python2
        return bytes(bytearray([d]))
    return bytes([d])   # python3


if bytes == str:
    input = raw_input