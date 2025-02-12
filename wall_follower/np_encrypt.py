"""
DO NOT MODIFY THIS FILE.

This encrypts/decrypts your numpy arrays for grading purposes.
"""

import numpy as np

# http://www.brianveitch.com/cryptography/generate_rsa_keys.php
public_key = 1305295067
private_key = 3070545323
modulus = 7366846427

pow_modulo = np.frompyfunc(pow, 3, 1)

def encode(array):
    array_int = array.astype(np.float32).view(np.uint32).astype(np.uint64)
    array_int_enc = pow_modulo(array_int, public_key, modulus)
    return array_int_enc

def decode(array):
    array_int = array.astype(np.uint64)
    array_int_dec = pow_modulo(array_int, private_key, modulus)
    array_dec = array_int_dec.astype(np.uint32).view(np.float32)
    return array_dec
