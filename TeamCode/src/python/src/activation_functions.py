from math import e

def sigmoid(x: float) -> float:
    return 1 / (1 + pow(e, -x))

def swish(x: float) -> float:
    return x * sigmoid(x)