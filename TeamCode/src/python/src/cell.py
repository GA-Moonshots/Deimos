from typing import List
from math import e
from random import randint
from decimal import Decimal

from .activation_functions import swish

class Cell:
    def __init__(self):
        self.m = randint(1, 1000) / 100
        self.x = randint(1, 1000) / 100
        self.c = randint(1, 1000) / 100
        self.output = 0

    weights: List[int] = []

    def update(self, inputs: List[Decimal]):
        if len(self.weights) > len(inputs):
            for i in range(len(self.weights) - len(inputs)):
                self.weights.remove(0)
        elif len(self.weights) < len(inputs):
            for i in range(len(inputs) - len(self.weights)):
                self.weights.append(0)
        
        v = 0
        for i in range(len(inputs)):
            v += self.weights[i] * inputs[i]

        a, b = v.as_integer_ratio()

        v = (a / b) * self.x

        self.output = self.m * swish(v) + self.c
