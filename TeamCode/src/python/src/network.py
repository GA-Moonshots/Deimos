from time import sleep
from typing import List
from .cell import Cell
from tqdm import tqdm
from ijson import items

class Network:
    gradient_mult = 0.0001
    cells: List[List[Cell]] = []
    inputs: int = 0
    outputs: int = 0

    def __init__(self, inputs: int, outputs: int):
        self.inputs = inputs
        self.cells.append([Cell()])
        self.outputs = outputs

    def update(self, data_file:str, network_file:str):
        f = open(data_file, 'r')
        data = items(f, "item")
        count = 0
        for d in data:
            count += 1

        f.seek(0)

        for point in tqdm(items(f, "item"), total=count):
            print(point)
            # Get inputs and outputs for this point
            inputs = []
            outputs = []

            for i in range(0, self.inputs):
                inputs.append(point[i])
            for i in range(self.inputs, self.inputs + self.outputs):
                outputs.append(point[i])

            # Calculate the input layer before looping

            for cell in self.cells[0]:
                cell.update(inputs)
                print(cell.output)

            # calculate hidden layers

            # calculate output layers
