import numpy as np
import json
import random
from src import Network

def generate_random_data():
    data = []
    for i in range(0, 1000):
        p, i, d = (
            random.gammavariate(random.randint(1, 1000) / 1000,random.randint(1, 1000) / 1000),
            random.gammavariate(random.randint(1, 1000) / 1000,random.randint(1, 1000) / 1000),
            random.gammavariate(random.randint(1, 1000) / 1000,random.randint(1, 1000) / 1000),
        )
        # What is that math???
        e = p * i * d * random.lognormvariate(5, 1)
        data.append([p, i, d, e])
    with open("data.json", 'w') as f:
        f.write(json.dumps(data, indent=4))

if __name__ == '__main__':
    generate_random_data()
    network = Network(3, 1)

    network.update("data.json", "network.json")