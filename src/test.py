import numpy as np
from bresenham import bresenham

l = list(bresenham(-1, -4, 3, 2))

for i in l:
    print(i[0])


