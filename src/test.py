import numpy as np
from eigen import transform

t = transform()

print(t.getRotationMatrix(0))
print()
print(t.getTranslationMatrix(0,-1))
print()
print(t.pointTransform(2,3))