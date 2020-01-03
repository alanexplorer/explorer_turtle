import math
import numpy as np

class transform():

  R = []
  T = []

  def getRotationMatrix(self, theta):
    
    self.R = np.array([[math.cos(theta), -math.sin(theta), 0], 
                    [math.sin(theta), math.cos(theta), 0.0],
                      [0.0, 0.0, 1.0]])
    return self.R

  def getTranslationMatrix(self, dx, dy):
    
    self.T = np.array([[1.0, 0.0, dx], 
                    [0.0, 1.0, dy], 
                  [0.0, 0.0, 1.0]])
    return self.T

  def pointTransform(self, x, y):

    v = np.array([[x], [y], [0]])
    r = np.dot(self.R, self.T)
    v = np.dot(r, v)
    return (v[0], v[1])

