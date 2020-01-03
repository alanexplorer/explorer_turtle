import math

class line():
    def find(self, x0, y0, x1, y1):
        deltaX = x1 - x0
        deltaY = y1 - y0
        deltaErr = math.fabs(deltaY/deltaX) #Assume deltax != 0 (line is not vertical),// note that this division needs to be done in a way that preserves the fractional part
        error = 0.0 #no error at start
        y = y0
        for x in range(x0,x1):
            print(x,y)
            error = error + deltaErr
            if error >=1:
                y = y + sign(deltaY)*1
                error = error-1.0

