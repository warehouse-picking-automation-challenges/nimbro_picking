

class Rectangle:
  def __init__(self, xc, yc, w, h):
    self.xc, self.yc, self.w, self.h = float(xc), float(yc), float(w), float(h)
  
  def area(self):
    return self.w*self.h

  def left(self):
    return self.xc - 0.5*self.w

  def right(self):
    return self.xc + 0.5*self.w

  def top(self):
    return self.yc - 0.5*self.h

  def bottom(self):
    return self.yc + 0.5*self.h

  def topLeft(self):
    return (self.left(), self.top())

  def topRight(self):
    return (self.right(), self.top())

  def bottomLeft(self):
    return (self.left(), self.bottom())

  def bottomRight(self):
    return (self.right(), self.bottom())
  
  def intersection(self, other):
    left   = max(self.left(), other.left())
    right  = min(self.right(), other.right())
    top    = max(self.top(), other.top())
    bottom = min(self.bottom(), other.bottom())
    
    if left >= right or top >= bottom:
      return Rectangle(0, 0, 0, 0)
    
    return Rectangle(
      xc=(left+right)/2,
      yc=(top+bottom)/2,
      w=(right-left),
      h=(bottom-top)
    )

  def iou(self, other):
    intersection = self.intersection(other).area()
    if intersection == 0:
      return 0

    assert intersection >= 0
    assert intersection <= self.area() + 1e-7
    assert intersection <= other.area() + 1e-7

    return intersection / (self.area() + other.area() - intersection)
  
  def unionRect(self, other):
    left = min(self.left(), other.left())
    right = max(self.right(), other.right())
    top = min(self.top(), other.top())
    bottom = max(self.bottom(), other.bottom())
    
    return Rectangle(
      xc=(left+right)/2,
      yc=(top+bottom)/2,
      w=(right-left),
      h=(bottom-top)
    )

  def __repr__(self):
    return u"[%dx%d+%d+%d] (+%d+%d)" % (self.w, self.h, self.xc, self.yc, self.xc - 0.5*self.w, self.yc - 0.5*self.h)

def find_closest_rectangle(rectangles, key):
  minDist = None
  minRect = None

  for rect in rectangles:
    dist = (key.xc - rect.xc)**2 + (key.yc - rect.yc)**2
    if not minDist or dist < minDist:
      minDist = dist
      minRect = rect
  
  return minRect
