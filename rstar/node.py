import numpy as np

class Node:
    def __init__(self, state, parent, g, h):
        self.state = state
        self.parent = parent
        self.g = g
        self.h = h

    @property
    def f(self):
        return self.g + self.h

    def __hash__(self): #Necessary to allow for map of Nodes
        return hash(np.array_str(self.state))

    def __gt__(self, other): #Equality functions overriden to allow for comparison in min heap
        return self.f > other.f

    def __ge__(self, other):
        return self.f >= other.f

    def __lt__(self, other):
        return self.f < other.f

    def __le__(self, other):
        return self.f <= other.f

    def __eq__(self, other):
        return np.array_equal(self.state, other.state)

    def toString(self):
        return format("state: \n%s" %(self.state))

