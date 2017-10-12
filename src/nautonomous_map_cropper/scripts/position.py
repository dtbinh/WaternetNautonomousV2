from enum import Enum

class Position(Enum):
    TOPLEFT = 0, # only allow positive x and positive y values
    CENTER = 1 # allows positive and negative values for x and y

