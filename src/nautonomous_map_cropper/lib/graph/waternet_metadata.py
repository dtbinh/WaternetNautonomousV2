
# Vaarkaart Metadata to store the speed and directionality of the edge.

class WaternetMetadata:
    def __init__(self, speed, directionality):
        self.speed_ = speed
        self.directionality_ = directionality
    
    # Return the speed that is acceptable on an edge
    def speed(self):
        return self.speed_

    # Return the directionality of an edge
    # 0: start <-> destination
    # 1: start --> destination
    # 2: start <-- destination 
    def directionality(self):
        return self.directionality_