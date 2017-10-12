from waternet_metadata import WaternetMetadata

# Meta data from the vaarkaart
class VaarkaartMetadata(WaternetMetadata):
    def __init__(self, name, name_code, directionality, road_type, speed, capacity):
        WaternetMetadata.__init__(self, speed, directionality)
        
        self.name_ = name
        self.name_code_ = name_code
        self.road_type_ = road_type
        self.capacity_ = capacity

