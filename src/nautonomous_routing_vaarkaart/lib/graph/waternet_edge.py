
# Vaarkaart Vertex to store the node, adjacent nodes and the connected nodes.
class WaternetEdge:
    def __init__(self, id, start_vertex_str, destination_vertex_str, metadata):
        self.id_ = id

        self.start_vertex_str_ = start_vertex_str
        self.destination_vertex_str_ = destination_vertex_str
       
        self.metadata_ = metadata

    # Return the id
    def id(self):
        return self.id_

    # Get the string of the start vertex
    def start_vertex_str(self):
        return self.start_vertex_str_

    # Get the string of the end vertex
    def destination_vertex_str(self):
        return self.destination_vertex_str_

    # Get the metadata of the edge
    def metadata(self):
        return self.metadata_

    # Cost function to scale the astar search 
    def cost_function(self):
        # inverse the speed so the search function can multiply the distance with the inverse of the speed (m/s) to distance (m) * inverse speed (s/m)
   
        # formula (1.0 / (self.metadata_.speed() / 3.6) # example 7.5 km/h -> 2.08 m/s take the inverse of this is 0.48 s/m
        # simplified (3.6 / self.metadata_.speed())
        return (3.6 / self.metadata_.speed())
        
    def __repr__(self):
        return str(self.id()) + " " + str(self.start_vertex_str()) + " " + str(self.destination_vertex_str())