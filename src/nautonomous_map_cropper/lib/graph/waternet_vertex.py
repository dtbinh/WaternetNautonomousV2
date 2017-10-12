from point import Point

# Vaarkaart Vertex to store the point and references to edges.
class WaternetVertex(Point):

    def __init__(self, x, y):

        Point.__init__(self, x, y)

        self.edges_id_ = []
    
    # Return the point of the vertex
    def point(self):
        return Point(self.x_, self.y_)
    
    # Link a new edge to the vertex
    def add_edge_id(self, edge_id):
        if edge_id not in self.edges_id_:
            self.edges_id_.append(edge_id)
   
    # Return the edge ids.
    def edge_ids(self):
        return self.edges_id_

    # Check if the vertex is the same by checking if the x and y coordinate are the same.
    def __eq__(self, other):
        return self.x() == other.x() and self.y() == other.y()

    # Print the point
    def __repr__(self):
        return Point.__repr__(self) 
