
# AStar node
class AStarNode:
    def __init__(self, vertex_value, edge_id, parent_value, cost_value):
        self.vertex = vertex_value
        self.edge = edge_id
        self.parent = parent_value
        self.route_cost = cost_value
    
    def __repr__ (self):
        return str(self.vertex) + " " + str(self.parent) + " " + str(self.route_cost)