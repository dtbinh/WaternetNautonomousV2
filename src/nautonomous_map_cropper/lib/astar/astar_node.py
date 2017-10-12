
# AStar node
class AStarNode:
    def __init__(self, vertex_value, parent_value, cost_value):
        self.vertex = vertex_value
        self.parent = parent_value
        self.route_cost = cost_value
    
    def __repr__ (self):
        return str(self.vertex) + " " + str(self.parent) + " " + str(self.route_cost)