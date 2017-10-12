
# Vaarkaart Graph to store the vertices and edges of the graph.
class WaternetGraph:

    def __init__(self):
        self.edges_dict_ = {}
        self.vertices_dict_ = {}
    
    ################## EDGES ##################

    # Add a new edge an add it to the dictionary using its id as key.
    def add_edge(self, edge):
        self.edges_dict_[edge.id()] = edge

    # Remove edge (if it exists) 
    def remove_edge(self, edge):
        if(edge.id() in self.edges_dict_):
            del self.edges_dict_[edge.id()]

    # Request an edge based on its id.
    def edge(self, edge_id):
        return self.edges_dict_[edge_id]

    # Return the edges
    def edges(self):
        return self.edges_dict_.values()
    
    ################## VERTICES ##################

    # add a new vertex if it is not already in the set of keys for the dictionary
    def add_vertex(self, vertex):
        if(not str(vertex) in self.vertices_dict_):
            self.vertices_dict_[str(vertex)] = vertex

    # Get the vertex from the dictionary.
    def vertex(self, vertex_str):
        return self.vertices_dict_[vertex_str]

    # Get all the keys from the vertices dictionary
    def vertices(self):
        return self.vertices_dict_.values()

    ################## NEIGHBOURS ##################

    # Initialize the neighbours by connecting the vertices to edge ids using the list of edges that they are connected to.
    def initialize_neighbours(self):    
        for edge in self.edges():
            self.vertices_dict_[edge.start_vertex_str()].add_edge_id(edge.id())
            self.vertices_dict_[edge.destination_vertex_str()].add_edge_id(edge.id())

    # Get the neighbour of the vertex by checking the connected edges from the vertex and filter out vertices that contradict the directionality rule.
    def neighbours(self, vertex):
        neighbours = []
        vertex = self.vertex(str(vertex))
        
        for edge_id in vertex.edge_ids():
            edge = self.edge(edge_id)

            start_vertex_str = edge.start_vertex_str()
            destination_vertex_str = edge.destination_vertex_str()

            # directionality indicates are on a 0 (start <-> destination) or 1 (start -> destination) identified road
            if start_vertex_str == str(vertex) and edge.metadata().directionality() != 2: 
                neighbours.append((self.vertex(destination_vertex_str), edge_id)) 
            # directionality indicates are on a 0 (start <-> destination) or 2 (start <- destination) identified road
            elif destination_vertex_str == str(vertex) and edge.metadata().directionality() != 1: 
                neighbours.append((self.vertex(start_vertex_str), edge_id))
                
        return neighbours

    ################## String ##################

    def __str__(self):
        return str(self.edges_dict_) + " " + str(self.vertices_dict_)