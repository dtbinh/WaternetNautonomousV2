
import matplotlib.pyplot as plt
import networkx as nx

# Visualize vaarkaart based on the graph
def visualize_route(waternet_graph, start_vertex, destination_vertex, route_poses):
    node_colors = []

    # Create a visualization graph using nx.
    visualization_graph = nx.Graph()

    # Creates the nodes from the waternet graph
    for vertex in waternet_graph.vertices():
        visualization_graph.add_node(str(vertex), pos = (vertex.x(), vertex.y()))

    # Creates the edges from the waternet graph
    for edge in waternet_graph.edges():
        visualization_graph.add_edge(edge.start_vertex_str(), edge.destination_vertex_str())    
        
    # Color the vertices based on the node function.
    for node in visualization_graph.nodes():
        if node == str(start_vertex):
            node_colors.append("green")
        elif node == str(destination_vertex):
            node_colors.append("red")
        else:
            found = False
            utm_coordinate = map(float, node[1:-1].split(",")) # [1:-1]. removes the brackets (easting, northing) before converting to a float.

            for pose in route_poses:
                if(is_close(pose.x, utm_coordinate[0]) and is_close(pose.y, utm_coordinate[1])):
                    found = True
                    break

            if not found:
                node_colors.append("white")	
            else: 
                node_colors.append("blue")
    
    # Prepare the graph
    pos = nx.get_node_attributes(visualization_graph, 'pos')

    # Draw the graph
    nx.draw_networkx_nodes(visualization_graph, pos = pos, node_color = node_colors, node_size = 50)
    nx.draw_networkx_edges(visualization_graph, pos = pos)

    # Show the graph
    plt.show()
    
def is_close(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)