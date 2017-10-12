
from waternet_edge import WaternetEdge
from waternet_graph import WaternetGraph
from vaarkaart_metadata import VaarkaartMetadata

import utm_helper

# Get two edges (first and last) from the entire list of positions from the vaarkaart edge.
def get_start_and_destination_vertex(positions_json):
    length_json = len(positions_json)
        
    # Only take the first and last position of the positions
    start_vertex = utm_helper.convert_GPS_json_to_UTM_position(positions_json[0])
    destination_vertex = utm_helper.convert_GPS_json_to_UTM_position(positions_json[length_json-1])

    return start_vertex, destination_vertex

# Prepare the graph using the vertexes and edges from the vaarkaart.
def prepare_graph(vaarkaart_graph, json_array):

    for json_element in json_array:
        # retrieve the start and destination of a given segment.
        start_vertex, destination_vertex = get_start_and_destination_vertex(json_element["positions"])

        # add the vertices to the list if they are not already in that list.
        if start_vertex not in vaarkaart_graph.vertices():
            vaarkaart_graph.add_vertex(start_vertex)

        if destination_vertex not in vaarkaart_graph.vertices():
           vaarkaart_graph.add_vertex(destination_vertex)

        # create the edge based on the start and destination
        metadata = VaarkaartMetadata(json_element["info"]["name"], json_element["metadata"]["NAME"], json_element["directionality"], json_element["metadata"]["ROADTYPEAB"], json_element["metadata"]["SPEEDAB"], json_element["metadata"]["CAPACITYAB"])
        edge = WaternetEdge(json_element["id"], str(start_vertex), str(destination_vertex), metadata)

        vaarkaart_graph.add_edge(edge)

    return vaarkaart_graph

# Create a graph using the prepared graph
def create_graph(vaarkaart_intersections, vaarkaart_traffic):

    vaarkaart_graph = WaternetGraph()

    vaarkaart_graph = prepare_graph(vaarkaart_graph, vaarkaart_intersections["data"])

    return vaarkaart_graph

# Create a search graph by inserting the start and destination vertex if necessary and rewire edges that were destroyed.
def create_search_graph(search_graph, start_edge, start_vertex, destination_edge, destination_vertex):

    # If the start vertex is not in the search graph we have to split remove the graph and add the start vertex and connect with two new edges.
    if start_vertex not in search_graph.vertices():
        search_graph.remove_edge(start_edge)

        search_graph.add_vertex(start_vertex)

        search_graph.add_edge(WaternetEdge(str(start_edge.id())+"_1", start_edge.start_vertex_str(), str(start_vertex), start_edge.metadata()))
        search_graph.add_edge(WaternetEdge(str(start_edge.id())+"_2", str(start_vertex), start_edge.destination_vertex_str(), start_edge.metadata()))

    # If the destination vertex is not in the search graph we have to split remove the graph and add the destination vertex and connect with two new edges.
    if destination_vertex not in search_graph.vertices():
        search_graph.remove_edge(destination_edge)

        search_graph.add_vertex(destination_vertex)

        search_graph.add_edge(WaternetEdge(str(destination_edge.id())+"_1", destination_edge.start_vertex_str(), str(destination_vertex), destination_edge.metadata()))
        search_graph.add_edge(WaternetEdge(str(destination_edge.id())+"_2", str(destination_vertex), destination_edge.destination_vertex_str(), destination_edge.metadata()))

    # Initialize the neighbours using the currently connected edges and vertices.
    search_graph.initialize_neighbours()

    return search_graph
