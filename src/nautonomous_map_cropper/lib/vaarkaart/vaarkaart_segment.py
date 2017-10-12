
from waternet_edge import WaternetEdge
from waternet_vertex import WaternetVertex
from waternet_graph import WaternetGraph
from graph_adapter import get_start_and_destination_vertex
from utm_helper import convert_GPS_json_to_UTM_pose2d
from nautonomous_map_msgs.srv import CropRequest

# Prepare the graph using the vertexes and edges from the vaarkaart.
def create_segments(json_array):
    crop_requests = {}
    data = json_array["data"]
    for json_element in data:

        positions = json_element["positions"]
        
        route = []
        for position in positions:
            route.append(convert_GPS_json_to_UTM_pose2d(position))

        route_id = json_element["id"]

        crop_requests[route_id] = CropRequest(route, "segment_" + str(route_id))

    return crop_requests