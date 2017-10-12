import os

import json
import urllib

import graph_adapter
import vaarkaart_segment

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))
vaarkaart_intersection_url = project_path + '/../data/intersections.json'

vaarkaart_traffic_url = "https://grachten.waternet.nl/api/v2/trafficdata"

# Load the vaarkaart intersections data file.
def load_vaarkaart_intersections():
    global vaarkaart_intersection_url

    with open(vaarkaart_intersection_url) as vaarkaart_intersections_data:    
       print "Opened vaarkaart successfully"
       return json.load(vaarkaart_intersections_data)

    print "could not open vaarkaart"
    return None

# Load the vaarkaart traffic url. (currently disabled to reduce the loading time)
def load_vaarkaart_traffic():
    global vaarkaart_traffic_url

    #vaarkaart_traffic_response = urllib.urlopen(vaarkaart_traffic_url)

    #return json.loads(vaarkaart_traffic_response.read())

    return None

# Load the vaarkaart and return the graph
def load_vaarkaart_graph():
    vaarkaart_intersections = load_vaarkaart_intersections()

    vaarkaart_graph = graph_adapter.create_graph(vaarkaart_intersections, None)

    return vaarkaart_graph

def load_vaarkaart_segments():
    vaarkaart_intersections = load_vaarkaart_intersections()

    vaarkaart_graph = vaarkaart_segment.create_segments(vaarkaart_intersections)

    return vaarkaart_graph