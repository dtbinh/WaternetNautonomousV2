import os

import json
import urllib

import graph_adapter

project_path = os.path.abspath(os.path.join(os.path.split(__file__)[0], os.pardir))
vaarkaart_intersection_url = project_path + '/../data/intersections.json'

vaarkaart_traffic_url = "https://grachten.waternet.nl/api/v2/trafficdata"

# Load the vaarkaart intersections data file.
def load_vaarkaart_intersections():
    global vaarkaart_intersection_url

    with open(vaarkaart_intersection_url) as vaarkaart_intersections_data:    
       return json.load(vaarkaart_intersections_data)
    
    return None

# Load the vaarkaart traffic url. (currently disabled to reduce the loading time)
def load_vaarkaart_traffic():
    global vaarkaart_traffic_url

    #vaarkaart_traffic_response = urllib.urlopen(vaarkaart_traffic_url)

    #return json.loads(vaarkaart_traffic_response.read())

    return None

# Load the vaarkaart and return the graph
def load_vaarkaart():
    vaarkaart_intersections = load_vaarkaart_intersections()
    vaarkaart_traffic = load_vaarkaart_traffic()

    vaarkaart_graph = graph_adapter.create_graph(vaarkaart_intersections, vaarkaart_traffic)

    return vaarkaart_graph