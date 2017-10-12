# -*- coding: utf-8 -*-
from astar_node import AStarNode
import graph_helper
import priority_queue

# Check if the node is the goal state
def is_goal_state(current_node, goal_node):
    return current_node.vertex == goal_node

# Construct the route based on the node parent chain.
def construct_route(node):
    route = []
    edge_ids = []
    route_cost = node.route_cost

    # While there is a node left.
    while(node is not None):
        
        route.insert(0, node.vertex)
        if node.edge != None:
            edge_ids.insert(0, node.edge)

        node = node.parent
    
    return route, edge_ids, route_cost

# Check if the node is in the explored node list.
def is_explored_node(current_node, explored_nodes):
    found_explored = False

    for explored_node in explored_nodes:
        if explored_node.vertex == current_node.vertex:
            found_explored = True

    return found_explored

# Check if the node is in the unexplored node list, and get the route cost if it is in the unexplored node list.
def is_unexplored_node(current_node, goal_node, unexplored_nodes):
    is_existing_unexplored_node = False
    current_unexplored_route_cost = 0

    for (_, _, unexplored_node) in unexplored_nodes.heap:
        if unexplored_node.vertex == current_node.vertex:
            is_existing_unexplored_node = True
            current_unexplored_route_cost = ((unexplored_node.route_cost + unexplored_node.vertex.euclidean_distance(goal_node))) #  + amount(open_list_item[2].state, open_list_item[2].parent.state)
            break

    return is_existing_unexplored_node, current_unexplored_route_cost

# Find the route using the waternet_graph, 
def astar_search(waternet_graph, start_vertex, goal_vertex):

    current_node = AStarNode(start_vertex, None, None, 0)

    # Check if the start is the goal state
    if is_goal_state(current_node, goal_vertex):
        return construct_route(current_node)
    		
    # Priority queue initialised with node and empty explored list
    unexplored_nodes = priority_queue.PriorityQueue() #priority doesn't matter, right?
    unexplored_nodes.push(current_node, 0)

    explored_nodes = []
    
    # Perform AStar search until the unexplored nodes is empty.
    while not unexplored_nodes.isEmpty():
        
        # Pop the current_node of the unexplored node list.
        current_node = unexplored_nodes.pop()
        explored_nodes.append(current_node)

        # Check if this current node is the goal node
        if is_goal_state(current_node, goal_vertex):
            return construct_route(current_node)            

        # Get the neighbours of the current_node
        for current_neighbour, edge_id in waternet_graph.neighbours(current_node.vertex):
            #value = graph_helper.euclidean_distance(node.state, current_neighbour)*1000
            current_edge = waternet_graph.edge(edge_id)

            if isinstance(edge_id, basestring):
                edge_id = int(edge_id.split("_")[0])
            current_neighbour = AStarNode(current_neighbour, edge_id, current_node, (current_node.route_cost + current_node.vertex.euclidean_distance(current_neighbour) * current_edge.cost_function())) #initialise node with actual distance, successor pathcost + entire path cost
            
            # Check if the neighbour is in the explored explored nodes
            if is_explored_node(current_neighbour, explored_nodes):
                continue

            # Check if the neighbour already exists in the explored nodes list.
            is_unexplored_neighbour, current_unexplored_route_cost = is_unexplored_node(current_neighbour, goal_vertex, unexplored_nodes)

            #  + amount(child.state, child.parent.state)
            distance = current_neighbour.route_cost

            # Check if it is not an unexplored node
            if not is_unexplored_neighbour:
                unexplored_nodes.push(current_neighbour, distance)

            elif current_neighbour.route_cost < current_unexplored_route_cost:
                # Replace in frontier heap only if the 
                for unexplored_node in unexplored_nodes.heap:
                    # Only check if the state of the unexplored node is equal to the current neighbour state.
                    if(unexplored_node[2].vertex == current_neighbour.vertex):
                        unexplored_nodes.heap.remove(unexplored_node)
                        unexplored_nodes.push(current_neighbour, distance)
                        break

    return False
