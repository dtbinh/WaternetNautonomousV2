tree: The tree search algorithm uses a fixed step size and follows the most optimal branch
grid: Used a fixed grid, pushes every node to the back and checks whether the new node already exists
	Timeing tests for grid_1:
		Check if position already exists (check 1): 0.793
		Plotting: 0.43
		Creating 3 new nodes (check 2): 0.824 (0.031 for the decision making)
		Initialization:	0.011
		Finding the optimal node: 0.230

		Total of 1.51 on average

grid_2: Uses a fixed grid but also assigns a Network node for every grid position

tree_search: The tree search algorithm uses a fixed step size and finds the most optimal branch
