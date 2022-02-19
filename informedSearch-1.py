
import random
import heapq

# Node class to hold the location and parent node (needed for getting final path)
class Node:
 
    def __init__(self, value, par):
        self.value = value
        self.parent = par
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0

    #set path cost g(n)
    def setG(self, gCost):
        self.g = self.parent.g + gCost
 
    #set sum of g(n) + h(n)
    def setF(self):
        self.f = self.g + self.h

    
    # Define < operator so that we can use heapq library
    def __lt__(self, other):
        
        # If f is set then we're using A* - compare with f
        if self.f > 0:
            return self.f < other.f
        
        # Otherwise we're using greedy - compare with h
        else:
            return self.h < other.h


# The grid values must be separated by spaces, e.g.
# 1 1 1 1 1 
# 1 0 0 0 1
# 1 0 0 0 1
# 1 1 1 1 1
# Returns a 2D list of 1s and 0s
def readGrid(filename):
	#print('In readGrid')
	grid = []
	with open(filename) as f:
		for l in f.readlines():
			grid.append([int(x) for x in l.split()])
	
	f.close()
	#print 'Exiting readGrid'
	return grid


# Writes a 2D list of 1s and 0s with spaces in between each character
# It will change the start location to 'S', goal to 'G', and path points to '*'
# 1 1 1 1 1 
# 1 S * * 1
# 1 0 0 G 1
# 1 1 1 1 1
def outputGrid(grid, start, goal, path):
	#print('In outputGrid')
	filenameStr = 'informedPath.txt'

	# Open filename
	f = open(filenameStr, 'w')

	# Mark the start and goal points
	grid[start[0]][start[1]] = 'S'
	grid[goal[0]][goal[1]] = 'G'

	# Mark intermediate points with *
	for i, p in enumerate(path):
		if i > 0 and i < len(path)-1:
			grid[p[0]][p[1]] = '*'

	# Write the grid to a file
	for r, row in enumerate(grid):
		for c, col in enumerate(row):
			
			# Don't add a ' ' at the end of a line
			if c < len(row)-1:
				f.write(str(col)+' ')
			else:
				f.write(str(col))

		# Don't add a '\n' after the last line
		if r < len(grid)-1:
			f.write("\n")

	# Close file
	f.close()
	#print('Exiting outputGrid')


# Generates a random grid
def genGrid():
    print('In genGrid')
    
    num_rows = 10
    num_cols = 10
    
    grid = [[0]*num_cols for i in range(0,num_rows)]
    
    max_cost = 5
    ob_cost = 0
    
    for i_r in range(0,num_rows):
        for i_c in range(0,num_cols):
            
            # Default to obstacle cost
            cost = ob_cost
            
            # Chance to be an obstacle
            chance = random.random()
            if chance > 0.2:
                # Generate a random cost for the location
                cost = random.randint(1,max_cost)
                
            grid[i_r][i_c] = cost

    return grid

def printGrid(grid):
    for i in range(len(grid)):
        print(grid[i])

# Returns false if NOT in the openList
# or Returns the existing node to check for replacement
def InOpenList(node, openList):
    #print('openList: %s' % openList)
    for n in openList:
        if n.value == node.value:
            print('node %s is equal to existing node in open list %s' % (node.value, n.value))
            print('node.g: %s n.g: %s' % (node.g, n.g))
            return n
    return False
 
# Returns true if in the closed list
def InClosedList(node, closedList):
    for n in closedList:
        if n.value == node.value:
            return True
    return False

def printNodeList(l):
    for node in l:
        print(node.value)

def heuristic(node, goal):
    
    goaln = Node(goal, '')
    #manhattan distance
    # |x1-x2|+|y1-y2|
    hDist = abs(node.value[0] - goaln.value[0]) + abs(node.value[1] - goaln.value[1])

    return hDist



# Returns all adjacent locations for node that are free space
def getNeighbors(location, grid):
    #print('In getNeighbors')

    result = []

    # Use location[:] to get a copy of the list
    # For each direction (u,r,d,l), check the bounds and value on the grid
    # Clockwise order -> u, r, d, l

    up = location[:]
    up[0] -= 1
    if up[0] > 0 and grid[up[0]][up[1]] != 0:
        result.append(up)

    right = location[:]
    right[1] += 1
    if right[1] < len(grid[right[0]]) and grid[right[0]][right[1]] != 0:
        result.append(right)

    down = location[:]
    down[0] += 1
    if down[0] < len(grid) and grid[down[0]][down[1]] != 0:
        result.append(down)

    left = location[:]
    left[1] -= 1
    if left[1] > 0 and grid[left[0]][left[1]] != 0:
        result.append(left)

    #print('Exiting getNeighbors')
    return result



# Gets all children for node and adds them to the openList if possible
def expandNode(current, openList, closedList, goal, grid, algoType):
    # Get neighbors
    neighbors = getNeighbors(current.value, grid)
 
    # For each neighbor, create node, check if duplicate, and push on
    for n in neighbors:
 
        # Create new node
        nd = Node(n, current)
 
        # Set node values
        nd.setG(grid[n[0]][n[1]])
        nd.h = heuristic(nd, goal)
        
        # If greedy search, add to open list without checking closed list
        if algoType == 'greedy':
 
            # Push onto open list
            heapq.heappush(openList, nd)
 
        # else if astar
        else:
            # Set f value
            nd.setF()
 
            # Check if it's in the open list
            # only do this in a* because in greedy the node cost will always be the same
            inOpen = InOpenList(nd, openList)
 
            # If it is in the open list, compare the new node and existing node path costs
            if inOpen:
 
                # If the new node is better
                if nd.g < inOpen.g:
                    # Replace it in the open list
                    inOpen = nd
                    heapq.heapify(openList)
 
                    # If it is in the closed list too, remove it
                    inClosed = InClosedList(nd, closedList)
                    if inClosed:
                        closedList.remove(nd)
 
            # Else not in closed list, push the new node onto the open list
            elif not InClosedList(nd, closedList):
                heapq.heappush(openList, nd) 



# Sets the path variable by inserting each node on the current node's path
def setPath(current, path):
    #print('In setPath')

    # While not at the root, append each node's parent
    while current.parent != '':
        path.insert(0, current.parent.value)
        current = current.parent

    #print('Exiting setPath')


def informedSearch(type, grid, start, goal):
    #print('\nIn uninformedSearch')
    print('\nStarting search, type: %s, start: %s, goal: %s' % (type, start, goal))

    # Set initial variables
    current = Node(start, '')
    path = []
    # List of nodes in the open list
    openList = []

    # Initially, push the root node onto the open list
    heapq.heappush(openList, current)
    #openListCopy.append(current)

    # List of expanded nodes
    closedList = []

    # Track the number of expanded nodes
    numExpanded = 0


    ###############################
    ###        Main Loop        ###
    ###############################

    # While we are not at the goal and we haven't expanded all nodes
    while len(openList) != 0: 

        # Pop off open list
        current  = heapq.heappop(openList)

        # Add to closed list
        closedList.append(current)

        # Check for goal
        if current.value == goal:
            break
        
        else:
            # Expand this node
            expandNode(current, openList, closedList, goal, grid, type)

            # Data
            numExpanded += 1

    pathCost = current.g
    
    # If we found the goal, then build the final path
    if len(openList) != 0 or current == goal:

        # Set the path variable
        setPath(current, path)

        # Append the goal because setPath doesn't add that
        path.append(goal)

    return [path, pathCost, numExpanded]


def main():

    #randomG = genGrid()
    #printGrid(randomG)
    start = [0,0]
    goal = [2,3]

    grid = readGrid('informedGrid.txt')
    printGrid(grid)
    
    algorithm = input('Input \'astar\' or \'greedy\' \n')

    if algorithm != 'astar' and algorithm != 'greedy':
        print('Invalid Input')
    else:
        [path, pathCost, numExpanded] = informedSearch(algorithm, grid, start, goal)

        print('Results: ')
        print('\nFinal Path: %s' %path)
        print('\nPath Cost: %s' %pathCost)
        print('\nExpanded Nodes: %s' %numExpanded)

        outputGrid(grid, start, goal, path)
    print("Heuristic used: Manhattan Distance")

if __name__ == '__main__':
    main()
    print('\nExiting normally')