import heapq


class State:
    """
    Class to represent a state on grid-based pathfinding problems. The class contains two static variables:
    map_width and map_height containing the width and height of the map. Although these variables are properties
    of the map and not of the state, they are used to compute the hash value of the state, which is used
    in the CLOSED list. 

    Each state has the values of x, y, g, h, and cost. The cost is used as the criterion for sorting the nodes
    in the OPEN list for both Dijkstra's algorithm and A*. For Dijkstra the cost should be the g-value, while
    for A* the cost should be the f-value of the node.
    """
    map_width = 0
    map_height = 0
    
    def __init__(self, x, y):
        """
        Constructor - requires the values of x and y of the state. All the other variables are
        initialized with the value of 0.
        """
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        
    def __repr__(self):
        """
        This method is invoked when we call a print instruction with a state. It will print [x, y],
        where x and y are the coordinates of the state on the map. 
        """
        state_str = "[" + str(self._x) + ", " + str(self._y) + "]"
        return state_str
    
    def __lt__(self, other):
        """
        Less-than operator; used to sort the nodes in the OPEN list
        """
        return self._cost < other._cost
    
    def state_hash(self):
        """
        Given a state (x, y), this method returns the value of x * map_width + y. This is a perfect 
        hash function for the problem (i.e., no two states will have the same hash value). This function
        is used to implement the CLOSED list of the algorithms. 
        """
        return self._y * State.map_width + self._x
    
    def __eq__(self, other):
        """
        Method that is invoked if we use the operator == for states. It returns True if self and other
        represent the same state; it returns False otherwise. 
        """
        return self._x == other._x and self._y == other._y

    def get_x(self):
        """
        Returns the x coordinate of the state
        """
        return self._x
    
    def get_y(self):
        """
        Returns the y coordinate of the state
        """
        return self._y
    
    def get_g(self):
        """
        Returns the g-value of the state
        """
        return self._g
        
    def set_g(self, g):
        """
        Sets the g-value of the state
        """
        self._g = g

    def get_cost(self):
        """
        Returns the cost of a state; the cost is determined by the search algorithm
        """
        return self._cost
    
    def set_cost(self, cost):
        """
        Sets the cost of the state; the cost is determined by the search algorithm 
        """
        self._cost = cost

def dijkstra_search(start, goal, map):
    """
    Dijkstra's algorithm for grid-based pathfinding.

    Parameters:
    start (State): The intitial state.
    goal (State): The goal state.
    map (Map): The grid map object.

    Returns:
    Tuple[int, int]: A tuple containing the cost of the path and the number of expanded nodes.
                     If no path is found, returns (-1, expanded_nodes).
    """
    OPEN = []               # Priority queue for open states
    CLOSED = {}             # Dictionary for closed states (explored states)
    expanded_nodes = 0
   
    # Initialize starting state
    start.set_cost(start.get_g())
    heapq.heappush(OPEN, start)
    CLOSED[start.state_hash()] = start
 
    while len(OPEN) != 0:
        node = heapq.heappop(OPEN)
        if node == goal:
            return node.get_g(), expanded_nodes # Goal reached, return cost and expanded nodes

        children = map.successors(node)
        expanded_nodes += 1

        for child in children:
            child.set_cost(child.get_g())

            if child.state_hash() not in CLOSED:
                CLOSED[child.state_hash()] = child
                heapq.heappush(OPEN, child)
                
            elif child.state_hash() in CLOSED and child < CLOSED[child.state_hash()]:
                CLOSED[child.state_hash()] = child
                heapq.heappush(OPEN, child)

    return -1, expanded_nodes # If no path is found

def heuristic(current, goal):
    """
    Heuristic function for A* algorithm.

    Parameters:
    current (State): The current state.
    goal (State): The goal state.

    Returns:
    The heuristic estimate of the cost from the current state to the goal state.
    """
    dx = abs(current.get_x() - goal.get_x())
    dy = abs(current.get_y() - goal.get_y())

    return 1.5 * min(dx, dy) + abs(dx - dy)

def Astar_search(start, goal, map):
    """
    A* algorithm for grid-based pathfinding.

    Parameters:
    start (State): The initial state.
    goal (State): The goal state.
    map (Map): The grid map object.

    Returns:
    Tuple[float, int]: A tuple containing the cost of the path and the number of expanded nodes.
                       If no path is found, returns (-1, expanded_nodes).
    """
    OPEN = []               # Priority queue for open states
    CLOSED = {}             # Dictionary for closed states (explored states)
    expanded_nodes = 0

    # Initialize starting state along with heuristic cost
    start.set_cost(start.get_g() + heuristic(start, goal))
    heapq.heappush(OPEN, start)
    CLOSED[start.state_hash()] = start

    while len(OPEN) != 0:
        node = heapq.heappop(OPEN)
        if node == goal:
            return node.get_cost(), expanded_nodes # Goal reached, return cost and expanded nodes
        
        children = map.successors(node)
        expanded_nodes += 1

        for child in children:
            h = heuristic(child, goal)
            f = child.get_g() + h
            child.set_cost(f)

            if child.state_hash() not in CLOSED:
                heapq.heappush(OPEN, child)
                CLOSED[child.state_hash()] = child

            temp = CLOSED[child.state_hash()]
            if child.state_hash() in CLOSED and child < temp:
                CLOSED[child.state_hash()] = child
                heapq.heappush(OPEN, child)

    return -1, expanded_nodes # If no path is found