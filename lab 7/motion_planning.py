import math
def neighbors(current):
    # define the list of 4 neighbors
    
    neighbors = [[0, 1], [0,-1], [1, 0], [-1,0]]        #0 = up; 1 = down; 2 = right; 3 = left
    return [ (current[0]+nbr[0], current[1]+nbr[1]) for nbr in neighbors ] #nbr going to each set of coordinates in neighbors; 
                                                                           #setting up the neighbors (top,down,right,left) at the current point 

def heuristic_distance(candidate, goal):
    #can do either euclidean/ manhattan method 
    distancex = abs(goal[0] - candidate[0])
    distancey = abs(goal[1] - candidate[1])
    distance = math.sqrt(distancex**2 + distancey**2)
    
    return distance
    # each grid is the cost of 1 

def get_path_from_A_star(start, goal, obstacles):
    # input  start: integer 2-tuple of the current grid, e.g., (0, 0)
    #        goal: integer 2-tuple  of the goal grid, e.g., (5, 1)
    #        obstacles: a list of grids marked as obstacles, e.g., [(2, -1), (2, 0), ...]
    #if goal = obstacles, in the case that the goal is in the obstacle, break the code with the raise function;
    #this is put before the the open list 


    open_list = []  #(list)
    
    # 0 is cost and 1 is the candidate (below); start is 2 elements
    open_list.append((0,start)) # adds an object at the end of open_list
    closed_list = []
    past_cost = {}
    past_cost[start] = 0
    parent = {} #parent is the shortest path from goal to start; dict
    
    #set all rest of nodes to infinity 
    while len(open_list) > 0:
        open_list.sort()
        #current = open_list.pop(0) #popping in the 2nd element; careful with openlist 2 element
        current = open_list.pop(0)[1]
        #print(current)
        # add current to closed_list; use append function
        closed_list.append(current)
        #check if current is goal 
        if current == goal:
            #return #path  
            break
        #if current is goal, stop code 
        for candidate in neighbors(current):
            #check that candidate is not obstacle
            #print(candidate)
            if candidate in obstacles:
                continue # pushes to next candidate in the iteration 
            if candidate not in closed_list:
                new_cost = past_cost[current] + 1 
                #print(new_cost)
                
                # checks if past cost exists, if not, assigns past cost
                if candidate not in past_cost or new_cost < past_cost[candidate]: # conditions: if past cost does not exist, or if it is greater then new cost
                    
                    past_cost[candidate] = new_cost
                    #get heursitic distance and call heuristic distance function

                    # line 14 is a dict 
                    parent[candidate] = current
                    final = heuristic_distance(candidate,goal) + past_cost[candidate] #final is cost
                    open_list.append((final,candidate))    #tuple
            #sort open_list from lowest to highest path
            
            #est_total_cost = past_cost[candidate] + heuristic_distance(candidate,goal)
            
    #Side note: if condition for checking if the robot is blocked; 3 of its neigbors are walls
    path = []
    print(current)
    # this while loop is going from the goal to start to create the path
    while current != start: #start not included in path 
        path.append(current)
        current = parent[current]
        #reverse path 

    path.reverse()



    # output path: a list of grids connecting start to goal, e.g., [(1, 0), (1, 1), ...]
    #   note that the path should contain the goal but not the start
    #   e.g., the path from (0, 0) to (2, 2) should be [(1, 0), (1, 1), (2, 1), (2, 2)] 
    return path


#open_list = all evaluated candidates so all possible neighbors; use list
#closed_list = all visit candidates; use list/dict
#past_cost = costs ofall visited candidates following the conditions (C); use dict

#append start to open list
#set past_cost of the first element to 0
    #to assign empty dict ; past_cost={}
    #past_cost[start]=0
    #to initialize or start a dict use : past_cost{(start:0)}
#set all rest of nodes to infinity
    #have to change number of nodes since we don't know how many to go from start to goal
    #no action required

    #green is code
    #blue is arguments
    #red is psuedocode
# from motion_planning import get_path_from_A_star, test in separate file 

#if __name__ == '__main__':
#    start = (0, 0) # this is a tuple data structure in Python initialized with 2 integers
#    goal = (-5, -2)
#    obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
#    path = get_path_from_A_star(start, goal, obstacles)
#    print(path)
