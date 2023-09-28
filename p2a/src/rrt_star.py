#RRT* implementation in 3D 

# import matplotlib.pyplot as plt
import numpy as np


# Class for each tree node
class Node:
    def __init__(self, x, y, z): 
        self.x = x        # coordinate
        self.y = y        # coordinate
        self.z = z        #coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost
    
    def __str__(self):
        return f"Node(x={self.x}, y={self.y}, z={self.z})  "


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, envi, start, goal):

        envi.make_env()            
        self.map_array = envi.get_map_array() # map array, 1->free, 0->obstacle
        self.size_x = self.map_array.shape[0]    # map size
        self.size_y = self.map_array.shape[1]    # map size
        self.size_z = self.map_array.shape[2]    # map size

        self.start = Node(start[0], start[1], start[2]) # start node
        self.goal = Node(goal[0], goal[1], goal[2])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)
        self.map_array[self.start.x][self.start.y][self.start.z] = 2 

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        return ((node1.x - node2.x)**2 + (node1.y-node2.y)**2 + (node1.z-node2.z)**2)**0.5


    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles using Bresenham's line algorithm
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''

        x1, y1, z1 = node1.x, node1.y, node1.z
        x2, y2, z2 = node2.x, node2.y, node2.z 
        
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        dz = abs(z2 - z1)
        
        sx = 1 if x2 > x1 else -1
        sy = 1 if y2 > y1 else -1
        sz = 1 if z2 > z1 else -1
        
        if dx >= dy and dx >= dz:
            err_1 = 2*dy - dx
            err_2 = 2*dz - dx
            while x1 != x2 or y1 != y2 or z1 != z2:
                if not self.map_array[x1][y1][z1]:
                    return False
                if err_1 > 0:
                    y1 += sy
                    err_1 -= 2*dx
                if err_2 > 0:
                    z1 += sz
                    err_2 -= 2*dx
                x1 += sx
                err_1 += 2*dy
                err_2 += 2*dz
        elif dy >= dx and dy >= dz:
            err_1 = 2*dx - dy
            err_2 = 2*dz - dy
            while x1 != x2 or y1 != y2 or z1 != z2:
                if not self.map_array[x1][y1][z1]:
                    return False
                if err_1 > 0:
                    x1 += sx
                    err_1 -= 2*dy
                if err_2 > 0:
                    z1 += sz
                    err_2 -= 2*dy
                y1 += sy
                err_1 += 2*dx
                err_2 += 2*dz
        else:
            err_1 = 2*dy - dz
            err_2 = 2*dx - dz
            while x1 != x2 or y1 != y2 or z1 != z2:
                if not self.map_array[x1][y1][z1]:
                    return False
                if err_1 > 0:
                    y1 += sy
                    err_1 -= 2*dz
                if err_2 > 0:
                    x1 += sx
                    err_2 -= 2*dz
                z1 += sz
                err_1 += 2*dy
                err_2 += 2*dx
        
        return True



    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        rand_num = np.random.uniform(0, 1) #generate a random number between 0 and 1 

        if rand_num <= goal_bias: # If the random number is less than goal bias 
            return [self.goal.x,self.goal.y,self.goal.z] #return goal 
        else:
            return [np.random.randint(0,self.size_x),np.random.randint(0,self.size_y),np.random.randint(0,self.size_z)] #return a random point in map_array

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        nearest_node = self.vertices[0] #initialize the first node in self.vertices to be the nearest node 
        sample_node = Node(point[0],point[1],point[2]) #make the point into a node

        best_dist = self.dis(nearest_node,sample_node) #initialize the best distance 

        for vertex in self.vertices: #loop through all vertices and change the best distance and the nearest node if closer vertex found
            vert_dist = self.dis(vertex,sample_node) 
            if(vert_dist < best_dist):
                nearest_node = vertex
                best_dist = vert_dist

        return nearest_node,best_dist #return nearest node to point 

    def steer(self,random_node,nearest_node,delta_q):
        """
        steer towards the random node from the nearest node in the existing tree 
        arguments: random node generated in the map, nearest node to that random node in the existing
                   tree, the incremental distance to steer towards the random node from the nearest node 
        return: the node obtained by steering 
        
        """

         # Check if the distance between random node and nearest node is less than the delta_q. If so, return the random node as the steered node
        if(self.dis(random_node,nearest_node)<delta_q): return random_node

        else:
            # Extract the x, y and z values for the nearest and random nodes
            x1 = nearest_node.x
            y1 = nearest_node.y
            z1 = nearest_node.z 

            x2 = random_node.x 
            y2 = random_node.y
            z2 = random_node.z 

            node_dist = self.dis(random_node,nearest_node) # Calculate the distance between the random node and nearest node 

            # Calculate the vector in x and y direction towards the random node
            vec_x = delta_q*int((x2-x1)/node_dist) 
            vec_y = delta_q*int((y2-y1)/node_dist)
            vec_z = delta_q*int((z2-z1)/node_dist) 
            # Calculate the new coordinates of the node after steering towards the random node
            steered_x = x1 + vec_x
            steered_y = y1 + vec_y
            steered_z = z1 + vec_z  
            steered_node = Node(steered_x,steered_y,steered_z) # Create a new node with the steered coordinates and return it
            return steered_node 



    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors = []

        for vertex in self.vertices:
            if(self.dis(new_node,vertex)<neighbor_size):
                neighbors.append(vertex) 
        
        return neighbors 


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###

        for neighbor in neighbors:
            if([neighbor.x,neighbor.y,neighbor.z] != [new_node.parent.x,new_node.parent.y,new_node.parent.z]): #don't consider the parent of new node to rewire 
                if(new_node.cost + self.dis(new_node,neighbor) < neighbor.cost): #if cost of the neighbor > cost to the neighbor with new_node as its parent
                    if(self.check_collision(new_node,neighbor)):
                        #rewire the parent 
                        neighbor.parent = new_node
                        neighbor.cost = new_node.cost + self.dis(new_node,neighbor)



    def RRT_star(self, n_pts=1000, neighbor_size=20): 
        #if not seeing desired results correct the parameters: delta_q_star, goal_tolerance, best_dist threshold, neighbor_size, goal_tolerance, goal probability of self.get_new_point
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        print(self.start)
        print(self.goal)

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check ylision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        for sample_number in range(n_pts):

            random_sample = self.get_new_point(0.005) #get a new point 
            random_node = Node(random_sample[0],random_sample[1],random_sample[2]) #create a node for the random sample generated 


            delta_q_star = 400 #setting the incremental distance for moving from nearest vertex to random vertex (steering)
            goal_tolerance = 300 #a node within this distance is considered close enough to the goal 
            
            nearest_node,best_dist = self.get_nearest_node(random_sample) #get its nearest node to the random sample in the existing tree 
            # print(best_dist)
            while best_dist <= 100:
                random_sample = self.get_new_point(0.005) #get a new point 
                nearest_node,best_dist = self.get_nearest_node(random_sample) #get its nearest node
            

            sample_node = self.steer(random_node,nearest_node,delta_q_star) #steer towards the random node and return the corresponding node

            # print(sample_node)

            # for i in self.vertices: print(i)
            # print("\n")

            if(self.map_array[sample_node.x][sample_node.y][sample_node.z] == 0 or self.map_array[sample_node.x][sample_node.y][sample_node.z] == 2): continue #if the sample is in an obstacle or self.vertices

            neighbor_size = 500 
            neighbors = self.get_neighbors(sample_node,neighbor_size)
            if(not neighbors): continue 
            #get the node with the least cost in the neighbors found 
            best_neighbor = neighbors[0]
            best_neighbor_cost = best_neighbor.cost  
            for neighbor in neighbors: #TODO: put collision check inside this loop to get a node for sure that has no collision 
                if(neighbor.cost<best_neighbor_cost):
                    best_neighbor_cost = neighbor.cost
                    best_neighbor = neighbor
            
            if(self.check_collision(sample_node,best_neighbor)): #if the line between the sample node and nearest node has no obstacle 
                sample_node.parent = best_neighbor #update the parent
                sample_node.cost = best_neighbor.cost + self.dis(sample_node,best_neighbor) #update cost 
                self.rewire(sample_node,neighbors)
                self.vertices.append(sample_node) #add sample node to list of vertices 
                self.map_array[sample_node.x][sample_node.y][sample_node.z] = 2 #if a node is added to self.vertices, its corresponding val in map_array will be 2
                # print(sample_node)
                
                if(self.dis(sample_node,self.goal)< goal_tolerance): #if sample node is close enough to goal 
                    self.found = True 
                    self.goal.parent = sample_node #update goal parent 
                    self.goal.cost = sample_node.cost + self.dis(sample_node,self.goal) #update goal cost 
                    #keep iterating and optimizing the current path even after goal is found in RRT*

        
        path = []

        # Output
        if self.found:
            self.vertices.append(self.goal) #add goal to vertices list after the desired number of iterations 
            self.map_array[self.goal.x][self.goal.y][self.goal.z] = 2
            steps = len(self.vertices) - 2
            length = self.goal.cost
            # for i in self.vertices: print(i)
            # print("\n")
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)

            current_node = self.goal
            while current_node is not None:  # Traverse from the goal node to the start node
                path.append(current_node)
                current_node = current_node.parent  # move to the parent node
            
            path.reverse()  # Reverse the list to print from start to goal
            for node in path:
                print(node)  
            
            return path 
            
        else:
            print("No path found")
            return path 

