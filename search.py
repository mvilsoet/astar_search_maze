# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021

"""
This is the main entry point for MP3. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)


# Feel free to use the code below as you wish
# Initialize it with a list/tuple of objectives
# Call compute_mst_weight to get the weight of the MST with those objectives
# TODO: hint, you probably want to cache the MST value for sets of objectives you've already computed...
# Note that if you want to test one of your search methods, please make sure to return a blank list
#  for the other search methods otherwise the grader will not crash.
from collections import deque
import heapq

class MST:
    def __init__(self, objectives):
        self.elements = {key: None for key in objectives}
        # implement some distance between two objectives
        # ... either compute the shortest path between them, or just use the manhattan distance between the objectives
        self.distances   = {
                (i, j): self.manhattan(i, j) for i, j in self.cross(objectives)
            }

    def manhattan(self, agent: tuple, waypoint: tuple):
        return abs(agent[0]-waypoint[0]) + abs(agent[1]-waypoint[1])

    # Prim's algorithm adds edges to the MST in sorted order as long as they don't create a cycle
    def compute_mst_weight(self):
        weight      = 0
        for distance, i, j in sorted((self.distances[(i, j)], i, j) for (i, j) in self.distances):
            if self.unify(i, j):
                weight += distance
        return weight

    # helper checks the root of a node, in the process flatten the path to the root
    def resolve(self, key):
        path = []
        root = key
        while self.elements[root] is not None:
            path.append(root)
            root = self.elements[root]
        for key in path:
            self.elements[key] = root
        return root

    # helper checks if the two elements have the same root they are part of the same tree
    # otherwise set the root of one to the other, connecting the trees
    def unify(self, a, b):
        ra = self.resolve(a)
        rb = self.resolve(b)
        if ra == rb:
            return False
        else:
            self.elements[rb] = ra
            return True

    # helper that gets all pairs i,j for a list of keys
    def cross(self, keys):
        return (x for y in (((i, j) for j in keys if i < j) for i in keys) for x in y)

class Node:
    pos = tuple((0,0)) #field() is initializer
    parent = None
    g_cost = 0
    total_cost = 0
    waypoints = [] #"THIS NODE IS A WP" or "parent is a wp"

    def __lt__(self, other): #for pq ordering
        return self.total_cost < other.total_cost

def manhattan(agent: tuple, waypoint: tuple) -> int:
    return abs(agent[0]-waypoint[0]) + abs(agent[1]-waypoint[1])
 
def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """

    start_node = Node()
    start_node.pos = maze.start
    frontier = deque([start_node])
    visited = {start_node.pos : start_node} #explored all neighbors, stuff that got popped goes here

    while frontier:
        curr_node = frontier.popleft()
        if maze[curr_node.pos] == maze.legend.waypoint:
            break #done with beefs
        
        for neighbor_pos in maze.neighbors(curr_node.pos[0],curr_node.pos[1]):
            if neighbor_pos not in visited:
                if maze.navigable(neighbor_pos[0], neighbor_pos[1]):

                    neighbor_node = Node()
                    neighbor_node.pos = neighbor_pos
                    neighbor_node.parent = curr_node.pos

                    frontier.append(neighbor_node)
                    visited[neighbor_node.pos] = neighbor_node
                    
    path = [curr_node.pos] #once while loop ends, curr_node is the waypoint
    parent = curr_node.parent
    while parent: #backtracing through the nodes to reach start
        path.insert(0, visited[parent].pos) #use the visited dict to find the pos of the parent
        parent = visited[parent].parent #move to next node in linked list

    return path

def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    start_node = Node()
    start_node.pos = maze.start
    frontier = []
    heapq.heappush(frontier, start_node)
    visited = {start_node.pos : start_node}

    while frontier:
        curr_node = heapq.heappop(frontier)

        if curr_node.pos == maze.waypoints[0]:
            break

        for neighbor_pos in maze.neighbors(curr_node.pos[0], curr_node.pos[1]):
            if neighbor_pos not in visited:
                if maze.navigable(neighbor_pos[0], neighbor_pos[1]):

                    neighbor_node = Node() 
                    neighbor_node.pos = neighbor_pos
                    neighbor_node.parent = curr_node.pos
                    neighbor_node.g_cost = curr_node.g_cost+1
                    neighbor_node.total_cost = curr_node.g_cost+1+manhattan(curr_node.pos, maze.waypoints[0])

                    heapq.heappush(frontier, neighbor_node)
                    visited[neighbor_node.pos] = neighbor_node

    path = [curr_node.pos]
    parent = curr_node.parent
    while parent:
        path.insert(0, visited[parent].pos)
        parent = visited[parent].parent

    return path

def astar_multiple(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    curr_node = Node()
    curr_node.pos = maze.start
    curr_node.waypoints = maze.waypoints

    mst = MST(maze.waypoints)
    MST_weights = {maze.waypoints: mst.compute_mst_weight()}

    frontier = []
    heapq.heappush(frontier, curr_node)
    visited = {(curr_node.pos, curr_node.waypoints) : curr_node.total_cost} #(pos,wp): node    
    end_node = Node()

    while True:
        curr_node = heapq.heappop(frontier)
        
        if len(curr_node.waypoints) == 0:
            end_node = curr_node
            break

        for neighbor_pos in maze.neighbors(curr_node.pos[0], curr_node.pos[1]):

            neighbor_node = Node() 
            neighbor_node.pos = neighbor_pos
            neighbor_node.parent = curr_node
            neighbor_node.g_cost = curr_node.g_cost+1
            neighbor_node.waypoints = tuple(wp for wp in curr_node.waypoints if wp != neighbor_node.pos)

            mindist = min( manhattan(neighbor_pos, waypoint) for waypoint in curr_node.waypoints )
            if MST_weights.get(neighbor_node.waypoints) == None:
                mst = MST(neighbor_node.waypoints) #stolen from shreya
                MST_weights[neighbor_node.waypoints] = mst.compute_mst_weight()
            h = mindist + MST_weights.get(neighbor_node.waypoints)

            neighbor_node.total_cost = curr_node.g_cost+1+h

            if ((neighbor_node.pos,neighbor_node.waypoints) not in visited) or (neighbor_node.total_cost < visited[(neighbor_node.pos,neighbor_node.waypoints)]): #if neighbor is new || if re-visit has better cost
                heapq.heappush(frontier, neighbor_node)
                visited[(neighbor_node.pos,neighbor_node.waypoints)] = neighbor_node.total_cost

    path = [end_node.pos] #backtracing, curr_node is the last node visited (final waypoint)
    parent = end_node.parent
    while parent != None: 
        path.insert(0, parent.pos) #insert parent's pos into path
        parent = parent.parent #move to next node in linked list

    return path

def fast(maze):
    """
    Runs suboptimal search algorithm for extra credit/part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    curr_node = Node()
    curr_node.pos = maze.start
    curr_node.waypoints = maze.waypoints

    mst = MST(maze.waypoints)
    MST_weights = {maze.waypoints: mst.compute_mst_weight()}

    frontier = []
    heapq.heappush(frontier, curr_node)
    visited = {(curr_node.pos, curr_node.waypoints) : curr_node.total_cost} #(pos,wp): node    
    end_node = Node()

    while True:
        curr_node = heapq.heappop(frontier)
        
        if len(curr_node.waypoints) == 0:
            end_node = curr_node
            break

        for neighbor_pos in maze.neighbors(curr_node.pos[0], curr_node.pos[1]):

            neighbor_node = Node() 
            neighbor_node.pos = neighbor_pos
            neighbor_node.parent = curr_node
            neighbor_node.g_cost = curr_node.g_cost+1
            neighbor_node.waypoints = tuple(wp for wp in curr_node.waypoints if wp != neighbor_node.pos)

            mindist = min( manhattan(neighbor_pos, waypoint) for waypoint in curr_node.waypoints )
            if MST_weights.get(neighbor_node.waypoints) == None:
                mst = MST(neighbor_node.waypoints) #stolen from shreya
                MST_weights[neighbor_node.waypoints] = mst.compute_mst_weight()
            h = 2.8*(mindist + MST_weights.get(neighbor_node.waypoints))

            neighbor_node.total_cost = curr_node.g_cost+1+h

            if ((neighbor_node.pos,neighbor_node.waypoints) not in visited) or (neighbor_node.total_cost < visited[(neighbor_node.pos,neighbor_node.waypoints)]): #if neighbor is new || if re-visit has better cost
                heapq.heappush(frontier, neighbor_node)
                visited[(neighbor_node.pos,neighbor_node.waypoints)] = neighbor_node.total_cost

    path = [end_node.pos] #backtracing, curr_node is the last node visited (final waypoint)
    parent = end_node.parent
    while parent != None: 
        path.insert(0, parent.pos) #insert parent's pos into path
        parent = parent.parent #move to next node in linked list

    return path


