import numpy as np 
import matplotlib.pyplot as plt
import heapq
import cv2
import osmnx as ox
import networkx as nx
from math import radians, cos, sqrt

maps = ox.load_graphml('ochodua_dongda_hanoi_graph.graphml')

def Create_path_coord (path, maps):
    path_coords = [
        [(maps.nodes[e[0]]['y'], maps.nodes[e[0]]['x']), (maps.nodes[e[1]]['y'], maps.nodes[e[1]]['x'])]
        for e in path_to_edges(path)
    ]
    return path_coords

def Create_simple_Graph(maps):
    Edges =list (maps.edges(data=True, keys=True))
    Graph ={}
    for node in maps.nodes:
        Graph[node] = []
    for node in maps.nodes:
        for edges in Edges:
            if edges[0]== node:
                    Graph[node].append([edges[1],edges[3]['length']]) 

    return Graph

def h1(current, goal): 
# euclidean distance between two points in multidigraph 
    lat1 = maps.nodes[current]['y']
    lon1 = maps.nodes[current]['x']
    lat2 = maps.nodes[goal]['y']
    lon2 = maps.nodes[goal]['x']
    
    lat_mean = radians((lat1 + lat2) / 2)
    
    dx = (lon2 - lon1) * 111320 * cos(lat_mean)
    dy = (lat2 - lat1) * 111320
    
    return sqrt(dx**2 + dy**2)

def heuristic_bfs(graph, start, goal ):
# Heuristic function for A* algorithm using mean_BFS 
    if start == goal:
        return 0
        
    parent_nodes = {}
    parent_nodes[start] = None
    frontier = []
    frontier.append(start)
    explored = []
    
    while frontier:
        current = frontier.pop(0)
        if current == goal:
            explored.append(current)
            break
        explored.append(current)
        for node in graph[current]:
            if node[0] not in explored and node[0] not in frontier:
                frontier.append(node[0])
                parent_nodes[node[0]] = current
                
    if goal not in parent_nodes:
        return float('inf')
        
    return calculate_distance_bfs(parent_nodes, start, goal)

def calculate_distance_bfs(parent_nodes, graph, start, goal):
    try:
        distance = 0
        current = goal
        while current != start:
            if current not in parent_nodes or parent_nodes[current] not in graph:
                return float('inf')
            parent = parent_nodes[current]
            # Tìm khoảng cách giữa current và parent
            for node, dist in graph[parent]:
                if node == current:
                    distance += dist
                    break
            current = parent
        return distance
    except KeyError:
        return float('inf')

def reconstruct_path(came_from, current):
    total_path = [current]
    while current in came_from:
        current = came_from[current]
        total_path.append(current)
    total_path.reverse()
    return Create_path_coord(total_path,maps)

def path_to_edges(path):
    return [(path[i], path[i + 1]) for i in range(len(path) - 1)]

def A_star(graph, start, goal):
    if start not in graph or goal not in graph:
        return None
    max_cutoff = h1(start, goal,)*2
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = h1(start, goal)
    # A star with heuristic calculate by euclid distance / mean_bfs (not recommended) 
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor_data in graph[current]:
            neighbor = neighbor_data[0]
            if neighbor not in graph:  
                continue
            cost = neighbor_data[1]
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + h1(neighbor, goal)
                if f_score[neighbor] > max_cutoff:
                    continue
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

def Greedy_best_first_search(graph, start, goal):
    if start not in graph or goal not in graph:
        return None
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    visited = set()
    
    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        
        visited.add(current)
        
        for neighbor_data in graph[current]:
            neighbor = neighbor_data[0]
            if neighbor in visited:
                continue
            if neighbor not in came_from:
                came_from[neighbor] = current
                heapq.heappush(open_set, (h1(neighbor, goal), neighbor))
    
    return None
def UCS (graph, start , goal ):
    if start not in graph or goal not in graph:
            return None
    open_set = [(0, start)]
    came_from = {}
    visited = set()
    g_score = {node : float ('inf') for node in graph}
    g_score[start] = 0
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            return reconstruct_path(came_from, current)
        visited.add(current)

        for neighbor in graph[current] :
            if neighbor[0] in visited : continue
            for node, dist in graph[current]:
                if node == neighbor[0]:
                    break
            if g_score[current] + dist < g_score[neighbor[0]]:
                g_score[neighbor[0]] = neighbor[1] + g_score [current]
                came_from[neighbor[0]] = current
                heapq.heappush (open_set, (g_score[neighbor[0]], neighbor))
    return None 

def Dijkstra(graph, start, goal):
    """ Thuật toán Dijkstra tìm đường đi ngắn nhất từ start đến goal """
    if start not in graph or goal not in graph:
        return None
    
    open_set = []
    heapq.heappush(open_set, (0, start))  

    came_from = {}  
    g_score = {node: float('inf') for node in graph}  
    g_score[start] = 0  

    while open_set:
        current_cost, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor_data in graph[current]:  
            neighbor, cost = neighbor_data  
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score[neighbor]:  
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                heapq.heappush(open_set, (g_score[neighbor], neighbor))

    return None  