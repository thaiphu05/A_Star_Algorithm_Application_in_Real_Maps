import numpy as np 
import matplotlib.pyplot as plt
import heapq
import cv2
import osmnx as ox
import networkx as nx
from math import radians, cos, sqrt
import joblib

# Load model từ file đã lưu
model = joblib.load('ml_heuristic_model.pkl')


maps = ox.graph_from_xml('map.osm')

def Create_path_coord(path, maps):
    path_coords = [
        [(maps.nodes[e[0]]['y'], maps.nodes[e[0]]['x']), (maps.nodes[e[1]]['y'], maps.nodes[e[1]]['x'])]
        for e in path_to_edges(path)
    ]
    return path_coords
def Delete_Path_1(graph, start, goal):
    try:
        edges_to_remove = list(graph.edges(start, keys=True))
        for u, v, k in edges_to_remove:
            if v == goal:
                graph.remove_edge(u, v, k)
    except Exception as e:
        print("Error removing edge:", e)
    return graph


def Delete_Path_2(graph, start, goal):
    try:
        for u, v, k in list(graph.edges(start, keys=True)):
            if v == goal:
                graph.remove_edge(u, v, k)
        for u, v, k in list(graph.edges(goal, keys=True)):
            if v == start:
                graph.remove_edge(u, v, k)
    except Exception as e:
        print("Error removing edge:", e)
    return graph

def Traffic_Jam (graph, start, goal):
    try:
        edges_to_remove = list(graph.edges(start, keys=True))
        for u, v, k in edges_to_remove:
            if v == goal:
                k= k*3
    except Exception as e:
        print("Error removing edge:", e)
    return graph

# def Add_Path(graph, start, goal, length):
#     if start not in graph:
#         graph[start] = []
#     if goal not in graph:
#         graph[goal] = []
#     graph[start].append([goal, length])
#     graph[goal].append([start, length])
#     return graph


def path_to_edges(path):
    return [(path[i], path[i + 1]) for i in range(len(path) - 1)]

def Create_simple_Graph(maps):
    Edges = list(maps.edges(data=True, keys=True))
    Graph = {node: [] for node in maps.nodes}
    for u, v, k, data in Edges:
        Graph[u].append([v, data['length']])
    return Graph

def h1(current, goal): 
    lat1 = maps.nodes[current]['y']
    lon1 = maps.nodes[current]['x']
    lat2 = maps.nodes[goal]['y']
    lon2 = maps.nodes[goal]['x']
    
    lat_mean = radians((lat1 + lat2) / 2)
    
    dx = (lon2 - lon1) * 111320 * cos(lat_mean)
    dy = (lat2 - lat1) * 111320
    
    return sqrt(dx**2 + dy**2)
def h_ml(current, goal):
    euc = h1(current, goal)
    return model.predict([[euc]])[0]

def heuristic_bfs(graph, start, goal):
    if start == goal:
        return 0
        
    parent_nodes = {start: None}
    frontier = [start]
    explored = []

    while frontier:
        current = frontier.pop(0)
        if current == goal:
            explored.append(current)
            break
        explored.append(current)
        for neighbor in graph[current]:
            if neighbor[0] not in explored and neighbor[0] not in frontier:
                frontier.append(neighbor[0])
                parent_nodes[neighbor[0]] = current
                
    if goal not in parent_nodes:
        return float('inf')
        
    return calculate_distance_bfs(parent_nodes, graph, start, goal)

def calculate_distance_bfs(parent_nodes, graph, start, goal):
    try:
        distance = 0
        current = goal
        while current != start:
            if current not in parent_nodes or parent_nodes[current] not in graph:
                return float('inf')
            parent = parent_nodes[current]
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
    return Create_path_coord(total_path, maps)

def A_star(graph, start, goal):
    if start not in graph or goal not in graph:
        return None

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0
    f_score = {node: float('inf') for node in graph}
    f_score[start] = h1(start, goal)

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor, cost in graph[current]:
            if neighbor not in graph:
                continue
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + min(h1(neighbor, goal)*1.2 , h_ml(neighbor, goal))
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
        
        for neighbor, _ in graph[current]:
            if neighbor in visited:
                continue
            if neighbor not in came_from:
                came_from[neighbor] = current
                heapq.heappush(open_set, (h1(neighbor, goal), neighbor))
    
    return None

def UCS(graph, start, goal):
    if start not in graph or goal not in graph:
        return None
    open_set = [(0, start)]
    came_from = {}
    visited = set()
    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0

    while open_set:
        cost_so_far, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)
        
        visited.add(current)

        for neighbor, cost in graph[current]:
            if neighbor in visited:
                continue
            tentative_g = g_score[current] + cost
            if tentative_g < g_score[neighbor]:
                g_score[neighbor] = tentative_g
                came_from[neighbor] = current
                heapq.heappush(open_set, (tentative_g, neighbor))
    return None

def Dijkstra(graph, start, goal):
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

        for neighbor, cost in graph[current]:
            tentative_g_score = g_score[current] + cost
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                heapq.heappush(open_set, (g_score[neighbor], neighbor))

    return None
