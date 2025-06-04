import numpy as np 
import matplotlib.pyplot as plt
import heapq
import cv2
import osmnx as ox
import networkx as nx
from math import radians, cos, sqrt
import joblib
import torch
import torch.nn as nn

# Load model từ file đã lưu
class HeuristicNet(nn.Module):
    def __init__(self):
        super(HeuristicNet, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(5, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
            nn.Linear(32, 2)
        )

    def forward(self, x):
        return self.net(x)

model = HeuristicNet()
model.load_state_dict(torch.load('ml_heuristic_model.pt'))

maps = ox.graph_from_xml('map_new.osm')
def euclidean(lat1, lon1, lat2, lon2):
    lat_mean = radians((lat1 + lat2) / 2)
    dx = (lon2 - lon1) * 111320 * cos(lat_mean)
    dy = (lat2 - lat1) * 111320
    return sqrt(dx**2 + dy**2)
def predict_distance(node1, node2, model):
    try:
        # Lấy tọa độ
        lat1 = maps.nodes[node1]['y']
        lon1 = maps.nodes[node1]['x']
        lat2 = maps.nodes[node2]['y']
        lon2 = maps.nodes[node2]['x']
        
        # Tính Euclidean distance
        euc_dist = euclidean(lat1, lon1, lat2, lon2)
        
        # Tạo tensor input đúng kích thước
        features = torch.tensor([lat1, lon1, lat2, lon2, euc_dist], 
                              dtype=torch.float32).reshape(1, -1)
        
        # Dự đoán
        model.eval()
        with torch.no_grad():
            output = model(features)
            
        return output[0][0].item()  # Lấy giá trị khoảng cách
        
    except Exception as e:
        print(f"Lỗi khi dự đoán: {str(e)}")
        return None
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

def Traffic_Jam(graph, start, goal):
    try:
        for _, v, k in list(graph.edges(start, keys=True)):
            if v == goal:
                graph[start][v][k]['length'] *= 3
        for _, v, k in list(graph.edges(goal, keys=True)):
            if v == start:
                graph[goal][v][k]['length'] *= 3
    except Exception as e:
        print("Error modifying edge weight:", e)
    return graph

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
                f_score[neighbor] = g_score[neighbor] + min(h1(neighbor, goal)*1.2 , predict_distance(neighbor, goal, model))
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
def ML_heuristic(u, end, G, model):
    u_x, u_y = G.nodes[u]['x'], G.nodes[u]['y']
    end_x, end_y = G.nodes[end]['x'], G.nodes[end]['y']
    
    try:
        path = nx.shortest_path(G, u, end)
        n_between = max(len(path) - 2, 0)
    except:
        n_between = 0

    input_tensor = torch.tensor([[u_x, u_y, n_between]], dtype=torch.float32)
    with torch.no_grad():
        h = model(input_tensor).item()
    return h
