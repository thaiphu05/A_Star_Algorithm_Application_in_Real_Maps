from flask import Flask, render_template, request, jsonify, url_for
import osmnx as ox
import networkx as nx
import os
import shutil
from shortest_path import *



app = Flask(__name__)

O_cho_dua_map = ox.load_graphml('ochodua_dongda_hanoi_graph.graphml')

G= Create_simple_Graph(O_cho_dua_map)


@app.route('/')
def index():
    node_coords = [(O_cho_dua_map.nodes[node]['y'], O_cho_dua_map.nodes[node]['x']) for node in O_cho_dua_map.nodes]
    path_coords = [
        [(O_cho_dua_map.nodes[e[0]]['y'], O_cho_dua_map.nodes[e[0]]['x']), (O_cho_dua_map.nodes[e[1]]['y'], O_cho_dua_map.nodes[e[1]]['x'])]
        for e in O_cho_dua_map.edges
    ]
    return render_template('index.html', node_coords=node_coords, path_coords=path_coords)

algorithm_list = {
    'Dijkstra': Dijkstra, 
    'A Star': A_star, 
    'UCS': UCS,
    'Greedy BFS': Greedy_best_first_search,
}

@app.route('/find_shortest_path', methods=['POST'])
def find_shortest_path():
    data = request.json
    start_coords = data['start']
    end_coords = data['end']
    algorithm = data['algorithm']
    max_depth = int(data['max_depth']) # if not specified, max_depth is 0
    # Find the nearest nodes on the graph to the clicked points
    start_node = ox.distance.nearest_nodes(O_cho_dua_map, start_coords[1], start_coords[0])  # lon, lat
    end_node = ox.distance.nearest_nodes(O_cho_dua_map, end_coords[1], end_coords[0])
    
    func = algorithm_list.get(algorithm)
    if not func:
        return jsonify({"error": "Invalid algorithm selected"}), 400

    # Calculate the path using the selected algorithm
    path_coords = []
    # if func != DLS and func != IDS:
    #     path_coords = func(G, start_node, end_node)
    # elif func == DLS:
    #     path_coords = func(G, start_node, end_node, max_depth)
    # elif func == IDS:
    #     path_coords, max_depth = func(G, start_node, end_node)
    path_coords = func(G, start_node, end_node)
    start_coords_path =[(O_cho_dua_map.nodes[start_node]['y'], O_cho_dua_map.nodes[start_node]['x']),(start_coords[0], start_coords[1])]
    end_coords_path =[(O_cho_dua_map.nodes[end_node]['y'], O_cho_dua_map.nodes[end_node]['x']),(end_coords[0], end_coords[1])]
    if path_coords is None:
        return jsonify({"error": "No path found"}), 404
    return jsonify({'path_coords': path_coords, 'max_depth': max_depth, 'start_path': start_coords_path , 'end_path': end_coords_path })

if __name__ == '__main__':
    app.run(debug=True,port = 8000)