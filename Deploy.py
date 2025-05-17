from flask import Flask, render_template, request, jsonify, url_for
import osmnx as ox
import networkx as nx
import os
import shutil
from shortest_path import *

app = Flask(__name__)

O_cho_dua_map = ox.graph_from_xml('map.osm')
G = Create_simple_Graph(O_cho_dua_map)

@app.route('/')
def index():
    global O_cho_dua_map
    global G
    node_coords = [(O_cho_dua_map.nodes[node]['y'], O_cho_dua_map.nodes[node]['x']) for node in O_cho_dua_map.nodes]
    path_coords = [
        [(O_cho_dua_map.nodes[e[0]]['y'], O_cho_dua_map.nodes[e[0]]['x']), (O_cho_dua_map.nodes[e[1]]['y'], O_cho_dua_map.nodes[e[1]]['x'])]
        for e in O_cho_dua_map.edges
    ]
    return render_template('index.html', node_coords=node_coords, path_coords=path_coords)

# Danh sách thuật toán
algorithm_list = {
    'Dijkstra': Dijkstra,
    'A Star': A_star,
    'UCS': UCS,
    'Greedy BFS': Greedy_best_first_search,
}

# Danh sách hành động với đường đi
path_setup_list = {
    'DeletePath': Delete_Path_1,
    'DeletePathBoth': Delete_Path_2,
    'SetTrafficJam' : Traffic_Jam
}

# Route xử lý thay đổi đường đi (xóa/thêm đường)
@app.route("/setup_path", methods=["POST"])
def setup_path():
    global O_cho_dua_map
    global G
    data = request.get_json()
    start_coords = (data["start"])
    end_coords = (data["end"])
    action = data.get("action")
    start = ox.distance.nearest_nodes(O_cho_dua_map, start_coords[1], start_coords[0])
    end = ox.distance.nearest_nodes(O_cho_dua_map, end_coords[1], end_coords[0])
    
    print("Start node:", start, "End node:", end)
    # Get the distance if edge exists
    try:
        distance = O_cho_dua_map[start][end][0]['length']
        print("Distance between nodes:", distance, "meters")
    except KeyError:
        print("No direct edge between these nodes")
    
    if action == "DeletePath":
        try:
            # Xóa cạnh
            O_cho_dua_map = Delete_Path_1(O_cho_dua_map, int(start), int(end))  # hoặc Delete_Path_1 nếu là oneway
            G = Create_simple_Graph(O_cho_dua_map)
            # Trả về lại node + path hiện tại để client cập nhật lại
            nodes = [[O_cho_dua_map.nodes[n]["y"], O_cho_dua_map.nodes[n]["x"]] for n in O_cho_dua_map.nodes()]
            paths = []
            for u, v, data in O_cho_dua_map.edges(data=True):
                if "geometry" in data:
                    coords = [[point[1], point[0]] for point in data["geometry"].coords]
                else:
                    coords = [
                        [O_cho_dua_map.nodes[u]["y"], O_cho_dua_map.nodes[u]["x"]],
                        [O_cho_dua_map.nodes[v]["y"], O_cho_dua_map.nodes[v]["x"]]
                    ]
                paths.append(coords)
            return jsonify({
                "nodes": nodes,
                "paths": paths

            })
        except Exception as e:
            return jsonify({"error": str(e)}), 500
    elif action == "DeletePathBoth":
        try:
            # Xóa cạnh
            O_cho_dua_map = Delete_Path_2(O_cho_dua_map, int(start), int(end))  # hoặc Delete_Path_1 nếu là oneway
            G = Create_simple_Graph(O_cho_dua_map)
            # Trả về lại node + path hiện tại để client cập nhật lại
            nodes = [[O_cho_dua_map.nodes[n]["y"], O_cho_dua_map.nodes[n]["x"]] for n in O_cho_dua_map.nodes()]
            paths = []
            for u, v, data in O_cho_dua_map.edges(data=True):
                if "geometry" in data:
                    coords = [[point[1], point[0]] for point in data["geometry"].coords]
                else:
                    coords = [
                        [O_cho_dua_map.nodes[u]["y"], O_cho_dua_map.nodes[u]["x"]],
                        [O_cho_dua_map.nodes[v]["y"], O_cho_dua_map.nodes[v]["x"]]
                    ]
                paths.append(coords)

            return jsonify({
                "nodes": nodes,
                "paths": paths
            })
        except Exception as e:
            return jsonify({"error": str(e)}), 500
    else :
        try:
            O_cho_dua_map = Traffic_Jam(O_cho_dua_map, int(start), int(end))  # hoặc Delete_Path_1 nếu là oneway
            G = Create_simple_Graph(O_cho_dua_map)            
            nodes = [[O_cho_dua_map.nodes[n]["y"], O_cho_dua_map.nodes[n]["x"]] for n in O_cho_dua_map.nodes()]

            paths = []
            for u, v, data in O_cho_dua_map.edges(data=True):
                if "geometry" in data:
                    coords = [[point[1], point[0]] for point in data["geometry"].coords]
                else:
                    coords = [
                        [O_cho_dua_map.nodes[u]["y"], O_cho_dua_map.nodes[u]["x"]],
                        [O_cho_dua_map.nodes[v]["y"], O_cho_dua_map.nodes[v]["x"]]
                    ]
                paths.append(coords)
            distance = O_cho_dua_map[start][end][0]['length']
            print("New distance between nodes:", distance, "meters")
            return jsonify({
                "nodes": nodes,
                "paths": paths
            })
        except Exception as e:
            return jsonify({"error": str(e)}), 500
# Route tìm đường đi ngắn nhất
@app.route('/find_shortest_path', methods=['POST'])
def find_shortest_path():
    global O_cho_dua_map
    global G
    data = request.json
    start_coords = data['start']
    end_coords = data['end']
    algorithm = data['algorithm']
    max_depth = int(data.get('max_depth', 0))

    start_node = ox.distance.nearest_nodes(O_cho_dua_map, start_coords[1], start_coords[0])
    end_node = ox.distance.nearest_nodes(O_cho_dua_map, end_coords[1], end_coords[0])

    func = algorithm_list.get(algorithm)
    if not func:
        return jsonify({"error": "Invalid algorithm selected"}), 400

    path_coords = func(G, start_node, end_node)

    start_coords_path = [(O_cho_dua_map.nodes[start_node]['y'], O_cho_dua_map.nodes[start_node]['x']), (start_coords[0], start_coords[1])]
    end_coords_path = [(O_cho_dua_map.nodes[end_node]['y'], O_cho_dua_map.nodes[end_node]['x']), (end_coords[0], end_coords[1])]

    if path_coords is None:
        return jsonify({"error": "No path found"}), 404

    return jsonify({'path_coords': path_coords, 'max_depth': max_depth, 'start_path': start_coords_path, 'end_path': end_coords_path})

if __name__ == '__main__':
    app.run(debug=True, port=8000)
