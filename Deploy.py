from flask import Flask, render_template, request, jsonify, url_for
import osmnx as ox
import networkx as nx
import os
import shutil

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get_graph', methods=['POST'])
def get_graph():
    data = request.json
    print(data)
    location = data['location']
    print(location)
    G = ox.graph_from_place(location, network_type='drive')
    ox.plot_graph(G)
    return jsonify({'status': 'success'})


if __name__ == '__main__':
    app.run(debug=True,port = 8000)