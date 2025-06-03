import osmnx as ox
import networkx as nx
import numpy as np
import pandas as pd
import xgboost as xgb
from math import radians, cos, sqrt
import joblib
from typing import Tuple, List

def euclidean(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate Euclidean distance between two points."""
    lat_mean = radians((lat1 + lat2) / 2)
    dx = (lon2 - lon1) * 111320 * cos(lat_mean)
    dy = (lat2 - lat1) * 111320
    return sqrt(dx**2 + dy**2)

def prepare_data(df: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
    """Prepare features and labels for model training."""
    # Extract features and labels
    X = df[['node1_x', 'node1_y', 'node2_x', 'node2_y']].values
    y = df[['shortest_distance', 'nodes_between']].values
    
    # Add Euclidean distance as feature
    euclidean_distances = np.array([
        euclidean(row[0], row[1], row[2], row[3]) for row in X
    ])
    X = np.hstack([X, euclidean_distances.reshape(-1, 1)])
    
    return X, y

def train_model(X: np.ndarray, y: np.ndarray) -> xgb.Booster:
    """Train XGBoost model with given parameters."""
    dtrain = xgb.DMatrix(X, label=y)
    
    params = {
        'objective': 'reg:squarederror',
        'eval_metric': 'rmse',
        'max_depth': 6,
        'learning_rate': 0.1,
        'n_estimators': 100,
        'early_stopping_rounds': 10
    }
    
    return xgb.train(params, dtrain)

def predict_distance(node1: int, node2: int, G: nx.MultiDiGraph, model: xgb.Booster) -> float:
    """Predict distance between two nodes using trained model."""
    try:
        lat1 = G.nodes[node1]['y']
        lon1 = G.nodes[node1]['x']
        lat2 = G.nodes[node2]['y']
        lon2 = G.nodes[node2]['x']
        
        euc_dist = euclidean(lat1, lon1, lat2, lon2)
        features = np.array([[lat1, lon1, lat2, lon2, euc_dist]])
        dtest = xgb.DMatrix(features)
        
        return model.predict(dtest)[0]
    except KeyError as e:
        raise KeyError(f"Node not found in graph: {e}")

def compare_distances(G: nx.MultiDiGraph, model: xgb.Booster, num_nodes: int = 20) -> pd.DataFrame:
    """Compare different distance calculations between node pairs."""
    nodes = list(G.nodes)[:num_nodes]
    results = []
    
    for i, node1 in enumerate(nodes):
        for j, node2 in enumerate(nodes):
            if i == j:
                continue
            
            try:
                lat1, lon1 = G.nodes[node1]['y'], G.nodes[node1]['x']
                lat2, lon2 = G.nodes[node2]['y'], G.nodes[node2]['x']
                
                euc_dist = euclidean(lat1, lon1, lat2, lon2)
                pred_dist = predict_distance(node1, node2, G, model)
                actual_dist = nx.shortest_path_length(G, node1, node2, weight='length')
                
                results.append({
                    'node1': node1,
                    'node2': node2,
                    'euclidean': euc_dist,
                    'predicted': pred_dist,
                    'actual': actual_dist,
                    'pred_error': abs(pred_dist - actual_dist)/actual_dist * 100
                })
                
            except Exception as e:
                print(f"Error processing nodes {node1},{node2}: {str(e)}")
    
    return pd.DataFrame(results)

if __name__ == "__main__":
    # Load data
    df = pd.read_csv('shortest_path_dataset.csv')
    X, y = prepare_data(df)
    
    # Train model
    model = train_model(X, y)
    
    # Load graph
    G = ox.graph_from_xml('map_new.osm', simplify=False)
    
    # Compare distances
    results_df = compare_distances(G, model)
    print("\nSummary Statistics:")
    print(results_df[['euclidean', 'predicted', 'actual', 'pred_error']].describe())
    print ("\nSample Results:")
    print(results_df.head())
    # Save model
    joblib.dump(model, 'ml_heuristic_model.pkl')