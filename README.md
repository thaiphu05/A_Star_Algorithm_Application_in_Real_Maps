# A* Algorithm Application in Real Maps

⭐ **A* Algorithm Application in Real Maps Using OSMnx and NetworkX**  
📌 *Course Project - Introduction to AI (SOICT, HUST)*

---

## 📌 Overview
This project implements the **A* Search Algorithm** on real-world maps using data from **OpenStreetMap (OSM)**. The graph is built with **OSMnx** and **NetworkX**, and the algorithm is applied to find the shortest path between two points in a real-world road network.

---

## 📂 Features
✅ **Graph Construction from OSM Data**: Extract real-world road networks.  
✅ **A* Algorithm Implementation**: Custom implementation without built-in shortest path functions.  
✅ **Visualization**: Display graph and computed shortest paths.  
✅ **Customizable Heuristic**: Supports different heuristics for pathfinding.  

---

## 📊 Data Attributes & Descriptions
| Attribute  | Description |
|------------|-------------|
| **osmid**  | Road ID in OpenStreetMap |
| **highway** | Road type (residential, primary, etc.) |
| **oneway**  | Indicates if the road is one-way |
| **reversed** | Road direction when loaded from OSM |
| **length**  | Road length (in meters) |
| **geometry** | GPS data of the road (LINESTRING) |
| **lanes**  | Number of lanes on the road |
| **name**  | Road name |

---

## 📌 Technologies Used
- **Python** 🐍
- **OSMnx** 🌍 (For extracting real-world road networks)
- **NetworkX** 🔗 (For graph representation and pathfinding)
- **Matplotlib** 📊 (For visualization)

---

## 🚀 How to Run
1. Install dependencies:
   ```bash
   pip install osmnx networkx matplotlib numpy
   ```
2. Run the main file :
   Run file A_star.ipynb ( All )
   
---

## 📌 Authors
🔹 **[Trinh Phu Thai]** - SOICT, HUST  
📌 *Course: Introduction to AI*  

🎯 **Project for educational purposes only.**

---

