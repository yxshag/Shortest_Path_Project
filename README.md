# Shortest_Path_Project
A highly optimized implementation of Dijkstra's Shortest Path Algorithm in python capable of parsing real-world geospatial data (.graphml maps) to find the most efficient routes.

# Shortest Path Algorithm for Real-World Maps (Python)

This project implements Dijkstra's Shortest Path Algorithm, A* Algorithm and Double A* Algorithm in Python to calculate the absolute shortest routes over large-scale road networks. 

## 🗺️ Project Focus
The core engine is built to process real-world spatial networks (originally tested using an OpenStreetMap graph layout of Bengaluru, India). 

## 🚀 Key Specifications
- **Data Structure:** Uses Python's built-in `heapq` module (binary heap) to drastically optimize the priority queue operations.
- **Time Complexity:** $O((V + E) \log V)$, making it efficient enough to calculate routes over thousands of intersections in seconds.
- **Input Format:** Parses `.graphml` graph files into nodes (intersections) and weighted edges (road distances/travel times).

## 💻 Tech Stack
- **Language:** Python 3
- **Key Modules:** `heapq` (Min-Heap), `os`, `osmnx`(open street maps), `flask`
- **Concepts:** Dijkstra's Algorithm, A* Algorithm, Double A* Algorithm, Graph Theory, Network Routing

---

## 🛠️ Setup & How to Run

To run this project locally, follow these exact steps:

### 1. Download the Required Files
Make sure you download **both** of the following files and place them inside the **same folder** on your computer:
* `app.py` (The main Python backend application)
* `index.html` (The frontend user interface)

### 2. Run the Application
Open your terminal or command prompt, navigate to the folder containing these files, and run the Python script:

```bash
python app.py
