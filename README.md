# Advanced-Shortest-Paths

This repository contains efficient implementations of advanced shortest-path algorithms for high-performance graph traversal.

## Algorithms Implemented

* **Bidirectional Dijkstra:**
    * Bidirectional Dijkstra's algorithm is an optimization of the classic Dijkstra's algorithm. Instead of searching only from the source node to the destination, it simultaneously searches from the destination node back to the source.
    * This approach often significantly reduces the search space, especially in large graphs, as the search fronts meet in the middle.
    * It's particularly effective when the shortest path is long, as it avoids exploring large portions of the graph that are far from both the source and destination.
    * This implementation focuses on minimizing the number of visited nodes and improving the overall query speed.

* **Bidirectional A\*:**
    * Bidirectional A\* combines the heuristic guidance of the A\* algorithm with the bidirectional search strategy.
    * A\* uses a heuristic function to estimate the distance from a node to the destination, directing the search towards promising paths.
    * By searching from both the source and destination while using the heuristic, Bidirectional A\* further refines the search, leading to potentially faster and more accurate pathfinding.
    * This implementation pays close attention to heuristic consistency, to guarantee optimal results.

* **Contraction Hierarchies (CH):**
    * Contraction Hierarchies is a preprocessing technique designed for static graphs, such as road networks, where the graph structure doesn't change frequently.
    * It involves a preprocessing phase that adds "shortcuts" to the graph, effectively creating a hierarchical representation of the network.
    * During query time, the algorithm leverages these shortcuts to rapidly find the shortest path, often achieving orders of magnitude faster query times compared to traditional Dijkstra's or A\*.
    * This implementation focuses on a robust preprocessing phase, and on the optimal query phase, to maximize the speed.
