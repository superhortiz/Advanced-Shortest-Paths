#include <cstdio>
#include <cstring>
#include <vector>
#include <algorithm>
#include <limits>
#include <queue>
#include <iostream>
#include <memory>
#include <cassert>
#include <unordered_set>

class Graph {
    typedef int Distance;
    typedef int Vertex;
    static constexpr int INF = std::numeric_limits<int>::max() / 2;

    // Number of nodes
    int N;
    // Source and target
    int s, t;
    // Estimate of the distance from s to t
    int estimate = INF;
    // Lists of edges outgoing from each node
    std::vector<std::vector<std::pair<int, int>>> outgoing_edges;
    // Lists of edges incoming to each node
    std::vector<std::vector<std::pair<int, int>>> incoming_edges;

    // Levels of nodes (heuristic)
    std::vector<int> level;
    // Ranks of nodes - hierarchy after the contraction process
    std::vector<int> rank;

    // Distance to node v, bidistance[0][v] - from source in the forward search,
    // bidistance[1][v] - from target in the backward search.
    std::vector<std::vector<Distance>> bidistance;

    // Added to perform Djikstra for witness seach
    std::vector<Distance> distance;

    // Wrapper around STL priority_queue
    class StlHeap {
    public:
        using T = std::pair<Distance, Vertex>;
        using Queue = std::priority_queue<T, std::vector<T>, std::greater<T>>;

        StlHeap() {
            queue.reset(new Queue());
        }

        bool empty() const {
            return queue->empty();
        }

        void update(Vertex v, Distance d) {
            queue->push(std::make_pair(d, v));
        }

        void clear() {
            queue.reset(new Queue());
        }

        std::pair<Distance, Vertex> pop() {
            std::pair<Distance, Vertex> top = queue->top();
            queue->pop();
            return top;
        }

    private:
        std::unique_ptr<Queue> queue;
    };

    // Priority queues for forward and backward searches
    StlHeap diqueue[2];

public:
    Graph() {
        read_stdin();
        bidistance.resize(2, std::vector<int>(N, INF));
    }

    int get_n() { return N;}

    std::vector<std::pair<int, int>>& get_adjacent(int v, bool forward = true) {
        if (forward) {
            return outgoing_edges[v];
        } else {
            return incoming_edges[v];
        }
    }

    void preprocess() {
        // Initialize distance, level, and rank vectors
        distance.resize(N, INF);
        level.resize(N, 0);
        rank.resize(N, 0);

        // Priority queue will store pairs of (importance, node) with the least important node in the head
        std::priority_queue<std::pair<int, int>, std::vector<std::pair<int,int>>, std::greater<std::pair<int, int>>> queue;

        // Vector to store calculated shortcuts
        std::vector<Shortcut> shortcuts;
        int importance, node;

        // Initial calculation of node importance and population of the priority queue
        for (node = 0; node < N; ++node) {
            importance = get_heuristic(node, shortcuts);
            queue.emplace(importance, node);
            shortcuts.clear();
        }

        // Initialize the current rank
        int current_rank = 0;

        // Main contraction loop
        while (!queue.empty()) {
            // Get the node with the lowest importance
            std::pair<int,int> top = queue.top();
            node = top.second;
            queue.pop();

            // Re-calculate the importance of the node
            importance = get_heuristic(node, shortcuts);

            // Check if the node should be contracted
            if (queue.empty() || importance <= queue.top().first) {
                // Contract the node and increment the rank
                contract_node(node, shortcuts, ++current_rank);
            } else {
                // Re-add the node to the queue with the updated importance
                queue.emplace(importance, node);
            }

            // Clear the shortcuts vector for the next iteration
            shortcuts.clear();
        }

        // Remove unnecessary edges
        finalize();

        // Resize visited vectors to prepare for query phase
        visited[0].resize(N);
        visited[1].resize(N);
    }

    // Returns distance from s to t in the graph
    int query(int s, int t) {
        // Reset data structures for a new query
        clear();

        // Initialize the starting distances for the forward and backward searches
        bidistance[0][s] = bidistance[1][t] = 0;

        // Start the forward and backward searches from 's' and 't'
        update(s, 0, true);
        update(t, 0, false);

        int v, dist;
        std::pair<Distance, Vertex> top;  // Pair to store distance and vertex from priority queue

        // Continue the searches until both priority queues are empty
        while (!diqueue[0].empty() || !diqueue[1].empty()) {
            // Forward search
            if (!diqueue[0].empty()) {
                top = diqueue[0].pop();  // Get the vertex with the smallest distance from the forward queue
                v = top.second;
                dist = top.first;

                // Only process the vertex if its distance is less than or equal to the current estimate
                if (dist <= estimate) {
                    update(v, dist, true);  // Relax the neighbors of 'v' in the forward direction
                }
            }

            // Backward search
            if (!diqueue[1].empty()) {
                top = diqueue[1].pop();  // Get the vertex with the smallest distance from the backward queue
                v = top.second;
                dist = top.first;

                // Only process the vertex if its distance is less than or equal to the current estimate
                if (dist <= estimate) {
                    update(v, dist, false);  // Relax the neighbors of 'v' in the backward direction
                }
            }
        }

        // If 'estimate' is still INF, it means no path was found.
        if (estimate == INF) return -1;

        // Return the shortest path distance found.
        return estimate;
    }

private:
    // Clear data structures for query
    void clear() {
        // Clear bidistance vector
        for (const auto& v : visited[0].get()) {
            bidistance[0][v] = INF;
        }

        for (const auto& v : visited[1].get()) {
            bidistance[1][v] = INF;
        }

        // Clear visited set
        visited[0].clear();
        visited[1].clear();

        // Clear priority queue
        diqueue[0].clear();
        diqueue[1].clear();

        // Reset estimate variable
        estimate = INF;
    }

    // Try to relax the node v using distance d either in the forward or in the backward search
    void update(int v, int d, bool forward) {
        // Add the current node v to the visited set
        visited[1 - forward].add(v);

        // Iterate through the adjacent edges of node v
        for (const auto& edge : get_adjacent(v, forward)) {
            int w = edge.first;
            int cost = edge.second;

            // Check if it's possible to relax the edge
            if (bidistance[1 - forward][w] > d + cost) {
                // Update the distance to w
                bidistance[1 - forward][w] = d + cost;

                // Update the priority queue
                diqueue[1 - forward].update(w, bidistance[1 - forward][w]);

                // Add w to the visited set
                visited[1 - forward].add(w);
            }
        }
        // Check if node v has been visited in the *opposite* direction
        if (visited[forward].has(v) && estimate > bidistance[0][v] + bidistance[1][v]) {
            // If the combined distance from 's' to 'v' and 't' to 'v' is less than the
            // current estimate, update the estimate.
            estimate = bidistance[0][v] + bidistance[1][v];
        }
    }

    // Manage visited nodes in graph traversal (query) with a list (vertices) and a marker vector (visited)
    class VertexSet {
    public:
        VertexSet(int n = 0) : visited(n) {}
        void resize(int n) {
            visited.resize(n);
        }
        void add(int v) {
            if (!visited[v]) {
                vertices.push_back(v);
                visited[v] = true;
            }
        }
        const std::vector<int>& get() const {
            return vertices;
        }
        const bool has(int v) {
            return visited[v];
        }
        void clear() {
            for (int v : vertices) {
                visited[v] = false;
            }
            vertices.clear();
        }

    private:
        std::vector<int> visited;
        std::vector<int> vertices;
    };

    VertexSet visited[2];

    // QEntry = (distance, vertex)
    typedef std::pair<int, int> QEntry;
    std::priority_queue<QEntry, std::vector<QEntry>, std::greater<QEntry>> queue;

    struct Shortcut {
        int from;
        int to;
        int cost;
    };

    // Calculates the maximum distance to perform Dijkstra for witness path search
    int max_distance_search(int v) {
        // Initialize distances
        Distance max_distance_u_v = 0;  // This corresponds to max{ l(u, v) }
        Distance adjusted_v_w = 0;  // This corresponds to max{ l(v, w) - l(w', w) }

        // Iterate through all incoming edges to vertex v
        for (const auto& incoming_edge : incoming_edges[v]) {
            Vertex u = incoming_edge.first;  // Get the source vertex (u) of the incoming edge

            // Skip contracted neighbors
            if (rank[u] != 0) continue;

            // Update max_distance_u_v with the maximum incoming edge weight
            max_distance_u_v = std::max(max_distance_u_v, incoming_edge.second);
        }

        // Iterate through all outgoing edges from vertex v
        for (const auto& outgoing_edge : outgoing_edges[v]) {
            Vertex w = outgoing_edge.first;  // Get the destination vertex (w) of the outgoing edge

            // Skip contracted neighbors
            if (rank[w] != 0) continue;

            // Get the weight of the outgoing edge (v, w)
            Distance distance_v_w = outgoing_edge.second;

            // This corresponds to min{ l(w', w) }
            Distance min_distance_wp_w = INF;
            bool w_has_wp = false;

            // Iterate through all incoming edges to vertex w to find the minimum distance from w's predecessors
            for (const auto& incoming_edge : incoming_edges[w]) {
                int wp = incoming_edge.first;

                // Skip contracted neighbors
                if (wp == v || rank[wp] != 0) continue;

                w_has_wp = true;
                Distance distance_wp_w = incoming_edge.second;
                min_distance_wp_w = std::min(min_distance_wp_w, distance_wp_w);  // Find the minimum distance
                
            }
            // Update max{ l(v, w) - l(w', w) }
            if (!w_has_wp) min_distance_wp_w = 0;
            adjusted_v_w = std::max(adjusted_v_w, distance_v_w - min_distance_wp_w);
        }

        // Return max { l(u, v) + l(v, w) - l(w', w) }
        return max_distance_u_v + adjusted_v_w;
    }

    // Performs Dijkstra algorithm for withness path search
    void dijkstra(int s, Distance max_distance_dijkstra, int skip_vertex, std::vector<Vertex> &processed_vertices) {
        // Set distance to source to zero
        distance[s] = 0;

        // Add the source vertex to the priority queue with distance 0
        queue.emplace(0, s);

        // Add the source vertex to the list of processed vertices
        processed_vertices.push_back(s);

        // Loop until the priority queue is empty or the distance of the top vertex exceeds the maximum allowed distance
        while (!queue.empty() && queue.top().first <= max_distance_dijkstra) {
            // Get the vertex with the minimum distance and remove it from the priority queue
            int v = queue.top().second;
            Distance dist = queue.top().first;
            queue.pop();

            // Skip processing if this is an outdated entry in the priority queue
            if (distance[v] < dist) continue;

            // Process all the neighbors of the current vertex 
            for (const auto& outgoing_edge : outgoing_edges[v]) {
                int w = outgoing_edge.first;  // Get the neighbor vertex w
                Distance cost = outgoing_edge.second;  // Get the edge weight (cost) from v to w

                // Check if the neighbor w should be processed
                if (w != skip_vertex && rank[w] == 0 && distance[w] > distance[v] + cost) {
                    // Process node w
                    distance[w] = distance[v] + cost;
                    queue.emplace(distance[w], w);
                    processed_vertices.push_back(w);
                }
            }
        }
        // Clear priority queue for the next search
        while (!queue.empty()) {
            queue.pop();
        }
    }

    // Get the heuristics for a node. It also determines the necessary shortcuts for the given node
    int get_heuristic(int v, std::vector<Shortcut>& shortcuts) {
        // Initialize data structures
        std::vector<int> processed_vertices;  // Stores vertices processed by Dijkstra
        std::unordered_set<int> affected_neighbors;  // Stores neighbors affected by shortcuts
        std::unordered_set<int> cont_neighbors;  // Stores contracted neighbors of v

        // Populate cont_neighbors with contracted outgoing neighbors of v
        for (const auto& outgoing_edge : outgoing_edges[v]) {
            Vertex w = outgoing_edge.first;
            if (rank[w] != 0) cont_neighbors.insert(w);  // If w is contracted, add it to cont_neighbors
        }

        // Calculate the maximum distance for Dijkstra's witness search
        Distance max_distance_dijkstra = max_distance_search(v);

        // Iterate through incoming edges of v to perform witness searches
        for (const auto& incoming_edge : incoming_edges[v]) {
            Vertex u = incoming_edge.first;

            // Populate cont_neighbors with contracted incoming neighbors of v
            if (rank[u] != 0) {
                cont_neighbors.insert(u);
                continue;
            }

            Distance cost_u_v = incoming_edge.second;

            // Perform Dijkstra's witness search from u, skipping v
            dijkstra(u, max_distance_dijkstra, v, processed_vertices);

            // Iterate through outgoing edges of v to check for shortcuts
            for (const auto& outgoing_edge : outgoing_edges[v]) {
                Vertex w = outgoing_edge.first;
                if (rank[w] != 0) continue;
                Distance cost_v_w = outgoing_edge.second;

                // Check if a shortcut is needed
                if (distance[w] > cost_u_v + cost_v_w) {
                    // Create and add the shortcut to the shortcuts vector
                    Shortcut sc = {u, w, cost_u_v + cost_v_w};
                    shortcuts.push_back(sc);

                    // Add u and w to the set of affected neighbors
                    affected_neighbors.insert(u);
                    affected_neighbors.insert(w);
                }
            }

            // Reset distance values and clear processed_vertices for the next Dijkstra run
            for (const auto& vertex : processed_vertices) {
                distance[vertex] = INF;
            }
            processed_vertices.clear();
        }

        // Calculate heuristics
        int edge_difference = shortcuts.size() - outgoing_edges[v].size() - incoming_edges[v].size();
        int shortcut_cover = affected_neighbors.size();
        int contracted_neighbors = cont_neighbors.size();

        // Calculate and return the heuristic value
        return (1.0 * edge_difference) + (1.5 * shortcut_cover) + (0.5 * contracted_neighbors) + (1.0 * level[v]);
    }

    // Performs node contraction
    void contract_node(int v, std::vector<Shortcut>& shortcuts, int my_rank) {
        // Create a set to store the neighbors of v
        std::unordered_set<int> neighbors;

        // Add incoming neighbors of v to the set
        for (const auto& incoming_edge : incoming_edges[v]) {
            int u = incoming_edge.first;
            if (level[u] == 0) neighbors.insert(u);
        }

        // Add outgoing neighbors of v to the set
        for (const auto& outgoing_edge : outgoing_edges[v]) {
            int u = outgoing_edge.first;
            if (level[u] == 0) neighbors.insert(u);
        }

        // Update the levels of the neighbors of v
        for (const auto& u : neighbors) {
            level[u] = std::max(level[u], level[v] + 1);
        }

        // Add the calculated shortcuts to the graph
        for (const auto& shortcut : shortcuts) {
            add_directed_edge(shortcut.from, shortcut.to, shortcut.cost);
        }

        // Assign the rank to the contracted node v
        rank[v] = my_rank;
    }

    void set_n(int n) {
        N = n;
        outgoing_edges.resize(n);
        incoming_edges.resize(n);
    }

    void add_edge_to_list(std::vector<std::pair<int, int>>& list, int w, int c) {
        for (int i = 0; i < list.size(); ++i) {
            std::pair<int, int>& p = list[i];
            // This avoids parallel edges
            if (p.first == w) {
                if (p.second > c) {
                    p.second = c;
                }
                return;
            }
        }
        list.push_back(std::make_pair(w, c));
    }

    void add_directed_edge(int u, int v, int c) {
        if (u == v) return;  // This avoids self loops
        add_edge_to_list(outgoing_edges[u], v, c);
        add_edge_to_list(incoming_edges[v], u, c);
    }

    void add_edge(int u, int v, int c) {
        add_directed_edge(u, v, c);
    }

    // Remove unnecessary edges
    void finalize() {
        // Iterate through all vertices in the graph
        for (int v = 0; v < N; ++v) {

            // Iterate through the incoming edges of vertex v
            for (auto it = incoming_edges[v].begin(); it != incoming_edges[v].end(); ) {
                int u = it->first;

                // Check if the rank of u is less than the rank of v (downward edge)
                if (rank[u] < rank[v]) {
                    // If it's a downward edge, remove it
                    incoming_edges[v].erase(it);
                } else {
                    ++it;  // Keep it and move to the next edge
                }
            }

            // Iterate through the outgoing edges of vertex v
            for (auto it = outgoing_edges[v].begin(); it != outgoing_edges[v].end(); ) {
                int w = it->first;

                // Check if the rank of w is less than the rank of v (downward edge)
                if (rank[w] < rank[v]) {
                    // If it's a downward edge, remove it
                    outgoing_edges[v].erase(it);
                } else {
                    ++it;  // Keep it and move to the next edge
                }
            }
        }
    }

    bool read_stdin() {
        int u, v, c, n, m;
        assert(scanf("%d %d", &n, &m) == 2);
        set_n(n);
        for (int i = 0; i < m; ++i) {
            assert(scanf("%d %d %d", &u, &v, &c) == 3);
            add_edge(u-1, v-1, c);
        }
        return true;
    }
};

int main() {
    Graph g;
    g.preprocess();
    std::cout << "Ready" << std::endl;

    int t;
    assert(scanf("%d", &t) == 1);  // successfully read input
    for (int i = 0; i < t; ++i) {
        int u, v;
        assert(scanf("%d %d", &u, &v) == 2);  // successfully read input
        printf("%d\n", g.query(u-1, v-1));
    }
}
