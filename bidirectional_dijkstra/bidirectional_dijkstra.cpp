#include <cstdio>
#include <cassert>
#include <vector>
#include <queue>
#include <limits>
#include <utility>

using namespace std;

// External vector of size 2 - for forward and backward search.
// Internal 2-dimensional vector is vector of adjacency lists for each node.
typedef vector<vector<vector<int>>> Adj;

// Distances can grow out of int type
typedef long long Len;

// Vector of two priority queues - for forward and backward searches.
// Each priority queue stores the closest unprocessed node in its head.
typedef vector<priority_queue<pair<Len, int>, vector<pair<Len,int>>, greater<pair<Len,int>>>> Queue;

const Len INF = numeric_limits<Len>::max() / 4;

class Bidijkstra {
    // Number of nodes
    int n_;

    // Graph adj_[0] and cost_[0] correspond to the initial graph,
    // adj_[1] and cost_[1] correspond to the reversed graph.
    // Graphs are stored as vectors of adjacency lists corresponding
    // to nodes.
    // Adjacency list itself is stored in adj_, and the corresponding
    // edge costs are stored in cost_.
    Adj adj_;
    Adj cost_;

    // distance_[0] stores distances for the forward search,
    // and distance_[1] stores distances for the backward search.
    vector<vector<Len>> distance_;

    // Stores all the nodes visited either by forward or backward search.
    vector<int> workset_;

    // Stores a flag for each node which is True iff the node was visited
    // either by forward or backward search.
    vector<vector<bool>> visited_;

    // Variable to store the current best distance
    Len estimate = INF;

public:
    Bidijkstra(int n, Adj adj, Adj cost)
        : n_(n), adj_(adj), cost_(cost), distance_(2, vector<Len>(n, INF)), visited_(2, vector<bool>(n, false))
    { workset_.reserve(n); }  // Reserve memory for n elements in the vector

    // Initialize the data structures before new query,
    // clear the changes made by the previous query.
    // This function ensures that only the nodes affected
    // by the previous query are reset, improving performance.
    void clear() {
        for (int i = 0; i < workset_.size(); ++i) {
            int v = workset_[i];
            distance_[0][v] = distance_[1][v] = INF;
            visited_[0][v] = visited_[1][v] = false;
        }
        workset_.clear();
        estimate = INF;
    }

    // Processes visit of either forward or backward search 
    // (determined by value of side), to node v trying to
    // relax the current distance by dist.
    void visit(Queue& q, int side, int v, Len dist) {
        // Mark vertex v as visited
        visited_[side][v] = true;

        // Relax all the neighbors of the vertex v
        for (size_t index = 0; index < adj_[side][v].size(); ++index) {

            // Get neighbor ID
            int w = adj_[side][v][index];

            // Get the cost of the edge v-w
            Len cost = cost_[side][v][index];

            // Relax the vertex w
            if (distance_[side][w] > dist + cost) {
                distance_[side][w] = dist + cost;

                // Add w to the priority queue and to the set of processed nodes
                q[side].emplace(distance_[side][w], w);
                workset_.push_back(w);
            }
        }

        // Update the estimate distance if it's necessary
        if (estimate > distance_[0][v] + distance_[1][v]) {
            estimate = distance_[0][v] + distance_[1][v];
        }
    }

    // Returns the distance from s to t in the graph.
    Len query(int s, int t) {
        // Clear data structures
        clear();

        // Initialize data structures
        Queue q(2);
        distance_[0][s] = distance_[1][t] = 0;
        q[0].emplace(0, s);
        q[1].emplace(0, t);
        workset_.push_back(s);
        workset_.push_back(t);

        while (!q[0].empty() && !q[1].empty()) {
            // Forward Dijkstra
            // Process the current nearest node in the forward search by extracting the top element from the queue.
            auto top = q[0].top();
            q[0].pop();
            Len dist = top.first;
            int v = top.second;

            // Visit the node v by relaxing its neighbors and updating their distances.
            if (estimate >= dist) visit(q, 0, v, dist);
            
            // Check if the node has already been visited by the opposite search. If yes, return estimate distance.
            if (visited_[1][v]) return estimate;


            // Backward Dijkstra
            // Process the current nearest node in the backward search by extracting the top element from the queue.
            auto top_r = q[1].top();
            q[1].pop();
            Len dist_r = top_r.first;
            int v_r = top_r.second;

            // Visit the node v_r by relaxing its neighbors and updating their distances.
            if (estimate >= dist_r) visit(q, 1, v_r, dist_r);

            // Check if the node has already been visited by the opposite search. If yes, return estimate distance.
            if (visited_[0][v_r]) return estimate;
        }

        // There is no path between s and t
        return -1;
    }
};

int main() {
    int n, m;
    scanf("%d%d", &n, &m);
    Adj adj(2, vector<vector<int>>(n));
    Adj cost(2, vector<vector<int>>(n));

    for (int i = 0; i < m; ++i) {
        int u, v, c;
        scanf("%d%d%d", &u, &v, &c);
        adj[0][u-1].push_back(v-1);
        cost[0][u-1].push_back(c);
        adj[1][v-1].push_back(u-1);
        cost[1][v-1].push_back(c);
    }

    Bidijkstra bidij(n, adj, cost);

    int t;
    scanf("%d", &t);
    for (int i = 0; i < t; ++i) {
        int u, v;
        scanf("%d%d", &u, &v);
        printf("%lld\n", bidij.query(u-1, v-1));
    }
}
