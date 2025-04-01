#include <cstdio>
#include <cassert>
#include <vector>
#include <queue>
#include <limits>
#include <utility>
#include <cmath>

using namespace std;

// External vector of size 2 - for forward and backward search.
// Internal 2-dimensional vector is vector of adjacency lists for each node.
typedef vector<vector<vector<int>>> Adj;

// Distances can grow out of int type
typedef long long Len;

// Vector of two priority queues - for forward and backward searches.
// Each priority queue stores the closest unprocessed node in its head.
typedef vector<priority_queue<pair<double, int>,vector<pair<double,int>>,greater<pair<double,int>>>> Queue;

const Len INF = numeric_limits<Len>::max() / 4;

class AStar {
    int n_;
    Adj adj_;
    Adj cost_;
    vector<vector<Len>> distance_;
    vector<int> workset_;
    vector<vector<bool>> visited_;
    // Coordinates of the nodes
    std::vector<std::pair<Len,Len>> xy_;

    // Variable to store the current best distance
    Len estimate = INF;

public:
    AStar(int n, Adj adj, Adj cost, std::vector<std::pair<Len,Len>> xy)
        : n_(n), adj_(adj), cost_(cost), distance_(2, vector<Len>(n_, INF)), visited_(2, vector<bool>(n_, false)), xy_(xy)
        { workset_.reserve(n); }

    // See the description of this method in the starter for friend_suggestion
    void clear() {
        for (int i = 0; i < workset_.size(); ++i) {
            int v = workset_[i];
            distance_[0][v] = distance_[1][v] = INF;
            visited_[0][v] = visited_[1][v] = false;
        }
        workset_.clear();
        estimate = INF;
    }

    double euclidean_distance(int u, int v) {
        double dx = (xy_[u].first - xy_[v].first);
        double dy = (xy_[u].second - xy_[v].second);
        return sqrt(dx * dx + dy * dy);
    }

    // Processes visit of either forward or backward search 
    // (determined by value of side), to node v trying to
    // relax the current distance by dist.
    void visit(Queue& q, int side, int v, double dist, int source, int target) {
        if (visited_[side][v] == true) return;

        // Mark vertex v as visited
        visited_[side][v] = true;

        // Relax all the neighbors of the vertex v
        for (size_t index = 0; index < adj_[side][v].size(); ++index) {

            // Get neighbor ID
            int w = adj_[side][v][index];

            // Get the cost of the edge v-w
            Len cost = cost_[side][v][index];

            // Relax the vertex w
            if (distance_[side][w] > distance_[side][v] + cost) {
                distance_[side][w] = distance_[side][v] + cost;

                // Add w to the priority queue and to the set of processed nodes
                q[side].emplace(distance_[side][w] + (euclidean_distance(w, target) - euclidean_distance(w, source)) / 2, w);
                workset_.push_back(w);
            }
        }

        // Update the estimate distance if it's necessary
        if (estimate > distance_[0][v] + distance_[1][v]) {
            estimate = distance_[0][v] + distance_[1][v];
        }
    }

    // Returns the distance from s to t in the graph
    Len query(int s, int t) {
        // Clear data structures
        clear();

        // Initialize data structures
        Queue q(2);
        distance_[0][s] = distance_[1][t] = 0;
        Len distance_s_t = euclidean_distance(s, t);
        q[0].emplace(0 + distance_s_t, s);
        q[1].emplace(0 + distance_s_t, t);
        workset_.push_back(s);
        workset_.push_back(t);

        while (!q[0].empty() && !q[1].empty()) {
            // Forward A*
            // Process the current nearest node in the forward search by extracting the top element from the queue.
            auto top = q[0].top();
            q[0].pop();
            double dist = top.first;
            int v = top.second;
            
            // Visit the node v by relaxing its neighbors and updating their distances.
            if (estimate >= dist) visit(q, 0, v, dist, s, t);
            
            // Check if the node has already been visited by the opposite search. If yes, return estimate distance.
            if (visited_[1][v]) return estimate;


            // Backward A*
            // Process the current nearest node in the backward search by extracting the top element from the queue.
            auto top_r = q[1].top();
            q[1].pop();
            double dist_r = top_r.first;
            int v_r = top_r.second;

            // Visit the node v_r by relaxing its neighbors and updating their distances.
            if (estimate >= dist_r) visit(q, 1, v_r, dist_r, t, s);

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
    std::vector<std::pair<Len, Len>> xy(n);
    for (int i = 0; i < n; ++i){
        int a, b;
        scanf("%d%d", &a, &b);
        xy[i] = make_pair(a, b);
    }
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

    AStar astar(n, adj, cost, xy);

    int t;
    scanf("%d", &t);
    for (int i = 0; i < t; ++i) {
        int u, v;
        scanf("%d%d", &u, &v);
        printf("%lld\n", astar.query(u-1, v-1));
    }
}
