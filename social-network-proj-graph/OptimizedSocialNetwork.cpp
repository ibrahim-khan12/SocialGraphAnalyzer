#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <fstream>
#include <algorithm>
#include <climits>
#include <chrono>
#include <functional>

using namespace std;
using namespace std::chrono;

struct Edge {
    int to, weight;
    Edge(int t, int w) : to(t), weight(w) {}
};

struct Node {
    int id, cost;
    Node(int i, int c) : id(i), cost(c) {}
    bool operator>(const Node& other) const { return cost > other.cost; }
};

struct AStarNode {
    int id, gCost, fCost;
    AStarNode(int i, int g, int f) : id(i), gCost(g), fCost(f) {}
    bool operator>(const AStarNode& other) const { return fCost > other.fCost; }
};

class Graph {
private:
    unordered_map<int, vector<Edge>> adj;
    unordered_map<int, int> influences;
    vector<int> nodes;

public:
    void addEdge(int u, int v, int w) {
        adj[u].emplace_back(v, w);
        adj[v].emplace_back(u, w);
    }
    
    void addInfluence(int node, int inf) {
        influences[node] = inf;
    }
    
    void addNode(int node) {
        if (find(nodes.begin(), nodes.end(), node) == nodes.end()) {
            nodes.push_back(node);
        }
    }
    
    int getHeuristic(int node) {
        return adj[node].size();
    }
    
    int getInfluence(int node) {
        return influences.count(node) ? influences[node] : 0;
    }
    
    // Optimized Dijkstra
    pair<vector<int>, int> dijkstra(int start, int end) {
        unordered_map<int, int> dist;
        unordered_map<int, int> prev;
        priority_queue<Node, vector<Node>, greater<Node>> pq;
        
        for (int node : nodes) dist[node] = INT_MAX;
        dist[start] = 0;
        pq.emplace(start, 0);
        
        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();
            
            if (current.id == end) break;
            if (current.cost > dist[current.id]) continue;
            
            for (const Edge& e : adj[current.id]) {
                int newDist = dist[current.id] + e.weight;
                if (newDist < dist[e.to]) {
                    dist[e.to] = newDist;
                    prev[e.to] = current.id;
                    pq.emplace(e.to, newDist);
                }
            }
        }
        
        vector<int> path;
        if (dist[end] != INT_MAX) {
            for (int at = end; at != start; at = prev[at]) {
                path.push_back(at);
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
        }
        
        return {path, dist[end]};
    }
    
    // Optimized A*
    pair<vector<int>, int> aStar(int start, int end) {
        unordered_map<int, int> gScore, fScore, prev;
        priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> pq;
        
        for (int node : nodes) {
            gScore[node] = INT_MAX;
            fScore[node] = INT_MAX;
        }
        
        gScore[start] = 0;
        fScore[start] = getHeuristic(start);
        pq.emplace(start, 0, fScore[start]);
        
        while (!pq.empty()) {
            AStarNode current = pq.top();
            pq.pop();
            
            if (current.id == end) {
                vector<int> path;
                for (int at = end; at != start; at = prev[at]) {
                    path.push_back(at);
                }
                path.push_back(start);
                reverse(path.begin(), path.end());
                return {path, gScore[end]};
            }
            
            if (current.gCost > gScore[current.id]) continue;
            
            for (const Edge& e : adj[current.id]) {
                int tentativeG = gScore[current.id] + e.weight;
                
                if (tentativeG < gScore[e.to]) {
                    prev[e.to] = current.id;
                    gScore[e.to] = tentativeG;
                    fScore[e.to] = tentativeG + getHeuristic(e.to);
                    pq.emplace(e.to, gScore[e.to], fScore[e.to]);
                }
            }
        }
        
        return {vector<int>(), -1};
    }
    
    // Dynamic Programming for longest increasing influence path
    pair<int, vector<int>> longestInfluencePath() {
        vector<int> sortedNodes = nodes;
        sort(sortedNodes.begin(), sortedNodes.end(), 
             [this](int a, int b) { return influences[a] < influences[b]; });
        
        unordered_map<int, int> dp, parent;
        for (int node : nodes) {
            dp[node] = 1;
            parent[node] = -1;
        }
        
        int maxLen = 1, maxNode = sortedNodes[0];
        
        for (int current : sortedNodes) {
            for (const Edge& e : adj[current]) {
                if (influences[e.to] > influences[current]) {
                    if (dp[current] + 1 > dp[e.to]) {
                        dp[e.to] = dp[current] + 1;
                        parent[e.to] = current;
                        
                        if (dp[e.to] > maxLen) {
                            maxLen = dp[e.to];
                            maxNode = e.to;
                        }
                    }
                }
            }
        }
        
        vector<int> path;
        for (int at = maxNode; at != -1; at = parent[at]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());
        
        return {maxLen, path};
    }
};

Graph loadData() {
    Graph g;
    
    // Load graph
    ifstream file("social-network-proj-graph.txt");
    int u, v, w;
    while (file >> u >> v >> w) {
        g.addEdge(u, v, w);
        g.addNode(u);
        g.addNode(v);
    }
    file.close();
    
    // Load influences
    ifstream inf("social-network-proj-Influences.txt");
    int node, influence;
    while (inf >> node >> influence) {
        g.addInfluence(node, influence);
    }
    inf.close();
    
    return g;
}

void runTests() {
    Graph g = loadData();
    cout << "Graph loaded successfully!" << endl;
    
    // Test with smaller node IDs for faster execution
    int start = 0, end = 100;
    
    cout << "\n=== SHORTEST PATH ALGORITHMS ===" << endl;
    
    // Dijkstra
    auto t1 = high_resolution_clock::now();
    auto dijResult = g.dijkstra(start, end);
    auto t2 = high_resolution_clock::now();
    auto dijTime = duration_cast<microseconds>(t2 - t1);
    
    cout << "Dijkstra's Algorithm:" << endl;
    cout << "Path length: " << dijResult.first.size() << endl;
    cout << "Total cost: " << dijResult.second << endl;
    cout << "Time: " << dijTime.count() << " μs" << endl;
    
    // A*
    t1 = high_resolution_clock::now();
    auto astarResult = g.aStar(start, end);
    t2 = high_resolution_clock::now();
    auto astarTime = duration_cast<microseconds>(t2 - t1);
    
    cout << "\nA* Algorithm:" << endl;
    cout << "Path length: " << astarResult.first.size() << endl;
    cout << "Total cost: " << astarResult.second << endl;
    cout << "Time: " << astarTime.count() << " μs" << endl;
    
    cout << "\nPerformance Comparison:" << endl;
    cout << "Dijkstra: " << dijTime.count() << " μs" << endl;
    cout << "A*: " << astarTime.count() << " μs" << endl;
    cout << "Difference: " << abs(dijTime.count() - astarTime.count()) << " μs" << endl;
    
    // Dynamic Programming
    cout << "\n=== LONGEST INFLUENCE PATH ===" << endl;
    t1 = high_resolution_clock::now();
    auto dpResult = g.longestInfluencePath();
    t2 = high_resolution_clock::now();
    auto dpTime = duration_cast<microseconds>(t2 - t1);
    
    cout << "Longest chain length: " << dpResult.first << endl;
    cout << "Chain: ";
    for (int i = 0; i < min(10, (int)dpResult.second.size()); i++) {
        cout << dpResult.second[i] << "(" << g.getInfluence(dpResult.second[i]) << ")";
        if (i < min(9, (int)dpResult.second.size() - 1)) cout << " -> ";
    }
    if (dpResult.second.size() > 10) cout << " ...";
    cout << endl;
    cout << "Time: " << dpTime.count() << " μs" << endl;
}

int main() {
    cout << "Social Network Analysis - Optimized Version" << endl;
    cout << "===========================================" << endl;
    
    try {
        runTests();
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}