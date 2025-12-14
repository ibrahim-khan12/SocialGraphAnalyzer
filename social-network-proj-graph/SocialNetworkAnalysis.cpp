#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <fstream>
#include <algorithm>
#include <climits>
#include <chrono>
#include <string>
#include <sstream>

using namespace std;
using namespace std::chrono;

// Structure to represent an edge with weight
struct Edge {
    int to;
    int weight;
    
    Edge(int to, int weight) : to(to), weight(weight) {}
};

// Structure for A* node
struct AStarNode {
    int node;
    int gCost;      // Actual cost from start
    int fCost;      // g(n) + h(n)
    
    AStarNode(int node, int gCost, int fCost) : node(node), gCost(gCost), fCost(fCost) {}
    
    // For priority queue (min-heap based on fCost)
    bool operator>(const AStarNode& other) const {
        return fCost > other.fCost;
    }
};

// Graph class for social network
class SocialNetworkGraph {
private:
    unordered_map<int, vector<Edge>> adjacencyList;
    unordered_map<int, int> influences;
    unordered_map<int, string> userLabels;
    unordered_set<int> nodes;
    
public:
    // Add edge to undirected graph
    void addEdge(int from, int to, int weight) {
        adjacencyList[from].push_back(Edge(to, weight));
        adjacencyList[to].push_back(Edge(from, weight));
        nodes.insert(from);
        nodes.insert(to);
    }
    
    // Add influence score for a user
    void addInfluence(int user, int influence) {
        influences[user] = influence;
    }
    
    // Add user label
    void addUserLabel(int user, const string& label) {
        userLabels[user] = label;
    }
    
    // Get heuristic value for A* (number of direct connections)
    int getHeuristic(int node) {
        return adjacencyList[node].size();
    }
    
    // Get influence score
    int getInfluence(int user) {
        return influences.count(user) ? influences[user] : 0;
    }
    
    // Get user label
    string getUserLabel(int user) const {
        auto it = userLabels.find(user);
        return it != userLabels.end() ? it->second : "Unknown";
    }
    // Get all nodes
    const unordered_set<int>& getNodes() const {
        return nodes;
    }
    // Get neighbors of a node
    const vector<Edge>& getNeighbors(int node) const {
        auto it = adjacencyList.find(node);
        if (it != adjacencyList.end()) return it->second;
        static const vector<Edge> empty;
        return empty;
    }
    
    // Check if node exists
    bool hasNode(int node) {
        return nodes.count(node) > 0;
    }
    
    // Dijkstra's Algorithm
    pair<vector<int>, vector<int>> dijkstra(int start, int end) {
        unordered_map<int, int> distances;
        unordered_map<int, int> previous;
        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
        
        // Initialize distances
        for (int node : nodes) {
            distances[node] = INT_MAX;
        }
        distances[start] = 0;
        
        pq.push({0, start});
        
        while (!pq.empty()) {
            int currentDistance = pq.top().first;
            int currentNode = pq.top().second;
            pq.pop();
            
            if (currentNode == end) break;
            
            if (currentDistance > distances[currentNode]) continue;
            
            for (const Edge& edge : adjacencyList[currentNode]) {
                int neighbor = edge.to;
                int newDistance = distances[currentNode] + edge.weight;
                
                if (newDistance < distances[neighbor]) {
                    distances[neighbor] = newDistance;
                    previous[neighbor] = currentNode;
                    pq.push({newDistance, neighbor});
                }
            }
        }
        
        // Reconstruct path
        vector<int> path;
        if (distances[end] != INT_MAX) {
            int current = end;
            while (current != start) {
                path.push_back(current);
                current = previous[current];
            }
            path.push_back(start);
            reverse(path.begin(), path.end());
        }
        
        vector<int> dist;
        for (int node : nodes) {
            dist.push_back(distances[node]);
        }
        
        return {path, dist};
    }
    
    // A* Algorithm
    pair<vector<int>, int> aStar(int start, int end) {
        unordered_map<int, int> gScore;
        unordered_map<int, int> fScore;
        unordered_map<int, int> previous;
        priority_queue<AStarNode, vector<AStarNode>, greater<AStarNode>> openSet;
        unordered_set<int> closedSet;
        
        // Initialize scores
        for (int node : nodes) {
            gScore[node] = INT_MAX;
            fScore[node] = INT_MAX;
        }
        
        gScore[start] = 0;
        fScore[start] = getHeuristic(start);
        openSet.push(AStarNode(start, 0, fScore[start]));
        
        while (!openSet.empty()) {
            AStarNode current = openSet.top();
            openSet.pop();
            
            if (current.node == end) {
                // Reconstruct path
                vector<int> path;
                int currentNode = end;
                while (currentNode != start) {
                    path.push_back(currentNode);
                    currentNode = previous[currentNode];
                }
                path.push_back(start);
                reverse(path.begin(), path.end());
                return {path, gScore[end]};
            }
            
            closedSet.insert(current.node);
            
            for (const Edge& edge : adjacencyList[current.node]) {
                int neighbor = edge.to;
                
                if (closedSet.count(neighbor)) continue;
                
                int tentativeGScore = gScore[current.node] + edge.weight;
                
                if (tentativeGScore < gScore[neighbor]) {
                    previous[neighbor] = current.node;
                    gScore[neighbor] = tentativeGScore;
                    fScore[neighbor] = tentativeGScore + getHeuristic(neighbor);
                    
                    openSet.push(AStarNode(neighbor, gScore[neighbor], fScore[neighbor]));
                }
            }
        }
        
        return {vector<int>(), -1}; // No path found
    }
    
    // Dynamic Programming: Longest Increasing Influence Path
    pair<int, vector<int>> longestIncreasingInfluencePath() {
        unordered_map<int, int> dp;
        unordered_map<int, int> parent;
        int maxLength = 0;
        int maxNode = -1;
        
        // Sort nodes by influence score
        vector<int> sortedNodes(nodes.begin(), nodes.end());
        sort(sortedNodes.begin(), sortedNodes.end(), [this](int a, int b) {
            return influences[a] < influences[b];
        });
        
        // Initialize DP
        for (int node : nodes) {
            dp[node] = 1;
            parent[node] = -1;
        }
        
        // Fill DP table
        for (int i = 0; i < sortedNodes.size(); i++) {
            int current = sortedNodes[i];
            
            for (const Edge& edge : adjacencyList[current]) {
                int neighbor = edge.to;
                
                // If neighbor has higher influence, update DP
                if (influences[neighbor] > influences[current]) {
                    if (dp[current] + 1 > dp[neighbor]) {
                        dp[neighbor] = dp[current] + 1;
                        parent[neighbor] = current;
                    }
                }
            }
            
            // Update maximum
            if (dp[current] > maxLength) {
                maxLength = dp[current];
                maxNode = current;
            }
        }
        
        // Reconstruct longest path
        vector<int> longestPath;
        int current = maxNode;
        while (current != -1) {
            longestPath.push_back(current);
            current = parent[current];
        }
        reverse(longestPath.begin(), longestPath.end());
        
        return {maxLength, longestPath};
    }
};

// File reading functions
SocialNetworkGraph loadGraph() {
    SocialNetworkGraph graph;
    
    // Load graph connections
    ifstream graphFile("social-network-proj-graph.txt");
    if (!graphFile.is_open()) {
        cerr << "Error: Could not open graph file!" << endl;
        return graph;
    }
    
    int from, to, weight;
    while (graphFile >> from >> to >> weight) {
        graph.addEdge(from, to, weight);
    }
    graphFile.close();
    
    // Load influences
    ifstream influenceFile("social-network-proj-Influences.txt");
    if (!influenceFile.is_open()) {
        cerr << "Error: Could not open influences file!" << endl;
        return graph;
    }
    
    int user, influence;
    while (influenceFile >> user >> influence) {
        graph.addInfluence(user, influence);
    }
    influenceFile.close();
    
    // Load labels
    ifstream labelFile("social-network-proj-LABELS.txt");
    if (!labelFile.is_open()) {
        cerr << "Error: Could not open labels file!" << endl;
        return graph;
    }
    
    string line;
    while (getline(labelFile, line)) {
        istringstream iss(line);
        int userId;
        string name;
        
        if (iss >> userId) {
            getline(iss, name);
            // Remove leading space
            if (!name.empty() && name[0] == ' ') {
                name = name.substr(1);
            }
            graph.addUserLabel(userId, name);
        }
    }
    labelFile.close();
    
    return graph;
}

void printPath(const vector<int>& path, const SocialNetworkGraph& graph) {
    if (path.empty()) {
        cout << "No path found!" << endl;
        return;
    }
    
    for (int i = 0; i < path.size(); i++) {
        cout << path[i] << " (" << graph.getUserLabel(path[i]) << ")";
        if (i < path.size() - 1) {
            cout << " -> ";
        }
    }
    cout << endl;
}

int main() {
    cout << "===== Social Network Analysis Project =====" << endl;
    cout << "Loading graph data..." << endl;
    
    SocialNetworkGraph graph = loadGraph();
    
    cout << "Graph loaded successfully!" << endl;
    cout << "Total nodes: " << graph.getNodes().size() << endl;
    
    // Get user input for start and end nodes
    int startNode, endNode;
    cout << "\nEnter start node: ";
    cin >> startNode;
    cout << "Enter end node: ";
    cin >> endNode;
    
    if (!graph.hasNode(startNode) || !graph.hasNode(endNode)) {
        cout << "Invalid nodes entered!" << endl;
        return 1;
    }
    
    cout << "\n===== PART 1: Shortest Path Algorithms =====" << endl;
    
    // Dijkstra's Algorithm
    cout << "\n--- Dijkstra's Algorithm ---" << endl;
    auto dijkstraStart = high_resolution_clock::now();
    auto dijkstraResult = graph.dijkstra(startNode, endNode);
    auto dijkstraEnd = high_resolution_clock::now();
    auto dijkstraDuration = duration_cast<microseconds>(dijkstraEnd - dijkstraStart);
    
    cout << "Path: ";
    printPath(dijkstraResult.first, graph);
    cout << "Execution time: " << dijkstraDuration.count() << " microseconds" << endl;
    
    // A* Algorithm
    cout << "\n--- A* Algorithm ---" << endl;
    auto astarStart = high_resolution_clock::now();
    auto astarResult = graph.aStar(startNode, endNode);
    auto astarEnd = high_resolution_clock::now();
    auto astarDuration = duration_cast<microseconds>(astarEnd - astarStart);
    
    cout << "Path: ";
    printPath(astarResult.first, graph);
    cout << "Total cost: " << astarResult.second << endl;
    cout << "Execution time: " << astarDuration.count() << " microseconds" << endl;
    
    // Performance comparison
    cout << "\n--- Performance Comparison ---" << endl;
    cout << "Dijkstra time: " << dijkstraDuration.count() << " μs" << endl;
    cout << "A* time: " << astarDuration.count() << " μs" << endl;
    cout << "Speed difference: " << abs(dijkstraDuration.count() - astarDuration.count()) << " μs" << endl;
    
    if (astarDuration < dijkstraDuration) {
        cout << "A* is faster by " << (dijkstraDuration.count() - astarDuration.count()) << " μs" << endl;
    } else {
        cout << "Dijkstra is faster by " << (astarDuration.count() - dijkstraDuration.count()) << " μs" << endl;
    }
    
    // Part 2: Dynamic Programming
    cout << "\n===== PART 2: Longest Increasing Influence Path =====" << endl;
    auto dpStart = high_resolution_clock::now();
    auto dpResult = graph.longestIncreasingInfluencePath();
    auto dpEnd = high_resolution_clock::now();
    auto dpDuration = duration_cast<microseconds>(dpEnd - dpStart);
    
    cout << "Longest chain length: " << dpResult.first << endl;
    cout << "Chain: ";
    for (int i = 0; i < dpResult.second.size(); i++) {
        int user = dpResult.second[i];
        cout << user << " (Influence: " << graph.getInfluence(user) << ")";
        if (i < dpResult.second.size() - 1) {
            cout << " -> ";
        }
    }
    cout << endl;
    cout << "Execution time: " << dpDuration.count() << " microseconds" << endl;
    
    return 0;
}