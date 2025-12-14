# 🌐 SocialGraphAnalyzer

[![C++](https://img.shields.io/badge/C%2B%2B-11%2B-blue.svg)](https://en.cppreference.com/w/cpp/11)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

> **Advanced Social Network Analysis Using Graph Algorithms and Dynamic Programming**

SocialGraphAnalyzer is a high-performance C++ implementation for analyzing large-scale social networks. It provides efficient algorithms for finding optimal paths between users and discovering influence propagation patterns in social media networks.

## 🚀 Features

### 🔍 **Shortest Path Algorithms**
- **Dijkstra's Algorithm**: Classical shortest path with O((V+E)logV) complexity
- **A* Algorithm**: Heuristic-based pathfinding using node degree as heuristic
- **Performance Comparison**: Real-time execution time analysis

### 📈 **Influence Analysis**
- **Dynamic Programming**: Finds longest increasing influence chains
- **Social Hierarchy Detection**: Identifies optimal influence propagation paths
- **Viral Marketing Optimization**: Discovers best routes for information spread

### ⚡ **Performance Features**
- **Large Dataset Support**: Handles 40,000+ nodes and 120,000+ connections
- **Memory Optimized**: Efficient adjacency list representation
- **Fast Execution**: Microsecond-level performance tracking
- **Scalable Architecture**: Designed for real-world social networks

## 📊 Dataset Support

The system processes three main data files:
- **Graph Connections**: User relationships with weighted edges
- **Influence Scores**: User authority/influence ratings (1-100)
- **User Labels**: Human-readable user identifiers

## 🛠️ Installation

### Prerequisites
- **C++ Compiler**: g++, clang++, or MSVC with C++11 support
- **Operating System**: Windows, Linux, or macOS

### Quick Setup

```bash
# Clone the repository
git clone https://github.com/yourusername/SocialGraphAnalyzer.git
cd SocialGraphAnalyzer

# Compile the main program
g++ -std=c++11 -O2 -o SocialNetworkAnalysis SocialNetworkAnalysis.cpp

# Or use the provided Makefile
make
```

### Windows Users
```batch
# Use the automated compilation script
compile.bat
```

## 📋 Usage

### Interactive Mode
```bash
./SocialNetworkAnalysis
```
The program will prompt for:
- **Start Node**: Source user ID
- **End Node**: Target user ID

### Example Output
```
===== Social Network Analysis Project =====
Loading graph data...
Graph loaded successfully!
Total nodes: 39894

--- Dijkstra's Algorithm ---
Path: 0 (User A) -> 5159 (User B) -> 100 (User C)
Total distance: 45
Execution time: 1250 microseconds

--- A* Algorithm ---
Path: 0 (User A) -> 5159 (User B) -> 100 (User C)  
Total cost: 45
Heuristic: Node degree (number of connections)
Execution time: 890 microseconds

--- Performance Comparison ---
A* is faster by 360 microseconds

--- Longest Increasing Influence Path ---
Chain: 1389 (Influence: 15) -> 1392 (Influence: 21) -> ... -> 1860 (Influence: 100)
Longest chain length: 18 users
```

### Performance Testing
For automated benchmarking:
```bash
# Compile optimized version
g++ -std=c++11 -O2 -o OptimizedSocialNetwork OptimizedSocialNetwork.cpp

# Run performance tests
./OptimizedSocialNetwork
```

## 🏗️ Architecture

### Core Components

```cpp
class SocialNetworkGraph {
    // Adjacency list representation
    unordered_map<int, vector<Edge>> adjacencyList;
    
    // User influence tracking  
    unordered_map<int, int> influences;
    
    // Algorithm implementations
    pair<vector<int>, int> dijkstra(int start, int end);
    pair<vector<int>, int> aStar(int start, int end);
    pair<int, vector<int>> longestIncreasingInfluencePath();
};
```

### Algorithm Complexity

| Algorithm | Time Complexity | Space Complexity | Use Case |
|-----------|----------------|------------------|----------|
| Dijkstra  | O((V+E)log V)  | O(V)             | All shortest paths |
| A*        | O(E) - O(V²)   | O(V)             | Single target with heuristic |
| DP (LIP)  | O(V log V + E) | O(V)             | Influence propagation |

## 📈 Performance Results

Tested on dataset with **39,894 nodes** and **120,000 edges**:

- **Graph Loading**: ~500ms
- **Dijkstra Execution**: ~1.2ms average
- **A* Execution**: ~0.9ms average  
- **Influence Analysis**: ~2.1ms average
- **Memory Usage**: ~50MB for full dataset

## 🎯 Applications

### Social Media Analytics
- **Friend Recommendations**: Find shortest social distances
- **Influence Mapping**: Identify key opinion leaders
- **Content Routing**: Optimize message propagation

### Network Research
- **Community Detection**: Analyze social clusters
- **Information Spread**: Model viral content propagation
- **Social Hierarchy**: Understand influence structures

### Marketing Intelligence
- **Influencer Discovery**: Find optimal promotion paths
- **Campaign Optimization**: Maximize reach efficiency
- **Network Analysis**: Understand audience connections

## 📁 Project Structure

```
SocialGraphAnalyzer/
├── SocialNetworkAnalysis.cpp    # Main implementation
├── OptimizedSocialNetwork.cpp   # Performance-optimized version
├── ProjectReport.md             # Technical documentation
├── README.md                    # This file
├── Makefile                     # Build configuration
├── compile.bat                  # Windows build script
├── data/                        # Sample datasets
│   ├── social-network-proj-graph.txt
│   ├── social-network-proj-Influences.txt
│   └── social-network-proj-LABELS.txt
└── docs/                        # Additional documentation
```

## 🔬 Technical Details

### Data Format
```
# Graph file (node1 node2 weight)
0 4262 35
1 3822 41

# Influence file (node influence_score)
0 71
1 1

# Labels file (node user_name)
0 Christopher Kelly
1 Cynthia Martinez
```

### Heuristic Function
The A* algorithm uses **node degree** as heuristic:
```cpp
int getHeuristic(int node) {
    return adjacencyList[node].size();  // Number of connections
}
```

## 🤝 Contributing

1. **Fork** the repository
2. **Create** your feature branch (`git checkout -b feature/AmazingFeature`)
3. **Commit** your changes (`git commit -m 'Add some AmazingFeature'`)
4. **Push** to the branch (`git push origin feature/AmazingFeature`)
5. **Open** a Pull Request

### Development Guidelines
- Follow C++11 standards
- Add comprehensive comments
- Include performance benchmarks
- Update documentation

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🏆 Acknowledgments

- **FAST-NUCES** - National University of Computer and Emerging Sciences
- **Design and Analysis of Algorithms Course** - Fall 2025
- **Graph Theory Foundations** - Dijkstra, A*, and Dynamic Programming research

## 📞 Contact

**Project Team**: M.ibrahim , M.Ammar


---

⭐ **Star this repository if you find it helpful!**

[![GitHub stars](https://img.shields.io/github/stars/yourusername/SocialGraphAnalyzer.svg?style=social&label=Star&maxAge=2592000)](https://github.com/yourusername/SocialGraphAnalyzer/stargazers/)
