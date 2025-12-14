# Social Network Analysis Project Report

## Project Overview
This project implements graph algorithms for social network analysis using C++. The implementation includes shortest path algorithms (Dijkstra and A*) and dynamic programming for finding the longest increasing influence path.

## Algorithms Implemented

### 1. Dijkstra's Algorithm

**Purpose**: Find shortest path between two nodes in a weighted graph.

**Pseudocode**:
```
DIJKSTRA(graph, start, end):
    Initialize distances[all nodes] = INFINITY
    Initialize previous[all nodes] = UNDEFINED
    distances[start] = 0
    
    CREATE priority queue Q with all nodes
    
    WHILE Q is not empty:
        current = node with minimum distance in Q
        REMOVE current from Q
        
        IF current == end:
            BREAK
            
        FOR each neighbor of current:
            alternative = distances[current] + weight(current, neighbor)
            IF alternative < distances[neighbor]:
                distances[neighbor] = alternative
                previous[neighbor] = current
    
    RECONSTRUCT path from start to end using previous array
    RETURN path and total distance
```

**Time Complexity**: O((V + E) log V)
- V = number of vertices, E = number of edges
- Using min-priority queue (binary heap)
- Each vertex is extracted once: O(V log V)
- Each edge is relaxed once: O(E log V)

**Space Complexity**: O(V)
- Distance array, previous array, and priority queue

### 2. A* Algorithm

**Purpose**: Find shortest path using heuristic function to guide search.

**Heuristic Function**: h(n) = number of direct connections (degree of node)

**Pseudocode**:
```
A_STAR(graph, start, end, heuristic):
    Initialize gScore[all nodes] = INFINITY
    Initialize fScore[all nodes] = INFINITY
    gScore[start] = 0
    fScore[start] = heuristic(start)
    
    CREATE open set with start node
    CREATE closed set (empty)
    
    WHILE open set is not empty:
        current = node in open set with lowest fScore
        
        IF current == end:
            RECONSTRUCT path
            RETURN path and cost
            
        MOVE current from open set to closed set
        
        FOR each neighbor of current:
            IF neighbor in closed set:
                CONTINUE
                
            tentative_gScore = gScore[current] + weight(current, neighbor)
            
            IF tentative_gScore < gScore[neighbor]:
                previous[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = tentative_gScore + heuristic(neighbor)
                
                IF neighbor not in open set:
                    ADD neighbor to open set
    
    RETURN failure (no path found)
```

**Time Complexity**: O(E) in best case, O(V²) in worst case
- Depends on heuristic quality
- With good heuristic, explores fewer nodes than Dijkstra
- Priority queue operations: O(log V) per operation

**Space Complexity**: O(V)
- gScore, fScore arrays and open/closed sets

### 3. Longest Increasing Influence Path (Dynamic Programming)

**Purpose**: Find longest path where influence scores strictly increase.

**Pseudocode**:
```
LONGEST_INFLUENCE_PATH(graph, influences):
    SORT nodes by influence score (ascending)
    
    Initialize dp[all nodes] = 1
    Initialize parent[all nodes] = -1
    maxLength = 1, maxNode = first node
    
    FOR each node in sorted order:
        FOR each neighbor of node:
            IF influences[neighbor] > influences[node]:
                IF dp[node] + 1 > dp[neighbor]:
                    dp[neighbor] = dp[node] + 1
                    parent[neighbor] = node
                    
                    IF dp[neighbor] > maxLength:
                        maxLength = dp[neighbor]
                        maxNode = neighbor
    
    RECONSTRUCT longest path using parent array
    RETURN maxLength and path
```

**Time Complexity**: O(V log V + E)
- Sorting nodes: O(V log V)
- Processing each edge once: O(E)
- Overall: O(V log V + E)

**Space Complexity**: O(V)
- DP array, parent array, and sorted nodes array

## Implementation Details

### Data Structures Used:
1. **Adjacency List**: `unordered_map<int, vector<Edge>>`
   - Efficient for sparse graphs
   - O(1) average access time
   - Memory efficient

2. **Priority Queue**: For Dijkstra and A*
   - Min-heap implementation
   - O(log V) insertion and extraction

3. **Hash Maps**: For distance tracking and memoization
   - O(1) average access time
   - Used for influences, distances, and parent tracking

### Optimizations:
1. **Early Termination**: Stop when target is reached
2. **Memory Pool**: Reuse data structures where possible
3. **Efficient Comparators**: Custom comparison functions for priority queues
4. **Batch Processing**: Process multiple operations together

## Performance Analysis

### Theoretical Comparison:
| Algorithm | Time Complexity | Space Complexity | Best Use Case |
|-----------|----------------|------------------|---------------|
| Dijkstra  | O((V+E)logV)   | O(V)             | All shortest paths |
| A*        | O(E) to O(V²)  | O(V)             | Single target with good heuristic |
| DP (LIP)  | O(V log V + E) | O(V)             | Longest increasing paths |

### Expected Performance:
- **A*** should outperform **Dijkstra** when:
  - Good heuristic is available
  - Target is relatively close to source
  - Graph is not too dense

- **Dijkstra** may outperform **A*** when:
  - Heuristic is poor
  - Need paths to multiple targets
  - Very dense graphs

### Dataset Characteristics:
- **Nodes**: ~40,000 users
- **Edges**: ~120,000 connections
- **Graph Type**: Sparse, undirected, weighted
- **Influence Range**: 1-100

## Results and Analysis

### Performance Metrics:
1. **Execution Time**: Measured in microseconds
2. **Path Length**: Number of hops
3. **Total Cost**: Sum of edge weights
4. **Memory Usage**: Space complexity verification

### Test Cases:
- Short distance paths (nearby nodes)
- Long distance paths (far apart nodes)
- High influence vs low influence starting points
- Dense vs sparse graph regions

## Conclusions

1. **Algorithm Choice**: 
   - A* is generally faster for single-target shortest path
   - Dijkstra is more consistent and reliable
   - DP solution efficiently finds longest influence chains

2. **Heuristic Quality**: 
   - Node degree is a reasonable heuristic for social networks
   - Could be improved with geographic or community-based metrics

3. **Scalability**: 
   - All algorithms scale well with the given dataset size
   - Memory usage remains reasonable for larger networks

4. **Real-world Applications**:
   - Shortest paths: Friend recommendation, message routing
   - Influence chains: Viral marketing, information propagation

## Code Structure

### Files:
1. `SocialNetworkAnalysis.cpp` - Complete implementation with user interaction
2. `OptimizedSocialNetwork.cpp` - Performance-optimized version
3. `Makefile` - Build configuration

### Key Classes:
- `SocialNetworkGraph` - Main graph data structure
- `Edge` - Edge representation with weight
- `AStarNode` - Node for A* algorithm with costs

### Input Files:
- `social-network-proj-graph.txt` - Graph edges and weights
- `social-network-proj-Influences.txt` - User influence scores
- `social-network-proj-LABELS.txt` - User names/labels

## Future Improvements

1. **Bidirectional Search**: Implement bidirectional Dijkstra/A*
2. **Better Heuristics**: Use community detection or geographic data
3. **Parallel Processing**: Multi-threaded implementation for large graphs
4. **Memory Optimization**: Use more cache-friendly data structures
5. **Advanced DP**: Consider multiple influence types or weighted influences