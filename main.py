import networkx as nx
import matplotlib.pyplot as plt
import heapq

# Define the graph
graph = {
    'A': {'B': 2, 'C': 3},
    'B': {'A': 2, 'C': 1, 'D': 4},
    'C': {'A': 3, 'B': 1, 'D': 2},
    'D': {'B': 4, 'C': 2}
}

# Create a networkx graph object
G = nx.Graph()
# Add nodes to the graph
for node in graph:
    G.add_node(node)
# Add edges to the graph
for node in graph:
    for neighbor, weight in graph[node].items():
        G.add_edge(node, neighbor, weight=weight)

# Ask the user for their choice
while True:
    try:
        choice = int(input("Enter 1 for shortest path or 2 for minimum spanning tree: "))
        if choice != 1 and choice != 2:
            raise ValueError
        break
    except ValueError:
        print("Invalid choice, please enter 1 or 2")

if choice == 1:
    # Ask user for the starting node
    start_node = input("Enter the starting node( A String value) : ")

    # Get the start and end nodes from the user
    end_node = input("Enter the end node( A String value) : ")

    # Check if the start and end nodes are present in the graph
    if start_node not in graph:
        print(f"{start_node} is not present in the graph.")
        exit()

    if end_node not in graph:
        print(f"{end_node} is not present in the graph.")
        exit()

    # Use Dijkstra's algorithm to find the shortest path
    shortest_paths = {node: float('inf') for node in graph}
    shortest_paths[start_node] = 0
    visited = set()
    predecessors = {}

    while True:
        # Get the node with the smallest tentative distance
        current_node = None
        min_distance = float('inf')
        for node in graph:
            if shortest_paths[node] < min_distance and node not in visited:
                current_node = node
                min_distance = shortest_paths[node]

        if current_node is None:
            break

        # Mark the current node as visited
        visited.add(current_node)

        # Update the tentative distances of the neighbors of the current node
        if current_node in graph:
            for neighbor, weight in graph[current_node].items():
                tentative_distance = shortest_paths[current_node] + weight
                if tentative_distance < shortest_paths[neighbor]:
                    shortest_paths[neighbor] = tentative_distance
                    predecessors[neighbor] = current_node

    # Use Dijkstra's algorithm to find the shortest path
    shortest_paths = {node: float('inf') for node in graph}
    shortest_paths[start_node] = 0

    while True:
        # Get the node with the smallest tentative distance
        current_node = None
        min_distance = float('inf')
        for node in graph:
            if shortest_paths[node] < min_distance and node not in visited:
                current_node = node
                min_distance = shortest_paths[node]

        if current_node is None:
            break

        # Mark the current node as visited
        visited.add(current_node)

        # Update the tentative distances of the neighbors of the current node
        if current_node in graph:
            for neighbor, weight in graph[current_node].items():
                tentative_distance = shortest_paths[current_node] + weight
                if tentative_distance < shortest_paths[neighbor]:
                    shortest_paths[neighbor] = tentative_distance
                    predecessors[neighbor] = current_node

    # Backtrack from the end node to the start node to get the shortest path
    shortest_path = []
    node = end_node
    while node != start_node:
        shortest_path.insert(0, node)
        node = predecessors[node]
    shortest_path.insert(0, start_node)

    # Print the shortest path
    print(f"The shortest path from {start_node} to {end_node} is: {' -> '.join(shortest_path)}")

    # Visualize the shortest path using networkx and matplotlib
    G = nx.Graph(graph)
    pos = nx.spring_layout(G)
    nx.draw_networkx_nodes(G, pos, node_size=500)
    nx.draw_networkx_edges(G, pos, edgelist=G.edges(), width=1)
    nx.draw_networkx_labels(G, pos, font_size=20, font_family="sans-serif")
    nx.draw_networkx_edge_labels(G, pos, edge_labels=nx.get_edge_attributes(G, 'weight'))
    edge_list = list(zip(shortest_path, shortest_path[1:]))
    nx.draw_networkx_edges(G, pos, edgelist=edge_list, width=5, edge_color='r')
    plt.axis("off")
    plt.savefig("shortest_path.png")


else:
    # Prim's algorithm for finding the minimum spanning tree
    start_node = list(graph.keys())[0]
    mst = {start_node}
    edges = []
    for neighbor, weight in graph[start_node].items():
        heapq.heappush(edges, (weight, start_node, neighbor))

    num_edges = 0
    while len(edges) > 0:
        (weight, node1, node2) = heapq.heappop(edges)
        if node2 not in mst:
            mst.add(node2)
            num_edges += 1
            for neighbor, weight in graph[node2].items():
                if neighbor not in mst:
                    heapq.heappush(edges, (weight, node2, neighbor))

            # Print out the number of edges in the minimum spanning tree
            print("Number of edges in minimum spanning tree:", num_edges)

            # Visualize the minimum spanning tree using networkx and matplotlib
            mst_edges = []
            for node in mst:
                for neighbor, weight in graph[node].items():
                    if neighbor in mst:
                        mst_edges.append((node, neighbor, weight))

            G = nx.Graph()
            G.add_weighted_edges_from(mst_edges)
            pos = nx.spring_layout(G)
            nx.draw_networkx_nodes(G, pos)
            nx.draw_networkx_edges(G, pos)
            nx.draw_networkx_edge_labels(G, pos, edge_labels={(u, v): d['weight'] for u, v, d in G.edges(data=True)})
            nx.draw_networkx_labels(G, pos)
            plt.savefig("minimum_spanning_tree.png")
