import heapq
import matplotlib.pyplot as plt
import networkx as nx

# Class representing a node in the graph
class Node:
    def __init__(self, name):
        self.name = name
        self.neighbors = {}

    def add_neighbor(self, neighbor, distance, cost):
        self.neighbors[neighbor] = (distance, cost)

# Class representing the graph with nodes and edges
class Graph:
    def __init__(self):
        self.nodes = {}

    def add_node(self, name):
        node = Node(name)
        self.nodes[name] = node
        return node

    def add_edge(self, from_node, to_node, distance, cost):
        if from_node not in self.nodes:
            self.add_node(from_node)
        if to_node not in self.nodes:
            self.add_node(to_node)
        self.nodes[from_node].add_neighbor(to_node, distance, cost)
        self.nodes[to_node].add_neighbor(from_node, distance, cost)  # For undirected graph

    def dijkstra_with_budget(self, start, end, budget):
        pq = [(0, 0, start)]  # (distance, cost, node)
        distances = {node: float('infinity') for node in self.nodes}
        costs = {node: float('infinity') for node in self.nodes}
        previous_nodes = {node: None for node in self.nodes}
        distances[start] = 0
        costs[start] = 0

        while pq:
            current_distance, current_cost, current_node = heapq.heappop(pq)

            if current_node == end:
                return self._reconstruct_path(previous_nodes, start, end), current_distance, current_cost

            for neighbor, (distance, cost) in self.nodes[current_node].neighbors.items():
                new_distance = current_distance + distance
                new_cost = current_cost + cost

                if new_cost <= budget and new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    costs[neighbor] = new_cost
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(pq, (new_distance, new_cost, neighbor))

        return [], float('infinity'), float('infinity')  # No valid path

    def _reconstruct_path(self, previous_nodes, start, end):
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous_nodes[current]
        return path[::-1]

    def visualize(self, path):
        G = nx.Graph()
        edge_labels = {}

        # Add edges with labels containing both distance and cost
        for node in self.nodes:
            for neighbor, (distance, cost) in self.nodes[node].neighbors.items():
                G.add_edge(node, neighbor, weight=distance)
                edge_labels[(node, neighbor)] = f"Dist: {distance}, Cost: {cost}"

        pos = nx.spring_layout(G)  # Position the nodes using spring layout
        plt.figure(figsize=(8, 6))
        nx.draw(G, pos, with_labels=True, node_size=700, node_color="lightblue", font_size=10, font_weight="bold")
        
        # Draw edge labels including both distance and cost
        nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_color="green")

        # Highlight the shortest path
        if path:
            edges_in_path = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
            nx.draw_networkx_edges(G, pos, edgelist=edges_in_path, width=2.5, edge_color="red")

        plt.title("Graph Representation with Shortest Path Highlighted")
        plt.show()

# Example usage
graph = Graph()
graph.add_edge('A', 'B', 2, 5)
graph.add_edge('A', 'C', 4, 10)
graph.add_edge('B', 'D', 7, 15)
graph.add_edge('C', 'D', 1, 3)
graph.add_edge('D', 'E', 5, 8)

start = 'A'
end = 'E'
budget = 25

# Run Dijkstra's algorithm with budget constraint
path, distance, cost = graph.dijkstra_with_budget(start, end, budget)

if distance != float('infinity'):
    print(f"Shortest path from {start} to {end} is {path} with a distance of {distance} and a cost of {cost}")
    graph.visualize(path)  # Visualize the graph and the shortest path
else:
    print(f"No path found from {start} to {end} within the budget of {budget}")
