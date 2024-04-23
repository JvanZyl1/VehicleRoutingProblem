import torch
from torch_geometric.data import Data
from torch_geometric.nn import GCNConv
import matplotlib.pyplot as plt

# Define the graph
# Nodes: 10 metro stations, with more specific features
num_nodes = 10
# Features for each node: [average daily passengers, number of lines, accessibility score]
nodes_features = torch.tensor([
    [300, 2, 0.9], [150, 1, 0.7], [500, 3, 0.8], [200, 1, 0.85],
    [600, 2, 0.95], [100, 1, 0.65], [450, 2, 0.75], [250, 1, 0.9],
    [350, 2, 0.8], [400, 3, 0.85]
], dtype=torch.float)

# Edges: Connections between stations
# Example edges with attributes representing travel time in minutes
edges = torch.tensor([
    [0, 1], [1, 2], [2, 3], [3, 4],
    [4, 5], [5, 6], [6, 7], [7, 8],
    [8, 9], [9, 0], [1, 3], [2, 4]
], dtype=torch.long).t()
edges_attr = torch.tensor([5, 3, 10, 4, 7, 6, 5, 8, 9, 10, 2, 3], dtype=torch.float).unsqueeze(1)

# Create the graph data structure
graph_data = Data(x=nodes_features, edge_index=edges, edge_attr=edges_attr)

# Define a simple GNN model
class MetroGNN(torch.nn.Module):
    def __init__(self):
        super(MetroGNN, self).__init__()
        self.conv1 = GCNConv(3, 16)  # Input feature dimension is 3
        self.conv2 = GCNConv(16, 4)  # Output dimension is 4 (one for each policy)

    def forward(self, x, edge_index, edge_attr):
        x = self.conv1(x, edge_index, edge_attr)
        x = torch.relu(x)
        x = self.conv2(x, edge_index, edge_attr)
        return x

# Instantiate the model, optimizer, and loss function
model = MetroGNN()
optimizer = torch.optim.Adam(model.parameters(), lr=0.01)
criterion = torch.nn.MSELoss()

# Policies: [minimize travel time, maximize coverage, minimize transfers, maximize accessibility]
# Example target values for policies (normalized between 0 and 1 for simplicity)
target_policies = torch.rand((num_nodes, 4))

losses = []
# Train the model
def train():
    model.train()
    optimizer.zero_grad()
    out = model(graph_data.x, graph_data.edge_index, graph_data.edge_attr)
    loss = criterion(out, target_policies)
    loss.backward()
    optimizer.step()
    return loss.item()

# Example training loop
for epoch in range(100):
    loss = train()
    losses.append(loss)
    print(f"Epoch {epoch+1}: Loss = {loss:.4f}")

# Predict using the model
model.eval()
with torch.no_grad():
    predictions = model(graph_data.x, graph_data.edge_index, graph_data.edge_attr)
    print("Predictions:", predictions)

# Plot the graph
# Plot loss curve
plt.plot(losses)
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.title('Training Loss Curve')
plt.show()


# Show the graph
import networkx as nx

G = nx.Graph()
for i in range(num_nodes):
    G.add_node(i, features=nodes_features[i].numpy())

for i in range(edges.size(1)):
    src, dst = edges[:, i]
    G.add_edge(src.item(), dst.item(), weight=edges_attr[i].item())

pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True, node_size=3000, node_color='skyblue', font_size=10, font_color='black')
edge_labels = {(src, dst): attr['weight'] for src, dst, attr in G.edges(data=True)}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)



plt.show()