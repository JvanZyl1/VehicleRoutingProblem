import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch_geometric.nn import GCNConv
import numpy as np
from torch_geometric.data import Data
import matplotlib.pyplot as plt


# Settings for reproducibility
torch.manual_seed(42)
np.random.seed(42)

# Number of nodes
num_nodes = 50

# Node features: [daily passengers, number of lines, accessibility score, services, delay index]
# All features are initialized using a normal distribution for simplicity
mean_features = [300, 2, 0.8, 10, 0.15]  # mean values for features
std_features = [100, 1, 0.1, 5, 0.05]    # standard deviation for features

nodes_features = torch.randn((num_nodes, 5))
for i in range(5):
    nodes_features[:, i] = nodes_features[:, i] * std_features[i] + mean_features[i]

# Edges: Randomly connect stations, ensuring connectivity but with randomness
edges = []
for i in range(num_nodes):
    for j in range(i + 1, num_nodes):
        if np.random.rand() > 0.95:  # Sparse connectivity
            edges.append([i, j])
            edges.append([j, i])

edges = torch.tensor(edges, dtype=torch.long).t()

# Edge attributes: [min, avg, max travel times, reliability]
# Initialized similarly using a normal distribution
mean_edges = [5, 7, 10, 0.9]  # mean values for edge attributes
std_edges = [1, 2, 3, 0.1]    # standard deviation for edge attributes

num_edges = edges.size(1)
edges_attr = torch.randn((num_edges, 4))
for i in range(4):
    edges_attr[:, i] = edges_attr[:, i] * std_edges[i] + mean_edges[i]

# Create the graph data structure
graph_data = Data(x=nodes_features, edge_index=edges, edge_attr=edges_attr)
    

class MetroGNN(torch.nn.Module):
    def __init__(self):
        super(MetroGNN, self).__init__()
        self.conv1 = GCNConv(5, 16)  # Assuming 5 features per node, 16 output features
        self.conv2 = GCNConv(16, 4)  # 4 policies as output
        self.out_features = 4

    def forward(self, x, edge_index, edge_attr):
        # Assuming edge_attr's first column is the edge weight
        edge_weight = edge_attr[:, 0]  # Make sure this is the correct column for the weight
        x = self.conv1(x, edge_index, edge_weight=edge_weight)
        x = torch.relu(x)
        x = self.conv2(x, edge_index, edge_weight=edge_weight)
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

number_of_actions = 50

class GNN_RL(nn.Module):
    def __init__(self, gnn_model):
        super(GNN_RL, self).__init__()
        self.gnn = gnn_model
        self.policy = nn.Sequential(
            nn.Linear(gnn_model.out_features, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, number_of_actions),
            nn.Softmax(dim=-1)
        )

    def forward(self, graph_data):
        node_embeddings = self.gnn(graph_data.x, graph_data.edge_index, graph_data.edge_attr)
        state = torch.mean(node_embeddings, dim=0)  # Example state representation
        return self.policy(state)
    

def compute_reward(predicted_actions):
    # Example reward computation
    # Assume that the reward is the sum of the predicted actions
    return torch.sum(predicted_actions)



# Instantiate models
gnn_model = MetroGNN()
combined_model = GNN_RL(gnn_model)

# RL training loop
optimizer = optim.Adam(combined_model.parameters(), lr=0.01)
for epoch in range(1000):
    predicted_actions = combined_model(graph_data)
    reward = compute_reward(predicted_actions)  # Define your reward computation
    loss = -torch.log(predicted_actions) * reward
    loss = loss.sum()
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()


# Plot the loss
import matplotlib.pyplot as plt
plt.plot(losses)
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.title('Training Loss Curve')
plt.show()