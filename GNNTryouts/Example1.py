import torch
from torch_geometric.data import Data
import numpy as np
import matplotlib.pyplot as plt
from torch_geometric.nn import GCNConv

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

    def forward(self, x, edge_index, edge_attr):
        # Assuming edge_attr's first column is the edge weight
        edge_weight = edge_attr[:, 0]  # Make sure this is the correct column for the weight
        x = self.conv1(x, edge_index, edge_weight=edge_weight)
        x = torch.relu(x)
        x = self.conv2(x, edge_index, edge_weight=edge_weight)
        return x


# Model instantiation and a dummy training loop would follow as previously described

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
    G.add_edge(src.item(), dst.item(), weight=edges_attr[i, 0].item())

pos = nx.spring_layout(G)
nx.draw(G, pos, with_labels=True, node_size=3000, node_color='skyblue', font_size=10, font_color='black')
edge_labels = {(src, dst): attr['weight'] for src, dst, attr in G.edges(data=True)}
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)



plt.show()


# Bring in some reinforcement learning concepts
# Path: Example2.py
import torch
import torch.nn.functional as F

class PolicyGradient(torch.nn.Module):
    def __init__(self):
        super(PolicyGradient, self).__init__()
        self.fc1 = torch.nn.Linear(4, 128)
        self.fc2 = torch.nn.Linear(128, 64)
        self.fc3 = torch.nn.Linear(64, 4)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.softmax(self.fc3(x), dim=-1)
        return x
    
# Instantiate the policy network
policy_net = PolicyGradient()

# Define the optimizer
optimizer = torch.optim.Adam(policy_net.parameters(), lr=0.01)

# Define the loss function
def compute_loss(log_probs, rewards):
    return -torch.sum(log_probs * rewards)

# Define the training loop
losses = []

def train_policy_gradient(num_episodes=50):

    for episode in range(num_episodes):
        # Sample a policy from the policy network
        policy = policy_net(torch.rand((1, 4)))

        # Sample an action based on the policy
        action = torch.multinomial(policy, num_samples=1)

        # Compute the reward based on the action
        reward = target_policies[0, action.item()]

        # Compute the log probability of the action
        log_prob = torch.log(policy[0, action.item()])

        # Compute the loss and update the policy network
        loss = compute_loss(log_prob, reward)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        print(f"Episode {episode+1}: Loss = {loss.item()}")
        losses.append(loss.item())

# Train the policy network
train_policy_gradient()

# Predict using the policy network
with torch.no_grad():
    policy = policy_net(torch.rand((1, 4)))
    print("Predicted Policy:", policy)

# This is a simple example of how you can combine graph neural networks with reinforcement learning concepts.

# This reinfor

# Plot the loss curve
plt.plot(losses)
plt.xlabel('Episode')
plt.ylabel('Loss')
plt.title('Policy Gradient Loss Curve')
plt.show()
