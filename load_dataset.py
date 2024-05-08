import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt

class Dataset():
    def __init__(self, data_path):
        self.load_data(data_path)
        self.create_graph()
        
    def load_data(self, data_path):
        data = pd.read_csv(data_path, sep='\t\t', engine='python')
        self.headers = data.columns.to_list() #create a list of headers
        self.data = data.set_index('StringID').to_dict('index') #convert the data to a dictionary
    
    def create_graph(self):
        self.graph = nx.Graph()
        for node, attributes in self.data.items():
            self.graph.add_node(node, pos=[attributes['X'], attributes['Y']], demand=attributes['Demand'])

    def plot_data(self, show_demand=False, scale_nodes=True):
        plt.figure(figsize=(10, 10))
        pos = nx.get_node_attributes(self.graph, 'pos')
        demand_labels = nx.get_node_attributes(self.graph, 'demand')

        if scale_nodes:
            node_sizes = [v * 20 for v in demand_labels.values()]  #comment out line if dont want to scale nodes based on demand
        nx.draw(self.graph, pos, with_labels=False, node_size=node_sizes)#change with_labels to True if want to show node ID's
        
        if show_demand:
            nx.draw_networkx_labels(self.graph, pos, labels=demand_labels)
        plt.show()
    

dataset_path = 'dataset/0.3/40_20_0.3.txt'
dataset = Dataset(dataset_path)
print(dataset.graph.nodes(data=True))
dataset.plot_data(show_demand=True, scale_nodes=True)