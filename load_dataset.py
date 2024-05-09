import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

class Dataset():

    """
    This class contains the dataset and methods to load and plot the data.
    attributes:
        - data: a dictionary containing the data of the form 
                {node: {Type: str(d) or str(c), X: float(x_pos), Y: float(y_pos), Demand: float(d), ServiceBy: str(D/T) or str(T)}}
        - headers: a list of headers of the data
        - graph: a networkx graph object containing the data (used for visualization)
    methods:
        - load_data: loads the data from the file and stores it in a dictionary
        - create_graph: creates a networkx graph object from the data
        - plot_data: plots the data using networkx (red for depot, light blue for customers only accesible by truck, light green for customers accesible by both, black for other nodes)
                    if show_demand is True, the demand of each node is shown on the plot
                    if scale_nodes is True, the size of the nodes is scaled according to the demand (default is True)
                    if show_labels is True, the labels of the nodes are shown on the plot
    """
    def __init__(self, data_path, create_graph=True):
        self.load_data(data_path)
        if create_graph:
            self.create_graph()
        
    def load_data(self, data_path):
        data = pd.read_csv(data_path, sep='\t\t', engine='python')
        self.headers = data.columns.to_list() #create a list of headers
        self.data = data.set_index('StringID').to_dict('index') #convert the data to a dictionary
    
    def create_graph(self):
        self.graph = nx.Graph()
        for node, attributes in self.data.items():
            self.graph.add_node(node, x=attributes['X'], y=attributes['Y'], pos=[attributes['X'], attributes['Y']], demand=attributes['Demand'], ServiceBy=attributes['ServiceBy'])

    def plot_data(self, show_demand=False, scale_nodes=True, show_labels=False):
        plt.figure(figsize=(10, 10))
        pos = nx.get_node_attributes(self.graph, 'pos')
        demand_labels = nx.get_node_attributes(self.graph, 'demand')
        service_by_labels = nx.get_node_attributes(self.graph, 'ServiceBy')

        standard_node_size = 400
        #plot depot node in red, customers only accesible by truck in light blue and customers accesible by both in light green
        node_colors = ['red' if node == 'D0' else 'lightblue' if service_by_labels.get(node) == 'T' else 'lightgreen' if service_by_labels.get(node) == 'D/T' else '-' for node in self.graph.nodes()]
        if scale_nodes:
            #scale_nodes by demand size, ensure depot node is visible as it has demand = 0
            node_sizes = [v * 20 if v > 0 else standard_node_size for v in demand_labels.values()]
        else:
            node_sizes = standard_node_size  # default size if not scaling

        nx.draw(self.graph, pos, with_labels=show_labels, node_size=node_sizes, node_color=node_colors) 

        red_patch = mpatches.Patch(color='red', label='Depot')
        blue_patch = mpatches.Patch(color='lightblue', label='Truck only')
        green_patch = mpatches.Patch(color='lightgreen', label='Drone/Truck')
        plt.legend(handles=[red_patch, blue_patch, green_patch])

        if show_demand:
            nx.draw_networkx_labels(self.graph, pos, labels=demand_labels)
        plt.show()
    
if __name__ == '__main__':
    dataset_path = 'dataset/0.3/80_20_0.3.txt'
    dataset = Dataset(dataset_path)
    #print(dataset.data)
    print(dataset.graph.nodes)
    dataset.plot_data(show_demand=False, scale_nodes=True, show_labels=True)