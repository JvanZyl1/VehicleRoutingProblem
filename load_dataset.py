import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import itertools

class Dataset():

    """
    Class to represent, load and plot dataset
    
    """
    def __init__(self, data_path, create_graph=True):
        self.load_data(data_path)
        if create_graph:
            self.create_graph()
        
    def load_data(self, data_path):
        data = pd.read_csv(data_path, sep='\t\t', engine='python')
        self.headers = data.columns.to_list() #create a list of headers
        self.data = data.set_index('StringID').to_dict('index') #convert data to a dictionary

        #add new 'return depot' node 'D1' with the same properties as 'D0' (needed to get return time at depot)
        depot_props = self.data['D0']
        self.data['D1'] = depot_props
    
    def create_graph(self):
        self.graph = nx.Graph()
        for node, attributes in self.data.items():
            self.graph.add_node(node, x=attributes['X'], y=attributes['Y'], pos=[attributes['X'], attributes['Y']], demand=attributes['Demand'], ServiceBy=attributes['ServiceBy'])

    def plot_data(self, show_demand=False, scale_nodes=True, show_labels=False, active_routes=None, show_weights=False):
        plt.figure(figsize=(10, 10))
        pos = nx.get_node_attributes(self.graph, 'pos')
        demand_labels = nx.get_node_attributes(self.graph, 'demand')
        service_by_labels = nx.get_node_attributes(self.graph, 'ServiceBy')

        #colour and scale nodes by demand
        standard_node_size = 400
        node_colors = ['red' if node == 'D0' else 'lightblue' if service_by_labels.get(node) == 'T' else 'lightblue' if service_by_labels.get(node) == 'D/T' else 'red' for node in self.graph.nodes()]
        if scale_nodes:
            node_sizes = [v * 20 if v > 0 else standard_node_size for v in demand_labels.values()]
        else:
            node_sizes = standard_node_size

        nx.draw(self.graph, pos, with_labels=show_labels, node_size=node_sizes, node_color=node_colors) 

        red_patch = mpatches.Patch(color='red', label='Depot')
        blue_patch = mpatches.Patch(color='lightblue', label='Truck only')
        green_patch = mpatches.Patch(color='lightgreen', label='Drone/Truck')
        plt.legend(handles=[red_patch, blue_patch, green_patch])

        if show_demand:
            nx.draw_networkx_labels(self.graph, pos, labels=demand_labels)

        #plot active routes with edges with correpsonding truck colour
        if active_routes:
            color_cycle = itertools.cycle(plt.rcParams['axes.prop_cycle'].by_key()['color'])
            for vehicle, route in active_routes.items():
                vehicle_color = next(color_cycle)
                for i in range(len(route)):
                    nx.draw_networkx_edges(self.graph, pos, edgelist=[(route[i][0], route[i][1])], edge_color=vehicle_color, width=2)
                    if show_weights:
                        edge_label = {(route[i][0], route[i][1]): route[i][3]}
                        nx.draw_networkx_edge_labels(self.graph, pos, edge_labels=edge_label, font_color=vehicle_color)
                plt.plot([], [], color=vehicle_color, label=vehicle)

        plt.legend()
        plt.show()
    
if __name__ == '__main__':
    dataset_path = 'dataset/0.3/80_20_0.3.txt'
    dataset = Dataset(dataset_path)
    #print(dataset.data)
    print(dataset.graph.nodes)
    dataset.plot_data(show_demand=False, scale_nodes=True, show_labels=True)