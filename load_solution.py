import os
from load_dataset import Dataset
import numpy as np
from scipy.spatial import distance

##Functions##
def load_solution_variables(solution_file_path):
    solution = {}
    delta = 1e-6  # Small delta for numerical errors
    with open(solution_file_path, 'r') as file:
        for line in file:
            # Check if the line contains a space, indicating a possible variable assignment
            if ' ' in line:
                parts = line.split()
                if len(parts) == 2:  # Ensure the line is in the format 'variable_name value'
                    try:
                        variable_name = parts[0]
                        value = float(parts[1])
                        
                        # Check if the variable is active
                        if value > delta:
                            solution[variable_name] = value
                    except ValueError:
                        # If conversion to float fails, skip the line
                        continue
    return solution

def extract_active_routes(solution, N, Tr):
    
    # Extract active trucks
    active_trucks = [v for v in Tr if any(key.startswith(f'y[{v}]') and value >= 0.99 for key, value in solution.items())]

    # Extract routes for active trucks
    active_routes_truck = {}
    for truck in active_trucks:
        active_routes_truck[truck] = []
        for node_from in N:
            for node_to in N:
                if node_from != node_to:
                    key = f'x[{truck},{node_from},{node_to}]'
                    if solution.get(key, 0) >= 0.99:
                        timestamp = solution.get(f't[{truck},{node_to}]', 0)
                        weight = solution.get(f'W_dynamic[{truck},{node_from},{node_to}]', 0)
                        active_routes_truck[truck].append((node_from, node_to, timestamp, np.round(weight, 1)))
    
    return active_routes_truck, active_trucks

def get_manhattan_distance(data):
    """
    Returns a dictionary with manhattan distances between all nodes in dataset
    """
    distance_dict = {}
    for node1 in data.keys():
        for node2 in data.keys():
            distance_dict[node1, node2] = distance.cityblock([data[node1]['X'], data[node1]['Y']], [data[node2]['X'], data[node2]['Y']])
    return distance_dict

def get_time_dict(data, S_T, distance_dict):
    """
    Returns a dictionary with travel times between all nodes in dataset
    """
    time_dict = {}
    for node1 in data.keys():
        for node2 in data.keys():
            time_dict[node1, node2] = distance_dict[node1, node2] / S_T
    return time_dict

def check_in_x_var(i, j):
    """Check if the route from node i to node j is valid instead of having to pass bunch of conditions in each constraint.
    Conditions:
        1- cant travel between same node (i != j)
        2- cant leave return depot D1 (i != 'D1')
        3- cant arrive at start depot D0 (j != 'D0')
        4- cant travel from D0 to D1 (not (i == 'D0' and j == 'D1'))
            note that constraints 2 & 3 already ensure you cant travel from D1 to D0
    """
    if i != j and i != 'D1' and j != 'D0' and not (i == 'D0' and j == 'D1'):
        return True
    return False

def get_cost_obj(C_T, C_B, truck_distance_dict, active_routes_truck, cost_weight=1):
    """
    Returns the cost objective value for the model after it has optimised
    """
    cost_obj = 0
    for truck in active_routes_truck.keys():
        cost_obj += cost_weight * C_B  # add base cost for using a truck
        for route in active_routes_truck[truck]:
            i, j, _, _ = route
            cost_obj += cost_weight * C_T * truck_distance_dict[i, j]
    return cost_obj

def get_environmental_obj(truck_distance_dict, active_routes_truck, environmental_weight=1):
    """
    Returns the environmental objective value for the model after it has optimised
    """
    environmental_obj = 0
    for truck in active_routes_truck.keys():
        for route in active_routes_truck[truck]:
            i, j, _, W_dynamic = route
            environmental_obj += environmental_weight * W_dynamic * truck_distance_dict[i, j]
    return environmental_obj

def get_time_obj(active_routes_truck, time_weight=1):
    """
    Returns the time objective value for the model after it has optimised
    """
    time_obj = 0
    for truck in active_routes_truck.keys():
        for route in active_routes_truck[truck]:
            _, _, timestamp, _ = route
            time_obj = max(time_obj, timestamp)
    return time_weight * time_obj
## LOAD DATASET ##
current_dir = os.getcwd()
# Select which data folder to use
data_subfolder = '0.3'
data_subfoldercopy = '0.3_copy'
data_num_nodes = '40'
data_area = '40'

data_file_name = f'{data_num_nodes}_{data_area}_{data_subfoldercopy}'
dataset_path = f'dataset/{data_subfolder}/{data_file_name}.txt'
output_solution_file_path = os.path.join(current_dir, data_file_name + '_solution.sol')#used to save solution file
output_model_file_path = os.path.join(current_dir, data_file_name + '_model.lp')#used to save model file
dataset = Dataset(dataset_path)
N = list(dataset.data.keys()) #set of nodes with depot at start (D0) and at end (D1)
num_trucks = 5
Tr = [f'Tr{i}' for i in range(1, num_trucks+1)] #set of trucks

## MODEL PARAMETERS ##
W_T = 1500 #empty weight truck [kg]
Q_T = 1000 #load capacity of trucks [kg]
#W_D = 25 #empty weight drone [kg]
#Q_D = 5 #load capacity of drones [kg]
C_T = 25 #travel cost of trucks per unit distance [monetary unit/km]
#C_D = 1 #travel cost of drones per unit distance [monetary unit/km]
C_B = 500 #basis cost of using a truck equipped with a drone [monetary unit]
#E = 0.5 #maximum endurance of empty drones [hours]
S_T = 60 #average travel speed of the trucks [km/h]
#S_D = 65 #average travel speed of the drones [km/h]


# Load solution variables
solution = load_solution_variables(output_solution_file_path)
active_routes, active_trucks = extract_active_routes(solution, N, Tr)
# Sort the routes for each truck according to the timestamps
for truck in active_trucks:
    active_routes[truck].sort(key=lambda x: x[2])

print('Active trucks:', active_trucks)
print('Active routes for trucks:', active_routes)

# Print each seperate objective value after optimisation
truck_distance_dict = get_manhattan_distance(dataset.data)
time_dict = get_time_dict(dataset.data, S_T, truck_distance_dict)
cost_weight = 1
environmental_weight = 0.01
time_weight = 60 * 60
print('Cost objective/Total delivery cost [$]:', get_cost_obj(C_T, C_B, truck_distance_dict, active_routes, cost_weight=cost_weight))
print('Environmental objective [kg * km]:', get_environmental_obj(truck_distance_dict, active_routes, environmental_weight=environmental_weight))
print('Time objective:', get_time_obj(active_routes, time_weight=time_weight))

#convert t_max to hours, minutes, seconds
t_max_hours = solution['t_max']
t_max_minutes, t_max_seconds = divmod(t_max_hours * 3600, 60)
t_max_hours, t_max_minutes = divmod(t_max_minutes, 60)
print('Max delivery time: {} hours, {} minutes, {} seconds'.format(int(t_max_hours), int(t_max_minutes), int(t_max_seconds)))

#use plotting method in dataset class to visualise
dataset.plot_data(show_demand=False, scale_nodes=True, show_labels=True, active_routes=active_routes, show_weights=True)





