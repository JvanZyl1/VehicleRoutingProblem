## TRUCKS ONLY PROBLEM ##


from gurobipy import Model,GRB,LinExpr,quicksum
import numpy as np
from scipy.spatial import distance
import os
import socket
from load_dataset import Dataset

'''
# For Ugo's laptop (only needed for jupy notebooks)
# Define the node name or another identifier of your laptop
my_laptop_node = 'Ugos-MacBook-Pro.local'

# Get the current system's node name using socket.gethostname()
current_node = socket.gethostname()

if current_node == my_laptop_node:
    # Set the environment variable for Gurobi license file
    os.environ["GRB_LICENSE_FILE"] = "/Users/ugomunzi/gurobi/licenses/gurobi.lic"
    print("Gurobi license path set for Ugo's MacBook Pro.")
else:
    print("Not Ugo's MacBook Pro, using default or no specific license settings.")
'''

## MODEL PARAMETERS ##
W_T = 1500 #empty weight truck [kg]
Q_T = 1000 #load capacity of trucks [kg]
W_D = 25 #empty weight drone [kg]
Q_D = 5 #load capacity of drones [kg]
C_T = 25 #travel cost of trucks per unit distance [monetary unit/km]
C_D = 1 #travel cost of drones per unit distance [monetary unit/km]
C_B = 500 #basis cost of using a truck equipped with a drone [monetary unit]
E = 0.5 #maximum endurance of empty drones [hours]
S_T = 60 #average travel speed of the trucks [km/h]
S_D = 65 #average travel speed of the drones [km/h]
#Define Big M constant
M = 500 

## FUNCTIONS ##
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


## LOAD DATASET ##
current_dir = os.getcwd()
# Select which data folder to use
data_subfolder = '0.3'
data_subfoldercopy = '0.3_copy'
data_num_nodes = '40'
data_area = '20'

data_file_name = f'{data_num_nodes}_{data_area}_{data_subfoldercopy}'
dataset_path = f'dataset/{data_subfolder}/{data_file_name}.txt'
output_file_path = os.path.join(current_dir, data_file_name + '_solution.sol')#used to save solution file

dataset = Dataset(dataset_path)

## PRE-PROCESSING ##

num_trucks = 10 # set to high number, optimiser will decide how many truck to use
distance_dict = get_manhattan_distance(dataset.data)
time_dict = get_time_dict(dataset.data, S_T, distance_dict)

print(distance_dict)

#definitions of N_0, N and N_plus follow from paper
N = list(dataset.data.keys()) #set of nodes with depot at start
N_customers = N.copy()
N_customers.remove('D0')
V = [f'V{i}' for i in range(1, num_trucks+1)] #set of trucks


## DEFINE MODEL ##

# Create a new model
model = Model("Truck_Routing")

# Define decision variables
x = model.addVars(V, [(i, j) for i in N for j in N if i != j], lb=0, ub=1, vtype=GRB.BINARY, name='x')
y = model.addVars(V, lb=0, ub=1, vtype=GRB.BINARY, name='y')
t = model.addVars(V, N, lb=0, vtype=GRB.CONTINUOUS, name='t')

# Objective function (minimize cost both due to transportation and base cost of using truck if active)
cost_obj = quicksum(C_T * distance_dict[i,j] * x[v,i,j] for i in N for j in N if i != j for v in V) + quicksum(C_B * y[v] for v in V)
model.setObjective(cost_obj, GRB.MINIMIZE)

model.update()


# Constraint 1: Each customer is visited by exactly one truck

# Each customer is visited by exactly one truck
constraints = {}
# Loop over each customer
for customer in N_customers:
    # Initialize the sum for the current customer
    sum_for_current_customer = 0

    # Loop over each vehicle
    for vehicle in V:
        # Loop over each node
        for node in N:
            # Skip if customer is equal to node
            if customer != node:
                # Add the variable to the sum
                sum_for_current_customer += x[vehicle, node, customer]

    # The sum for the current customer must be equal to 1
    constraints[node] = sum_for_current_customer == 1

    # Add the constraints to the model
    model.addConstr(constraints[node], name='Each_customer_visited_once')


# Constraint 2: Each depot must be visited exactly once

# Each truck must leave the depot
# y - active
# 'D0' - depot
# Loop over each vehicle
for vehicle in V:
    # Initialize the sum for the current vehicle
    sum_for_current_vehicle = quicksum(x[vehicle, 'D0', customer] for customer in N_customers)
    
    #### THE ACTIVE PART IS WRONG, NEED TO FIX THIS !!!
    # Add the constraint to the model
    model.addConstr(sum_for_current_vehicle == 1, name=f'Truck_leaves_depot_{vehicle}')

    # Add the constraint to the model
    #model.addGenConstrIndicator(y[vehicle], 1, sum_for_current_vehicle == 1, name=f'Truck_leaves_depot_if_active_{vehicle}')


# Constraint 3: Each vehicle arrives at depot if active

# Each truck must return to the depot
# Loop over each vehicle
for vehicle in V:
    # Initialize the sum for the current vehicle
    sum_for_current_vehicle = quicksum(x[vehicle, customer, 'D0'] for customer in N_customers)
    
    #### THE ACTIVE PART IS WRONG, NEED TO FIX THIS !!!
    # Add the constraint to the model
    model.addConstr(sum_for_current_vehicle == 1, name=f'Truck_leaves_depot_{vehicle}')

    # Add the constraint to the model
    #model.addGenConstrIndicator(y[vehicle], 1, sum_for_current_vehicle == 1, name=f'Truck_leaves_depot_if_active_{vehicle}')


# Constraint 4: If a vehicle arrives at a customer node it must also leave

# If a truck visits a customer, it must leave the customer
for vehicle in V:
    for node in N_customers:
        model.addConstr(
            quicksum(x[vehicle, node, j] for j in N if j != node) == 
            quicksum(x[vehicle, j, node] for j in N if j != node),
            name=f'Flow_balance_{vehicle}_{node}'
        )
'''
#Constraint 5: Time at a node is equal or larger than time at previous nodes plus travel time (or irrelevant). Eliminates need for subtour constraints.
# Define a large constant M for the big-M method
'''
M_subtour = 6000000  # Make sure M is larger than the maximum possible travel time

# Add time constraints for all vehicles, nodes, and customers
for vehicle in V:
    for node in N:
        for customer in N:
            if node != customer:
                model.addConstr(
                    t[vehicle, customer] >= t[vehicle, node] + time_dict[(node, customer)] - M_subtour * (1 - x[vehicle, node, customer]),
                    name=f'Time_{vehicle}_{node}_{customer}'
                )

# Constraint 6: Payloads

# The total payload delivered to the customer must be less or equal to the truck load capacity Q_T
for v in V:
    model.addConstr(quicksum(dataset.data[i]['Demand'] * x[v, i, j] for i in N for j in N if i != j) <= Q_T, 
                    name=f'Payload_{v}')

# Constraint 7: Link y variable to x variable
#if any link in x (for each vehicle) is active -> y = 1
# can do this by checking if each truck leaves the depot (all trucks must leave depot if active)

for v in V:
    model.addConstr(y[v] == quicksum(x[v, 'D0', i] for i in N_customers), name=f'Link_y{v}_to_x_{v}')

# Constraint 8: Update time variable
# Loop over each vehicle
for vehicle in V:
    # Loop over each customer
    for customer in N_customers:
        # Initialize the sum for the current customer
        sum_for_current_customer = 0

        # Loop over each node
        for node in N:
            # Skip the current customer
            if node != customer:
                # Add the time at which the vehicle leaves the node plus the travel time from the node to the customer,
                # multiplied by the decision variable indicating whether the vehicle travels from the node to the customer,
                # to the sum for the current customer
                sum_for_current_customer += (t[vehicle, node] + time_dict[(node, customer)]) * x[vehicle, node, customer]

        # Add a constraint to the model that the time at which the vehicle arrives at the customer is equal to the sum for the current customer
        model.addConstr(t[vehicle, customer] == sum_for_current_customer, name=f'Update_time_{vehicle}_{customer}')



## SOLVE MODEL ##

# Update the model to integrate constraints
model.update()

# Write model to a file
model.write('TruckonlySimple.lp')

# Tune solver parameters
model.tune()

# Optimize the model
model.optimize()

# Print the results
if model.status == GRB.OPTIMAL:
    print('Optimal objective: %g' % model.objVal)
    for v in model.getVars():
        if v.x > 0:
            print('%s: %g' % (v.varName, v.x))
else:
    print('Optimization was stopped with status %d' % model.status)


## POST-PROCESSING ##

# Extract and store the solution
solution = {var.varName: var.x for var in model.getVars()}

# Print all routes for each vehicle
for vehicle in V:
    #print active vehicle (y)
    var_name_y = f'y[{vehicle}]'
    if var_name_y in solution and solution[var_name_y] >= 0.99:
        print(f'Vehicle {vehicle} is active')
    total_payload = 0
    for node_from in N:
        for node_to in N:
            if node_from != node_to:
                #print active links
                var_name_x = f'x[{vehicle},{node_from},{node_to}]'
                if var_name_x in solution and solution[var_name_x] >= 0.99:
                    print(f'{node_from} -> {node_to}')
                    total_payload += dataset.data[node_from]['Demand']
    print()
    print(f'Total payload delivered by vehicle {vehicle}: {total_payload}\n')


# Old post-processing

#exctract active vehicles
active_vehicles = [v for v in V if solution[f'y[{v}]'] >= 0.99]
#extract routes
active_routes = {}
for v in active_vehicles:
    active_routes[v] = []
    for node_from in N:
        for node_to in N:
            if node_from != node_to:
                if solution[f'x[{v},{node_from},{node_to}]'] >= 0.99:
                    active_routes[v].append(node_from)
print('active routes', active_routes)
                

#retrieve timestamps of customer visits
timestamps = {}
for v in active_vehicles:
    timestamps[v] = {}
    for node in active_routes[v]:  # Only consider nodes that the vehicle travels to
        timestamps[v][node] = solution[f't[{v},{node}]']

print('\n')
print('timestamps', timestamps)

#print all solution variables which have value of 1
dataset.plot_data(show_demand=False, scale_nodes=True, show_labels=True, active_routes=active_routes)

