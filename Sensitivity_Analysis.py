from gurobipy import Model,GRB,LinExpr,quicksum, GurobiError
import numpy as np
from scipy.spatial import distance
import os
import socket
from load_dataset import Dataset


## MODEL PARAMETERS ##
W_T = 1500 #empty weight truck [kg]
Q_T = 1000 #load capacity of trucks [kg]
C_T = 25 #travel cost of trucks per unit distance [monetary unit/km]
C_B = 500 #basis cost of using a truck equipped with a drone [monetary unit]
S_T = 60 #average travel speed of the trucks [km/h]


M = 1e9 #big M constant for big M method(1e9 is largest order of magnitude before numerical issues arise in Gurobi)

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

def get_cost_obj(C_T, truck_distance_dict, active_routes_truck, cost_weight=1):
    """
    Returns the cost objective value for the model after it has optimised
    """
    cost_obj = 0
    for truck in active_routes_truck.keys():
        for route in active_routes_truck[truck]:
            i, j, _, _ = route
            cost_obj += cost_weight * C_T * truck_distance_dict[i, j]
    return cost_obj

def get_environmental_obj(W_T, truck_distance_dict, active_routes_truck, environmental_weight=1):
    """
    Returns the environmental objective value for the model after it has optimised
    """
    environmental_obj = 0
    for truck in active_routes_truck.keys():
        for route in active_routes_truck[truck]:
            i, j, _, _ = route
            environmental_obj += environmental_weight * W_T * truck_distance_dict[i, j]
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


num_trucks = 5 # reduce from 10 to five to speed up optimisisation (basically half the number of variables)
truck_distance_dict = get_manhattan_distance(dataset.data)
truck_time_dict = get_time_dict(dataset.data, S_T, truck_distance_dict)

#definitions of N_0, N and N_plus follow from paper
N = list(dataset.data.keys()) #set of nodes with depot at start (D0) and at end (D1)
N_customers = N.copy()
N_customers.remove('D0')
N_customers.remove('D1')
Tr = [f'Tr{i}' for i in range(1, num_trucks+1)] #set of trucks


# Create a new model
model = Model("Truck_Routing")

# Define decision variables
x = model.addVars(Tr, [(i, j) for i in N for j in N if check_in_x_var(i, j)], lb=0, ub=1, vtype=GRB.BINARY, name='x')
y = model.addVars(Tr, lb=0, ub=1, vtype=GRB.BINARY, name='y')
t = model.addVars(Tr, N, lb=0, vtype=GRB.CONTINUOUS, name='t')
t_max = model.addVar(lb=0, vtype=GRB.CONTINUOUS, name='t_max')
payload = model.addVars(Tr, [(i, j) for i in N for j in N if check_in_x_var(i, j)], lb=0, ub=Q_T, vtype=GRB.CONTINUOUS, name='payload')
W_dynamic = model.addVars(Tr, [(i, j) for i in N for j in N if check_in_x_var(i, j)], lb=0, vtype=GRB.CONTINUOUS, name='W_dynamic')


# Objective 1: Cost both due to transportation and base cost of using truck if active)
cost_obj = quicksum(C_T * truck_distance_dict[i,j] * x[truck,i,j] for i in N for j in N if check_in_x_var(i, j) for truck in Tr) + \
           quicksum(C_B * y[truck] for truck in Tr)
time_obj = t_max
environmental_obj = quicksum(truck_distance_dict[i,j] * W_dynamic[truck, i, j] * x[truck,i,j] for i in N for j in N if check_in_x_var(i, j) for truck in Tr)

cost_weight = 1
time_weight = 10000
environmental_weight = 0.01


obj = cost_weight * cost_obj + time_weight * time_obj + environmental_weight * environmental_obj
model.setObjective(obj, GRB.MINIMIZE)

model.update()


# Constraint 1: Each customer is visited by exactly one truck
for customer in N_customers:
    # Initialize the sum for the current customer
    sum_for_current_customer = 0

    # Loop over each truck
    for truck in Tr:
        # Loop over each node
        for node in N:
            # Skip if customer is equal to node
            if check_in_x_var(node, customer):
                # Add the variable to the sum
                sum_for_current_customer += x[truck, node, customer]

    # The sum for the current customer must be equal to 1
    model.addConstr(sum_for_current_customer == 1, name=f'C1_Customer_{customer}_visited_once')

# Constraint 2: Each truck must leave the depot if active
for truck in Tr:
    sum_for_current_vehicle = quicksum(x[truck, 'D0', customer] for customer in N_customers)
    model.addConstr(sum_for_current_vehicle == y[truck], name=f'C2_Truck_leaves_depot_{truck}')

# Constraint 3: Each truck must return to the depot if active
for truck in Tr:
    sum_for_current_vehicle = quicksum(x[truck, customer, 'D1'] for customer in N_customers)
    model.addConstr(sum_for_current_vehicle == y[truck], name=f'C3_Truck_returns_to_depot_{truck}')

# Constraint 4: If a truck arrives at a customer node it must also leave (flow balance)
for truck in Tr:
    for node in N_customers:
        model.addConstr(
            quicksum(x[truck, node, j] for j in N if check_in_x_var(node, j)) == 
            quicksum(x[truck, j, node] for j in N if check_in_x_var(j, node)),
            name=f'C4_Flow_balance_{truck}_{node}'
        )


#Constraint 5: Time at a node is equal or larger than time at previous nodes plus travel time (or irrelevant). Eliminates need for subtour constraints.
# Add time constraints for all vehicles, nodes, and customers
for truck in Tr:
    for node in N:
        for customer in N:
            if check_in_x_var(node, customer):
                model.addConstr(
                    t[truck, customer] >= t[truck, node] + truck_time_dict[(node, customer)] - M * (1 - x[truck, node, customer]),
                    name=f'C5_Time_{truck}_{node}_{customer}'
                )

# Constraint 6: The total payload delivered to the customer must be less or equal to the truck load capacity Q_T
for truck in Tr:
    model.addConstr(quicksum(dataset.data[i]['Demand'] * x[truck, i, j] for i in N for j in N if check_in_x_var(i, j)) <= Q_T, 
                    name=f'C6_Payload_{truck}')

# Constraint 7: Link y variable to x variable : TRUCKS
#if any link in x (for each truck) is active -> y = 1
# can do this by checking if each truck leaves the depot (all trucks must leave depot if active)

for truck in Tr:
    model.addConstr(y[truck] == quicksum(x[truck, 'D0', i] for i in N_customers), name=f'C7_Link_y{truck}_to_x_{truck}')

# Constraint 8: set departure time from depot D0 for each truck = 0
for truck in Tr:
    model.addConstr(t[truck, 'D0'] == 0, name=f'C8_Departure_time_{truck}_D0')

#### CONSTRAINTS WHICH DON'T WORK #### : TIME CONSTRAINTS ARE NOT WORKING
'''
# Constraint 9: Update time variable for trucks
# Loop over each truck
for truck in Tr:
    # Loop over each node (destination)
    for j in N:
        if j != 'D0':  # Ensuring no calculation is made for the time to 'D0' as it's the starting point only
            # Initialize the sum for arriving at node j from any node i
            sum_for_arrival_to_j = quicksum((t[truck, i] + truck_time_dict[i, j]) * x[truck, i, j]
                                            for i in N if check_in_x_var(i, j))
            
            # Add the constraint that sets the arrival time at j based on departure times from all nodes i
            model.addConstr(t[truck, j] == sum_for_arrival_to_j, name=f'C9_Update_time_{truck}_{j}')

# Constraint 10: Update max delivery time variable
for truck in Tr:
    for node in N:
        # Add a constraint to the model that the maximum delivery time is greater than or equal to the delivery time to the customer for each vehicle
        model.addConstr(t_max >= t[truck, node], name=f'Update_max_delivery_time_{truck}_{node}')
'''
#### CONSTRAINT WHICH DON'T WORK END ####

#### WEIGHT CONSTRAINTS ####
'''
# Constraint 11: set payload for each truck at depot = sum of demand of all customers visited by truck
#note: this works, checked manually
for truck in Tr:
    sum_for_current_truck = 0
    for i in N:
        for j in N:
            if check_in_x_var(i, j):
                sum_for_current_truck += dataset.data[j]['Demand'] * x[truck, i, j]
        if check_in_x_var('D0', i):
            model.addConstr(payload[truck, 'D0', i] == sum_for_current_truck * x[truck, 'D0', i], name=f'Payload_{truck}_D0_{i}')


#Constraint 12: each time truck delivers to a customer, the payload is updated
for truck in Tr:
    for i in N:
        for j in N_customers:
            for k in N:
                """
                i is node which was previously visited by truck
                j is customer node where truck is delivering
                k is node where truck is going next
                """
                if check_in_x_var(i, j) and check_in_x_var(j, k):
                    model.addConstr(payload[truck, j, k] <= payload[truck, i, j] - dataset.data[j]['Demand'] * x[truck, i, j] + M * (1 - x[truck, i, j]), name=f'Payload_update_{truck}_{truck}_{i}_{j}_{k}')


# Constraint 13: Explicitly set payload to zero for inactive routes (for some reason without this some inactive trucks have payload = 1000)
    for truck in Tr:
        for i in N:
            for j in N:
                if check_in_x_var(i, j):
                    model.addConstr(payload[truck, i, j] <= M * x[truck, i, j], name=f'Zero_inactive_payload_{truck}_{i}_{j}')

# Constraint 14: update dynamic weight for each truck at each node (W_dynamic[truck, i, j] = W_T + payload[truck, i, j]) only when link (thus truck) is active
for truck in Tr:
    for i in N:
        for j in N:
            if check_in_x_var(i, j):
                model.addConstr(W_dynamic[truck, i, j] == (W_T + payload[truck, i, j]) * x[truck, i, j], name=f'Update_dynamic_weight_{truck}_{i}_{j}')

# Create your model here
model.update()

try:
    # Assume `model` has been defined and all variables/constraints set up as shown previously.
    
    # Optimize the model
    for v in model.getVars():
        v.vType = GRB.CONTINUOUS
    model.optimize()

    # Check if the model was solved to optimality
    if model.status == GRB.OPTIMAL:
        print('Model solved to optimality.')
        print('Shadow prices:')
        for c in model.getConstrs():
            print(f'{c.ConstrName}: {c.Pi}')

        # Save the shadow prices to a file

        with open('sensitivity_analysis.txt', 'w') as f:
            for c in model.getConstrs():
                f.write(f'{c.ConstrName}: {c.Pi}\n')

        
    else:
        print(f"Model not solved to optimality. Current model status: {model.status}")

except GurobiError as e:
    print('Error reported by Gurobi:', str(e))

except AttributeError as ae:
    print("AttributeError encountered:", str(ae))
    print("Make sure the model is optimized and the required attributes are available.")