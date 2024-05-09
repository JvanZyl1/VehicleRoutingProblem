from gurobipy import Model,GRB,LinExpr,quicksum
import numpy as np
from scipy.spatial import distance
import os

from load_dataset import Dataset

## FUNCTIONS ##
def get_num_trucks(data):
    """
    Returns the number of trucks needed to serve the customers
    """
    n = len(data)
    total_demand = sum([data[i]['Demand'] for i in range(n)])
    return int(np.ceil(total_demand/Q_T))

def get_manhattan_distance(data):
    """
    Returns a dictionary with manhattan distances between all nodes in dataset
    """
    distance_dict = {}
    for node1 in data.keys():
        for node2 in data.keys():
            distance_dict[node1, node2] = distance.cityblock([data[node1]['X'], data[node1]['Y']], [data[node2]['X'], data[node2]['Y']])
    return distance_dict

def get_euclidean_distance(data):
    """
    Returns a dictionary with euclidean distances between all nodes in dataset
    """
    distance_dict = {}
    for node1 in data.keys():
        for node2 in data.keys():
            distance_dict[node1, node2] = distance.euclidean([data[node1]['X'], data[node1]['Y']], [data[node2]['X'], data[node2]['Y']])
    return distance_dict


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

M = 100000 #big M constant for big M method

## LOAD DATASET ##
current_dir = os.getcwd()
data_subfolder = '0.3'
data_num_nodes = '40'
data_area = '20'

data_file_name = f'{data_num_nodes}_{data_area}_{data_subfolder}'
dataset_path = f'dataset/{data_subfolder}/{data_file_name}.txt'
output_file_path = os.path.join(current_dir, data_file_name + '_solution.sol')#used to save solution file

dataset = Dataset(dataset_path)
#dataset.plot_data(show_demand=True, scale_nodes=True, show_labels=False)

#num_trucks = get_num_trucks(dataset.data)
num_trucks = 2
num_drones = num_trucks

#definitions of N_0, N and N_plus follow from paper
N_0 = list(dataset.data.keys()) #set of nodes with depot at start
N = N_0.copy() #N is set of customer nodes
N.remove('D0') #remove depot from start
N_plus = N.copy()
N_plus.append('D0') #add depot to end
V = [f'V{i}' for i in range(1, num_trucks+1)] #set of trucks/drones (can use the same index for both)

#find set of nodes which can be reached by drones
N_drones = [i for i in N if dataset.data[i]['ServiceBy'] == 'D/T']

#get distance matrices
manhattan_distance = get_manhattan_distance(dataset.data)
euclidean_distance = get_euclidean_distance(dataset.data)

## MODEL ##
#note:for now dynamic scaling of endurance in ignored
m = Model('DroneTruckRouting')
m.setParam('OutputFlag', 1) #ensures that gurobi output is printed to console during optimisation
#decision variables (as defined in paper)
#note: indexing order as follows [v:truck/drone, i:start node, j:end node, optional k:retrieval node]
x = m.addVars(V, N_0, N_plus, vtype=GRB.BINARY, name='x') #can leave any node including depot (i in N_0) but can only deliver to customers or go to depot when finished delivering (j in N_plus)
y = m.addVars(V, N_0, N_drones, N_plus, vtype=GRB.BINARY, name='y') #can be launched and retrieved at all nodes including depot (i in N_0, k in N_plus) but can only visit nodes accesible by drones (j in N_drones)
t_truck = m.addVars(V, N_plus, vtype=GRB.CONTINUOUS, name='t_truck') #note:use N_plus as initial depot node doesnt have arrival time, only has arrival time when arriving at depot after having completed deliveries
t_drone = m.addVars(V, N_plus, vtype=GRB.CONTINUOUS, name='t_drone') #this accounts both for arrival time at customer nodes and retrieval time at any node
W_truck = m.addVars(V, N_0, N_plus, vtype=GRB.CONTINUOUS, name='W_truck') #weight of truck going from i to j (i in N_0, j in N_plus)
p_truck = m.addVars(V, N_0, N_plus, vtype=GRB.CONTINUOUS, name='p_truck') #payload of truck going from i to j (i in N_0, j in N_plus)
u_truck = m.addVars(V, N_plus, vtype=GRB.INTEGER, name='u_truck') #order of truck at node i (i in N_plus), include depot node as last node (N_plus) as starting point isnt 'visited' by truck
t_max = m.addVar(name='t_max') #variable used for finding truck which takes longest time to complete deliveries

"""
below code achieves same as above but with more explicit variable definitions (like bombelli does it in his code)
for v in V:
    for i in N_0:
        arrival_time_truck[i, v] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name='arrival_time_drone[%s,%s]'%(i,v))
        order_truck[i, v] = m.addVar(lb=0, vtype=GRB.INTEGER, name='order_truck[%s,%s]'%(i,v))
        for j in N:
            x[i, j, v] = m.addVar(lb=0, ub=1, vtype=GRB.BINARY, name='x[%s,%s,%s]'%(i,j,v))
            W_truck[i, j, v] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name='W_truck[%s,%s,%s]'%(i,j,v))
            payload_truck[i, j, v] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name='payload_truck[%s,%s,%s]'%(i,j,v))

#define drone variables (note: y has four indices [launch node, customer node, retrieval node, drone number], 
#drone can be launched and retrieved at all nodes (hence, iterate over N_plus) but can only visit (index j) customers in N_drones)
for v in V:
    for i in N_0:
        for j in N_drones:
            arrival_time_drone[j, v] = m.addVar(lb=0, vtype=GRB.CONTINUOUS, name='arrival_time_drone[%s,%s]'%(j,v))
            for k in N_plus:
                y[i, j, k, v] = m.addVar(lb=0, ub=1, vtype=GRB.BINARY, name='y[%s,%s,%s,%s]'%(i, j, k, v))
"""
m.update()

## CONSTRAINTS ##

# * Constraint 8: Ensures each customer is served by either a truck or a drone.
#C8 = m.addConstrs((quicksum(x[v,i,j] for v in V for i in N_0) + quicksum(y[v,i,j,k] for v in V for i in N_0 for k in N_plus) == 1 for j in N), name='C8') #this wont wont work for summing the srones (y var) as not all nodes are reachable by drones
C8 = m.addConstrs((quicksum(x[v,i,j] for v in V for i in N_0) + quicksum(y[v,i,j,k] if j in N_drones else 0 for v in V for i in N_0 for k in N_plus) == 1 for j in N), name='C8') #this should work as it only sums the drones that can reach the node
# * Constraint 9: Ensures each truck leaves the depot at most once
C9 = m.addConstrs((quicksum(x[v,'D0',j] for j in N_plus) == 1 for v in V), name='C9')
# * Constraint 10: Ensures that each truck arrives at the depot at most once
C10 = m.addConstrs((quicksum(x[v,i,'D0'] for i in N) == 1 for v in V), name='C10') 
# * Constraint 11: Ensures trucks dont travel directly from depot to depot
C11 = m.addConstrs((x[v, 'D0', 'D0'] == 0 for v in V), name='C11')
# * Constraint 12: Ensures each drone is launched at most once at all customer and depot nodes (GPT: "prevents multiple drone operations from a single node for each vehicle, making sure that a drone is involved in only one delivery mission at a time before it has to be retrieved.").
C12 = m.addConstrs((quicksum(y[v,i,j,k] if j != i and k!=j else 0 for j in N_drones for k in N_plus) <= 1 for v in V for i in N_0), name='C12')
# * Constraint 13: Ensures each drone is retrieved at most once at all customer and depot nodes.
C13 = m.addConstrs((quicksum(y[v,i,j,k] if i!=j and j!=k else 0 for i in N_0 for j in N_drones) <= 1 for v in V for k in N_plus), name='C13')
# * Constraint 14: Esnures drones are not loaded beyond its load capacity during flight.
C14 = m.addConstrs((quicksum(y[v,i,j,k]*dataset.data[j]['Demand'] if i!=j and k!=i else 0 for i in N_0 for k in N_plus) <= Q_D for j in N_drones for v in V), name='C14')
# * Constraint 15: Ensures each truck does not exceed its load capacity during entire delivery route.
#C15 = m.addConstrs(((quicksum(quicksum(x[v,i,j]*dataset.data[j]['Demand'] for i in N_0) + quicksum(y[v,i,j,k]*dataset.data[j]['Demand'] if j in N_drones else 0 for i in N_0 for k in N_plus)) for j in N) <= Q_T for v in V), name='C15')
C15 = m.addConstrs((quicksum(quicksum(x[v,i,j]*dataset.data[j]['Demand'] for i in N_0) + quicksum(y[v,i,j,k]*dataset.data[j]['Demand'] if j in N_drones else 0 for i in N_0 for k in N_plus) for j in N) <= Q_T for v in V), name='C15')
# * Constraint 16: Ensures that if drone is launched at node i and retrieved at node k, the truck must also pass through both nodes to launch/retrieve the drone.
C16 = m.addConstrs((quicksum(y[v, i, j, k] for v in V for j in N_drones) <= quicksum(x[v, h, i] for v in V for h in N_0 if h != i) + quicksum(x[v, l, k] for v in V for l in N_plus if l != k) for i in N for k in N_plus if i != k), name='C16')
# * Constraint 17: Ensures delivery sequence of trucks is consistent with that of the drones (GPT: "This constraint ensures that if a drone is deployed for a mission from node i to j and retrieved at node k, the truck must visit node i before node k. Essentially, it ties the truck's routing to the drone's operations, ensuring that the sequence of visits is logically consistent with the drone's deployment and retrieval.").
C17 = m.addConstrs((u_truck[v,k] - u_truck[v,i] >= 1 - M * (1 - quicksum(y[v,i,j,k] for j in N_drones)) for i in N for k in N_plus for v in V if k!=i), name='C17')
# * Constraint 18: Launch time of drone at node i cannot be earlier than arrival time of the truck at same node **unless** drone is not launched at node i (big M constant negates this constraint in this case)
C18 = m.addConstrs((t_drone[v,i] >= t_truck[v,i] - M * (1 - quicksum(y[v,i,j,k] for j in N_drones for k in N_plus if j!=i if k!=j)) for i in N for v in V), name='C18')
# * Constraint 19: Launch time of drone at node i cannot be later than arrival time of the truck at same node **unless** drone is not launched at node i (big M constant negates this constraint in this case)
C19 = m.addConstrs((t_drone[v,i] <= t_truck[v,i] + M * (1 - quicksum(y[v,i,j,k] for j in N_drones for k in N_plus if j!=i if k!=j)) for i in N for v in V), name='C19')
# * Constraint 20: Ensures drone retrieval time at node k is not earlier than truck's arrival at that node.
C20 = m.addConstrs((t_drone[v,k] >= t_truck[v,k] - M * (1 - quicksum(y[v,i,j,k] for i in N_0 for j in N_drones if i!=j)) for k in N_plus for v in V), name='C20')
# * Constraint 21: Ensures drone retrieval time at node k is not later than truck's arrival at that node.
C21 = m.addConstrs((t_drone[v,k] <= t_truck[v,k] + M * (1 - quicksum(y[v,i,j,k] for i in N_0 for j in N_drones if i!=j)) for k in N_plus for v in V), name='C21')
# * Constraint 22: Ensures arrival time of truck at node j is after departure time from node i based on manhattan distance d<sub>ij</sub><sup>M</sup>. Big M deactivates constraint if there is no direct link between the two nodes.
C22 = m.addConstrs((t_truck[v,j] >= t_truck[v,i] + manhattan_distance[i,j]/S_T - M * (1 - x[v,i,j]) for i in N for j in N for v in V if j!=i), name='C22')
# * Constraint 23: Ensures arrival time of drone at node j is after departure (launch) time from node i based on euclidean distance d<sub>ij</sub><sup>E</sup>. Big M deactivates constraint if drone doesnt make direct trip between the two nodes.
C23 = m.addConstrs((t_drone[v,j] >= t_drone[v,i] + euclidean_distance[i,j]/S_D - M * (1 - quicksum(y[v,i,j,k] for k in N_plus)) for i in N for j in N_drones for v in V if j!=i), name='C23')
# * Constraint 24: Ensures that the time of retrieval at node k occurs after the time of delivery of the drone at node j based on euclidean distance d<sub>ij</sub><sup>E</sup>. Big M deactivates constraint if drone doesnt make direct trip between the two nodes. 
C24 = m.addConstrs((t_drone[v,k] >= t_drone[v,j] + euclidean_distance[j,k]/S_D - M * (1 - quicksum(y[v,i,j,k] for i in N_0)) for j in N_drones for k in N_plus for v in V if k!=j), name='C24')
# * Constraint 25: Ensures total flight time of drone is less than its maximum endurance. Big M deactivates constraint if drone doesnt make direct trip between the two nodes. 
C25 = m.addConstrs((t_drone[v,k] - t_drone[v,i] <= E + M * (1 - quicksum(y[v,i,j,k] for j in N_drones)) for k in N_plus for i in N for v in V if i!=k), name='C25')

##additional constraints not defined in paper to calculate W_truck and t_max
#additional constraint not defined in paper to calculate W[v,i,j] for trucks
C27 = m.addConstrs((W_truck[v,i,j] == x[v,i,j] * (W_T + p_truck[v,i,j] + W_D * (1 - quicksum(y[v,i,m,j] for m in N_drones))) for v in V for i in N_0 for j in N_plus), name='W_truck_constraint')
#t_max constraint
C28 = m.addConstrs((t_max >= t_truck[v, 'D0'] for v in V), name='max_time_constraints')
## OBJECTIVE FUNCTION ##
# 3 objectives: minimise f1: the environemntal objective, f2 the total cost of delivery and f3 the total delivery time
#minimise enery consumption (f1) (note: only trucks used here, assumed drones have minimal energy consumption)
f1 = quicksum(manhattan_distance[i,j] * W_truck[v,i,j] * x[v,i,j] for v in V for i in N_0 for j in N_plus)
#minimise total cost of delivery (f2)
f2 = quicksum(quicksum(C_T * manhattan_distance[i,j] * x[v,i,j] for i in N_0 for j in N_plus) + quicksum(C_D * (euclidean_distance[i,j] + euclidean_distance[j,k]) * y[v,i,j,k] for i in N_0 for j in N_drones for k in N_plus) for v in V) + C_B * quicksum(x[v,'D0',j] for j in N_plus for v in V)
#minimise total delivery time (f3)
#f3 = max(t_truck[v, 'D0'] for v in V)#note this way doesnt work, have to add a decision variable t_max which is found through a constraint
f3 = t_max
m.setObjective(f1 + f2 + f3, GRB.MINIMIZE)
m.update()
m.write(data_file_name + '_model.mps') #write model to file
m.optimize()
m.write(output_file_path) #write solution to file

# Get the maximum delivery time
max_delivery_time = t_max.X

print(f'Maximum delivery time: {max_delivery_time}')







