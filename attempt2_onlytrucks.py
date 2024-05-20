#note to run this on the academic gurobi license just type in your terminal (before runnin script):
    #export GRB_LICENSE_FILE=/path/to/gurobi.lic
    #where /path/to/gurobi.lic is the path to the gurobi.lic file which you download from gurobi license managher website
    #ugos license path: /Users/ugomunzi/gurobi/licenses/gurobi.lic
        #export GRB_LICENSE_FILE=/Users/ugomunzi/gurobi/licenses/gurobi.lic
from gurobipy import Model,GRB,LinExpr,quicksum
import numpy as np
from scipy.spatial import distance
import os

from load_dataset import Dataset

"""
In this script, we will implement the model for the truck-only case.
    i.e. the classic vehicle routing problem where only trucks are used to deliver goods.

"""
#define gurboi license file location
os.environ['GRB_LICENSE_FILE'] = '/Users/ugomunzi/gurobi/licenses/gurobi.lic'
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

M = 500 #big M constant for big M method

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

num_trucks = 2
distance_dict = get_manhattan_distance(dataset.data)
time_dict = get_time_dict(dataset.data, S_T, distance_dict)

#definitions of N_0, N and N_plus follow from paper
N = list(dataset.data.keys()) #set of nodes with depot at start
N_customers = N.copy()
N_customers.remove('D0')
V = [f'V{i}' for i in range(1, num_trucks+1)] #set of trucks

## MODEL ##
model = Model("Truck-Only Model")

#decision variables
#define x such that you cannot travel between same node
x = model.addVars(V, [(i,j) for i in N for j in N if i != j], lb=0, ub=1, vtype=GRB.BINARY, name='x')
y = model.addVars(V, lb=0, ub=1, vtype=GRB.BINARY, name='y')
t = model.addVars(V, N, lb=0, vtype=GRB.CONTINUOUS, name='t')

model.update()

#constraints
# C1: each customer is visited exactly once
C1 = model.addConstrs((quicksum(x[v,i,j] for j in N if i != j for v in V) == 1 for i in N_customers), name='C1') 

# C2: each truck leaves the depot exactly once if it is active (y=1)
C2 = model.addConstrs((quicksum(x[v,'D0',j] for j in N if 'D0' != j) - y[v] == 0 for v in V), name='C2')

# C3: each truck arrives at depot once if it is active
C3 = model.addConstrs((quicksum(x[v,i,'D0'] for i in N if i != 'D0') - y[v] == 0 for v in V), name='C3')

# C4: if vehicle arrives at customer, must also leave
C4 = model.addConstrs((quicksum(x[v,i,h] for i in N if i != h) - quicksum(x[v,h,j] for j in N if h != j) == 0 for h in N_customers for v in V), name='C4')

# C5: time constraint (time at node equal or larger than time at previous node plus travel time)
C5 = model.addConstrs((t[v,j] >= t[v,i] + time_dict[i,j] - M*(1-x[v,i,j]) for i in N for j in N if i != j for v in V), name='C5')

# C6: payload for all visited customer nodes per vehicle less than limit Q_T
C6 = model.addConstrs((dataset.data[i]['Demand'] * quicksum(x[v,i,j] for j in N if i != j) <= Q_T for i in N_customers for v in V), name='C6')
# At least one truck must be active (otherwise optimal solution is to use no trucks)
#C7 = model.addConstr(quicksum(y[v] for v in V) >= 0.99, name='C7')
#C9 = model.addConstrs((x[v,i,i] == 0 for i in N for v in V), name='C9')#note: this constraint is essential (ensures you cant travek between same node) otherwise truck never leaves depot
#objective function (minimise cost both due to tranportation and basis cost of using truck (if active, i.e. y=1))
cost_obj = quicksum(C_T * distance_dict[i,j] * x[v,i,j] for i in N for j in N if i != j for v in V) + quicksum(C_B * y[v] for v in V)
model.setObjective(cost_obj, GRB.MINIMIZE)
model.update()
model.write('TruckonlySimple.lp')
#tune solver before optimizing to reduce time it takes
#model.tune()
model.optimize()

## POST-PROCESSING ##
solution = {}
for var in model.getVars():
    solution[var.varName] = var.x

#exctract active vehicles
active_vehicles = [v for v in V if solution[f'y[{v}]'] >= 0.99]
#extract routes
active_routes = {}
for v in active_vehicles:
    active_routes[v] = [i for i in N if solution[f'x[{v},{i},D0]'] >= 0.99] #start with depot
    while active_routes[v][-1] != 'D0':
        for j in N:
            if solution[f'x[{v},{active_routes[v][-1]},{j}]'] >= 0.99:
                active_routes[v].append(j)
                break

#retrieve timestamps of customer visits
timestamps = {}
for v in active_vehicles:
    timestamps[v] = {}
    for i in N_customers:
        timestamps[v][i] = solution[f't[{v},{i}]']

#print all solution variables which have value of 1
print([var for var in solution.keys() if solution[var] >=0.9])
print(active_routes)
#plot routes
dataset.plot_data(show_demand=True, scale_nodes=True, show_labels=False, active_routes=active_routes)




