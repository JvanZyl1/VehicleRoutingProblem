# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 13:26:18 2023

@author: abombelli
"""

import numpy as np
import pandas as pd
import os
import igraph as ig
from geopy.distance import great_circle
from scipy.spatial import distance
import random
from itertools import product
from gurobipy import Model,GRB,LinExpr,quicksum
from operator import itemgetter
import matplotlib.pyplot as plt

random.seed(42)
cwd         = os.getcwd()

plt.close('all')
axis_font  = {'fontname':'Arial', 'size':'15'}

#####################################
### Defining all initial settings ###
#####################################

# Size of square where we place all our elements
x_min, x_max = -10, 10
y_min, y_max = -10, 10
# Average speed (used to convert distances into times)
avg_speed    = 1
# lower, upper bound for earliest time a customer can be visited
te_min, te_max = 0, 100
# lower, upper bound for latest time a customer can be visited
tl_min, tl_max = 105, 300
# lower, upper bound for vehicle capacity
q_min, q_max = 10, 20
# lower, upper bound for vehicle range
r_min, r_max = 15, 30
# lower, upper bound for vehicle fixed cost
c_min, c_max = 100, 200
# lower, upper bound for customer demand
d_min, d_max = 3, 5


# Size of fleet
N_K = 4

# Number of customers to serve
N_C = 8

######################
### Defining depot ###
######################
D = {0:{'X':0,'Y':0}}

########################################
### Defining properties of customers ###
########################################

C = {k:{'X':np.round(x_min+random.random()*(x_max-x_min),1),
        'Y':np.round(y_min+random.random()*(y_max-y_min),1),
        'T_lb':random.randint(te_min,te_max),
        'T_ub':random.randint(tl_min,tl_max),
        'D':random.randint(d_min,d_max)} 
     for k in list(np.arange(1,N_C+1))}

#######################################
### Defining properties of vehicles ###
#######################################

K = {k:{'Q':random.randint(q_min,q_max),
        'R':random.randint(r_min,r_max),
        'C':random.randint(c_min,c_max)}
     for k in list(np.arange(1,N_K+1))}

##################################
### Defining full set of nodes ###
##################################
N = {**D, **C}

##############################################
### Defining distance matrix between nodes ###
##############################################
dist_matrix = np.zeros([len(N),len(N)])
for i in range(0,len(dist_matrix)-1):
    for j in range(i+1,len(dist_matrix)):
        dist_matrix[i][j] = np.round(distance.euclidean((N[i]['X'],N[i]['Y']),
                                               (N[j]['X'],N[j]['Y'])),1)

dist_matrix = dist_matrix+np.transpose(dist_matrix)

##########################################
### Defining time matrix between nodes ###
##########################################
time_matrix = dist_matrix/avg_speed

########################
### Defining subsets ###
########################

# Nodes from which we can reach node j using vehicle k 
Delta_plus_jk = {(j,k):[i for i in N.keys()
                        if i != j 
                        and dist_matrix[i][j]<=K[k]['R']] 
                 for j in N.keys() for k in K.keys()}

# Nodes we can reach from node j using vehicle k 
Delta_minus_jk = {(j,k):[i for i in N.keys()
                        if i != j 
                        and dist_matrix[j][i]<=K[k]['R']] 
                 for j in N.keys() for k in K.keys()}

##################     
### Plot nodes ###
##################
plt.close('all')
fig, ax = plt.subplots()
for d in D.keys():
    plt.plot(D[d]['X'],D[d]['Y'],marker='d',markersize=15,color='black')
plt.plot(D[0]['X'],D[0]['Y'],linestyle='',marker='d',markersize=15,color='black',label='Depot')
for c in C.keys():
    plt.plot(C[c]['X'],C[c]['Y'],marker='s',color='red')
    plt.text(C[c]['X'],C[c]['Y'],'%s'%(c),**axis_font)
plt.plot(C[1]['X'],C[1]['Y'],linestyle='',marker='s',color='red',label='Customers')


ax.set_xlim(x_min,x_max)
ax.set_ylim(y_min,y_max)
ax.set_xlabel('X',**axis_font)
ax.set_ylabel('Y',**axis_font)
ax.grid(True)
plt.legend(loc='best',framealpha=0.5)
plt.show()

#%%

###################################
### Defining optimization model ###
###################################
VRP_model = Model()

##########################
### Decision variables ###
##########################
x = {} # routing decision variables
y = {} # activation of a vehicle
t = {} # time decision variables

# Note; with Gurobi you can define all combinations of indices as a list 
# of tuples beforehand (see here the answer by Ioannis:
# https://stackoverflow.com/questions/52451928/efficient-way-to-add-
# variables-and-constraints-through-gurobi-python-without-enu)
# so that you can define a whole set of decision variables, e.g.,
# x, as a one-liner. I like for cycles a bit better here, although it is a 
# bit less efficient

for k in K.keys():
    for i in N.keys():
        for j in Delta_minus_jk[(i,k)]:  
            x[i,j,k] = VRP_model.addVar(lb=0, ub=1, 
                                    vtype=GRB.BINARY, 
                                    name='x[%s,%s,%s]'%(i,j,k))
for k in K.keys():
    y[k] = VRP_model.addVar(lb=0, ub=1, 
                                    vtype=GRB.BINARY, 
                                    name='y[%s]'%(k))

for c in C.keys():
    t[c] = VRP_model.addVar(lb=C[c]['T_lb'], ub=C[c]['T_ub'], 
                                    vtype=GRB.CONTINUOUS, 
                                    name='t[%s]'%(c))

VRP_model.update()

###################
### Constraints ###
###################

# Every customer must be visited exactly once: this is constraint (2)
# in slide 8 of the presentation
C1 = VRP_model.addConstrs(((quicksum(x[i,j,k] for k in K.keys()
                                    for j in Delta_minus_jk[(i,k)]) == 1)
                                    for i in C.keys()),name='C1')

# Conservation of flow per customer node and vehicle: this is constraint (5)
# in slide 10 of the presentation
C4 = VRP_model.addConstrs(((quicksum(x[j,c,k]
                                    for j in Delta_plus_jk[(c,k)]) -
                            quicksum(x[c,j,k]
                                    for j in Delta_minus_jk[(c,k)])== 0)
                                    for c in C.keys() for k in K.keys()),
                          name='C4')

# Time precedence constraints between customer nodes (considering customers
# as origins and customers but not the depot as destinations as t[0] is
# not defined: this is constraint (6)
# in slide 11 of the presentation
# In addition: I am using fixed M, which is not efficient. You can, for 
# every (i,j) combination, determine the smallest M you need
M = 500
C5 = VRP_model.addConstrs(((t[j]- t[i] -M*x[i,j,k] >= time_matrix[i][j]-M)
                          for k in K.keys() 
                          for i in C.keys() 
                          for j in Delta_minus_jk[(i,k)] if j!=0),
                          name='C5')

# Maximum load per vehicle: this is constraint (10)
# in slide 13 of the presentation
C8 = VRP_model.addConstrs(((C[i]['D']*quicksum(x[i,j,k]
                            for i in C.keys() for j in Delta_minus_jk[(i,k)]) 
                            <= K[k]['Q'])
                            for k in K.keys()),name='C8')

##########################
### Defining objective ###
##########################

# Note: you can also define all the coefficients of every decision variable
# when defining the decision variables. In that case, Gurobi already knows
# the objective basically. I personally like this approach better because
# I can define the objective function following its actual structure. In this
# case, I am replicating the objective function (1) in slide 7 of the
# presentation 
obj = LinExpr()

# First term: transportation cost due to traveled distance
C_ij = 5 # monetary units/unit distance
for k in K.keys():
    for i in N.keys():
        for j in Delta_minus_jk[(i,k)]:
            obj += C_ij*dist_matrix[i][j]*x[i,j,k]

# Second term: fixed charge cost due to activation of vehicles
for k in K.keys():
    obj += K[k]['C']*y[k]

# Define if it is a maximization or minimization problem (IMPORTANT!),
# update model, write .lp file (not mandatory, yet recommended)
#
# Note: running this code will probably yield a solution, yet non
# practically feasible as many constraints were omitted            
VRP_model.setObjective(obj,GRB.MINIMIZE)
VRP_model.update()
VRP_model.write('VRP_example.lp') 
VRP_model.optimize()

#######################
### Post-processing ###
#######################

# Note: I have my own may to carry out post-processing, i.e.,
# retrieve decision variables from the model solution and process them.
# It is not necessarily the best approach. Please find your own way :-)

# Retrieve variable names and values
# v.varName is the name as given in the decision variable definition
# v.x is the value
solution = []
for v in VRP_model.getVars():
    solution.append([v.varName,v.x])
    
# IMPORTANT: when looking for binary variables, write >= 0.99 (or any
# number reasonably close to 1) and not == 1. The reason for that is the
# decision variables are still solution of a numerically solved linear
# system, hence a binary decision variable that is 1 can be outputted
# as 0.999999 from the optimizer
    
# Retrieve vehicle activation decision variables
active_y_variables = [(sol[0],sol[1]) for sol in solution if sol[1]>=0.99
                      and sol[0][0]=='y']
K_used             = []
for act_y in active_y_variables:
    idx_brckt_opn  = [pos for pos,char in enumerate(act_y[0]) if char=='['][0]
    idx_brckt_clsd = [pos for pos,char in enumerate(act_y[0]) if char==']'][0]
    K_used.append(int(act_y[0][idx_brckt_opn+1:idx_brckt_clsd]))

# Retrieve routing decision variables
active_x_variables = [(sol[0],sol[1]) for sol in solution if sol[1]>=0.99
                      and sol[0][0]=='x']
# Split routing decision variables per used truck
active_x_variables_K_used = {}
for k in K_used:
    temp = []
    for act_x in active_x_variables:
        # Note: we are interested in the second comma
        idx_comma      = [pos for pos,char in enumerate(act_x[0]) if char==','][1]
        idx_brckt_clsd = [pos for pos,char in enumerate(act_x[0]) if char==']'][0]
        if int(act_x[0][idx_comma+1:idx_brckt_clsd])==k:
            temp.append(act_x)
    active_x_variables_K_used[k] = temp


# Retrieve time-stamps of customer visits
active_t_variables = [(sol[0],sol[1]) for sol in solution if sol[0][0]=='t']

