# -*- coding: utf-8 -*-
"""
Created on Thu Sep 24 22:22:35 2020

@author: abombelli
"""

# Loading packages that are used in the code
import numpy as np
import os
import pandas as pd
import time
from gurobipy import Model,GRB,LinExpr
import pickle
from copy import deepcopy

# Get path to current folder
cwd = os.getcwd()

# Get all instances
full_list           = os.listdir(cwd)

# instance name
instance_name = 'data_example_2.xlsx'

# Load data for this instance
edges                 = pd.read_excel(os.path.join(cwd,instance_name),sheet_name='Airport data')

# Define dictionaries with subsets of nodes that precede/follow a specific node
# N_plus[i] is the dictionary containing nodes that are reachable FROM node i
# N_minus[i] is the dictionary containing nodes that lead TO node i
N_plus  = {}
N_minus = {}

# Set of all nodes
N = sorted(np.unique(list(edges['From'])+list(edges['To'])))

for n in N:
    N_plus[n]  = [row.To for index,row in edges.iterrows() if row.From == n]
    N_minus[n] = [row.From for index,row in edges.iterrows() if row.To == n]
    
#%%

startTimeSetUp = time.time()
model = Model()

#################
### VARIABLES ###
#################
x = {}
for i in range(0,len(edges)):
    x[edges['From'][i],edges['To'][i]]=model.addVar(lb=0, ub=1, vtype=GRB.BINARY,name="x[%s,%s]"%(edges['From'][i],edges['To'][i]))

            
model.update()

###################
### CONSTRAINTS ###
###################

source = 25
sink   = 28

for n in N:
    lhs = LinExpr()
    for n_plus in N_plus[n]:
        lhs += x[n,n_plus]
    for n_minus in N_minus[n]:
        lhs -= x[n_minus,n]   
    if n == source:
        rhs = 1
    elif n == sink:
        rhs = -1
    else:
        rhs = 0
    model.addConstr(lhs=lhs, sense=GRB.EQUAL, rhs=rhs,
                          name='node_'+str(n))


model.update()
 
       
obj        = LinExpr() 

for i in range(0,len(edges)):
    obj += edges['Distance'][i]*x[edges['From'][i],edges['To'][i]]


model.setObjective(obj,GRB.MINIMIZE)
model.update()
model.write('model_formulation.lp')    

model.optimize()
endTime   = time.time()

solution = []
     
for v in model.getVars():
     solution.append([v.varName,v.x])
     
route_complete = False
current_node   = source
path           = [source]
     
while route_complete is False:
    # Connections from current node
    idx_this_node_out = np.where(edges['From']==current_node)[0]
    #print(idx_this_node_out)
    for i in range(0,len(idx_this_node_out)):
        if x[current_node,edges['To'][idx_this_node_out[i]]].x >= 0.99:
            path.append(edges['To'][idx_this_node_out[i]])
            current_node = edges['To'][idx_this_node_out[i]]
            
            if current_node == sink:
                route_complete = True
                break
            else:
                break
            
print(path)
            
            
    
     


    
