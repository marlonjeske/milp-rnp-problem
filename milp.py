import pyomo.environ as pyo
from collections import defaultdict

#Read data file
file_name = "U1.txt"
output_file = "U1_log.txt"
open(output_file, 'w').close() #Clean output file

cost_relays = {}
edges = set()
rssi = {}
capacity = {}
energy_tx = {}
energy_rx = {}
initial_energy = {}

with open(file_name, "r") as file:
    for line in file:
        element = line.strip().split()
        letter = element[0]
        
        if letter ==  "sensors":
            num_sensors = int(element[1])
        elif letter == "relays":
            num_relays = int(element[1])
        elif letter == "cost":
            relay_index = int(element[1]) 
            relay_cost = int(element[2])
            cost_relays[relay_index] = relay_cost
        elif letter == "e":
            ffrom = int(element[1])
            tto = int(element[2])
            edge_rssi = float(element[3])
            edge_capacity = int(element[4])
            edge_energy_tx = float(element[5])
            edge_energy_rx = float(element[6])
            edge = tuple([ffrom,tto])
            edges.add(edge)
            rssi[edge] = edge_rssi
            capacity[edge] = edge_capacity
            cap = edge_capacity
            energy_tx[edge] = edge_energy_tx
            energy_rx[edge] = edge_energy_rx
        elif letter == "initial_energy":
            initial_energy_index = int(element[1])
            initial_energy_value = float(element[2]) 
            initial_energy[initial_energy_index] = initial_energy_value
        elif letter == "packet_size":
            packet_size = int(element[1])
        elif letter == "num_disjoints":
            num_disjoints = int(element[1])
        


#Create the model
model = pyo.ConcreteModel()


#Define the number of sinks
num_sink = 1
total_nodes = num_sensors + num_relays + num_sink


#Index of the sink
model.sink_index = 1
sink_index = model.sink_index



# Sets of sensors, relays and all nodes and all edges
model.S = pyo.RangeSet(2, (num_sensors+1))  # Set of only Sensors (without sink)
S = model.S

model.R = pyo.RangeSet((num_sensors+2), (total_nodes))  # Set of relays
R = model.R

model.N = pyo.RangeSet(1, total_nodes) # Set of all nodes, S union R union sink
N = model.N

model.edges = pyo.Set(dimen = 2, initialize = edges) # Set of all edges (NxN)
E = model.edges

model.P = pyo.RangeSet(1,num_disjoints) # Set of edge-disjoint paths
P = model.P



#Incoming and Outcoming edges
Vm = defaultdict(set)
Vp = defaultdict(set)

for (i,j) in edges:
	Vp[i].add(j) # Add all j that receive edge from each i
	Vm[j].add(i)


#Parameters
model.Vm = pyo.Param(N, initialize = Vm, default = set(), within = pyo.Any)

model.Vp = pyo.Param(N, initialize = Vp, default = set(), within = pyo.Any)

model.cost = pyo.Param(R, initialize = cost_relays)
cost = model.cost

model.rssi = pyo.Param(edges, initialize = rssi)
rssi = model.rssi

model.capacity = pyo.Param(edges, initialize = capacity)
capacity = model.capacity

model.energy_tx = pyo.Param(edges, initialize = energy_tx)
energy_tx = model.energy_tx

model.energy_rx = pyo.Param(edges, initialize = energy_rx)
energy_rx = model.energy_rx

model.initial_energy = pyo.Param(N, initialize = initial_energy)
initial_energy = model.initial_energy

model.packet_size = packet_size
b = model.packet_size

model.M = cap + 1 #M must be greater than the edge capacity
M = model.M


#Decision variables
model.f = pyo.Var(S, P, E, 
                  domain = pyo.NonNegativeReals,
                  bounds = (0, cap)
                  #bounds = lambda model, s, p, i, j: (0, capacity[i,j])
                  )
f = model.f

model.y = pyo.Var(R, domain = pyo.Binary)
y = model.y

model.z = pyo.Var(E, domain = pyo.Binary)
z = model.z

model.w = pyo.Var(S, P, E, domain = pyo.Binary)
w = model.w

model.e = pyo.Var(N, 
                  domain = pyo.NonNegativeReals,
                  initialize = initial_energy,
                  bounds = lambda model, i: (0, initial_energy[i])
                  )
e = model.e


#Objective function
def objective_rule(model):
    installation_costs = sum(cost[i] * y[i] for i in R)
    total_flow = sum(f[s, p, i, j] for s in S for p in P for (i,j) in E)
    best_rssi = sum(rssi[i,j] * z[i,j] for (i,j) in E)
    energy = sum(e[i] for i in N if i != sink_index)
    return  installation_costs + total_flow + best_rssi + energy

model.objective = pyo.Objective(rule = objective_rule, sense = pyo.minimize)


#Constraints
def flow_conservation_rule(model, s, p, j):
	flow_in = sum(f[s, p, i, j] for i in model.Vm[j])
	flow_out = sum(f[s, p, j, k] for k in model.Vp[j])
	if j == s:
		constraint = (flow_out == b) #From every sensor
	elif j == sink_index:
		constraint = (flow_in == b) #To sink
	else:
		constraint = (flow_in == flow_out) #Conservation
	return constraint

model.flow_constraint = pyo.Constraint(S, P, N, rule = flow_conservation_rule)


def activate_edge_from_sp_two_rule(model, s, p, i, j):
    return  f[s, p, i, j] <= M * w[s, p, i, j]

model.activate_edge_from_sp_two_constraint = pyo.Constraint(S, P, E, rule = activate_edge_from_sp_two_rule)


def activate_edge_from_sp_rule(model, s, p, i, j):
    return  f[s, p, i, j] >= b * w[s, p, i, j]

model.activate_edge_from_sp_constraint = pyo.Constraint(S, P, E, rule = activate_edge_from_sp_rule)


def edge_disjoint_rule(model, s, i, j):
    return sum(f[s, p, i, j] for p in P) <= b  

model.edge_disjoint_constraint = pyo.Constraint(S, E, rule = edge_disjoint_rule)


def edge_capacity_rule(model, i, j):
    return sum(f[s, p, i, j] for s in S for p in P) <= capacity[i, j]

model.edge_capacity_constraint = pyo.Constraint(E, rule = edge_capacity_rule)


def activate_edge_with_flow_rule(model, i, j):
    if (i, j) in E:
        return sum(f[s, p, i, j] for s in S for p in P) <= M * z[i, j]
    else:
        return pyo.Constraint.Skip

model.activate_edge_with_flow_constraint = pyo.Constraint(E, rule = activate_edge_with_flow_rule)


def activate_edge_with_flow_rule_two(model, i, j):
    if (i, j) in E:
        return sum(f[s, p, i, j] for s in S for p in P) >= b * z[i, j]
    else:
        return pyo.Constraint.Skip

model.activate_edge_with_flow_two_constraint = pyo.Constraint(E, rule = activate_edge_with_flow_rule_two)


def active_repetear_edge_in(model, i, k):
	if (i,k) in E and k in model.Vp[i]:
		return z[i, k] <= y[k]
	else:
		return pyo.Constraint.Skip

model.active_repetear_edge_in_constraint = pyo.Constraint(N, R, rule = active_repetear_edge_in)


def active_repetear_edge_out(model, k, j):
	if (k,j) in E and j in model.Vm[k]:
		return z[k, j] <= y[k]
	else:
		return pyo.Constraint.Skip

model.active_repetear_edge_out_constraint = pyo.Constraint(R, N, rule = active_repetear_edge_out)


def energy_consumed_rule(model, i):
    tx_consumed = sum(f[s, p, i, j] * energy_tx[i,j] for s in S for p in P for j in model.Vp[i] if i != 1)
    rx_consumed = sum(f[s, p, k, i] * energy_rx[k,i] for s in S for p in P for k in model.Vm[i] if k != s)
    return e[i] == tx_consumed + rx_consumed
    
model.energy_consumed_constraint = pyo.Constraint(N, rule = energy_consumed_rule)         


def energy_capacity_rule(model, i):
    if i != sink_index:
        return e[i] <= initial_energy[i]
    else:
        return pyo.Constraint.Skip

model.energy_capacity_constraint = pyo.Constraint(N, rule=energy_capacity_rule)


##Solve the Problem

#Choose the solver: cplex, gurobi, mosek, glpk 
solver_name = "cplex" 

#Set solver
solver = pyo.SolverFactory(solver_name) 


#Export logfile
if solver_name == "cplex":
    solver.options["logFile"] = output_file
elif solver_name == "gurobi":
    solver.options["LogFile"] = ".txt"
elif solver_name == "glpk":
    solver.options["log"] = ".txt"
elif solver_name == "mosek":
    solver.options["log_file"] = ".txt"



#Results from solver
results = solver.solve(model, tee = True)

#Print information and objective function value from the solver
print(results)


output = "\n\n\n"
output += "#############################################################\n"
output += "#############################################################\n"
output += "#############################################################\n\n"

count_rep = 0
for i in R:
    if y[i]() >= 0.5:
        count_rep += 1
        output += f"Installed relay index: {i} \n"
output += f"Installed relays: {count_rep} \n"
    
total_flow = 0
for s in S:
    for p in P:
        for (i,j) in E:
            if f[s, p, i, j]() > 0.5:
                output += f"From {i} to {j} origin {s} path {p}: {f[s, p, i, j]()} \n"
                total_flow = total_flow + f[s, p, i, j]()
                
for (i,j) in E:
    flow_edge = 0
    for s in S:
        for p in P:        
            if f[s, p, i, j]() > 0.5:
                flow_edge = flow_edge + f[s, p, i, j]()
    if flow_edge > 0:
        output += f"Flow edge {i} to {j}: {flow_edge} \n"
        
best_rssi = 0
for (i,j) in E:
    if z[i,j]() > 0.5:
        output += f"Edge ({i},{j}) \n"
        best_rssi = best_rssi + rssi[i,j]

total_edges = 0        
for (i,j) in E:
    if z[i,j]() > 0.5:
        total_edges = total_edges + z[i,j]()
output += f"Total edges: {total_edges} \n"



mean_rssi = (best_rssi / total_edges)
output += f"Mean RSSI: {mean_rssi} \n"

    
    
for i in N:
    if e[i]() > 0:
        output += f"Energy {i}: {e[i]()} \n"
        

energy = 0
for i in N:
    energy = energy + e[i]()



installation_costs = 0  
for i in R:
    installation_costs = installation_costs + ( cost[i] * y[i]() )

    
output += f"\nThe objective function is: {model.objective()} \n"
output += f"FO Cost: {installation_costs} \n"
output += f"FO Flow: {total_flow} \n"
output += f"FO RSSI: {best_rssi} \n"
output += f"FO Energy: {energy} \n"



# Open the file in append mode and write the content
with open(output_file, "a") as file:
    file.write(output)
    
print(output)

# Notify the user that the content has been appended
print(f"Solver log and model output was saved to '{output_file}'.")


