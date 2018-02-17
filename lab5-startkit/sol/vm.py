# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
from ortools.constraint_solver import pywrapcp
import sys
import json
import math

# LOAD PROBLEM DATA
if len(sys.argv) != 2:
    print 'Usage: python %s <data file>' % sys.argv[0]
    sys.exit()
else:
    fname = sys.argv[1]

with open(fname) as fin:
    data = json.load(fin)

# Cache some useful data
services = data['services']
nservices = len(services)
nservers = data['nservers']
cap_cpu = data['cap_cpu']

# split services into individual VMs
vm_svc = [] # service idx, for each VM
vm_cpu = [] # CPU requirement, for each VM
for k, svc in enumerate(services):
    vm_num = svc['vm_num']
    vm_svc += [k] * vm_num
    vm_cpu += [svc['vm_cpu']] * vm_num
# Total number of virtual machines
nvm = len(vm_svc)

# Build solver instance
slv = pywrapcp.Solver('vm-reassignment')

# Cost variable (number of used services)
z = slv.IntVar(0, nservers, 'z')

# One variable for each VM
x = [slv.IntVar(0, nservers-1, 'x_%d' % i) for i in range(nvm)]

# VMs within the same service should go on different servers
# NOTE: this constraint can be replaced with an AllDifferent, for better propagation
# for i in range(nvm):
#     for j in range(i+1, nvm):
#         if vm_svc[i] == vm_svc[j]:
#             slv.Add(x[i] != x[j])

# Alldiff for VMs in the same service
# NOTE: once the symmetry breaking constraints are in, the AllDifferent constraints
# are redundant. Since on this problem they do not provide a lot of additional
# filtering, it's better to leaver them commented
# for k in range(nservices):
#     xk = [x[i] for i in range(nvm) if vm_svc[i] == k]
#     slv.Add(slv.AllDifferent(xk))

# Symmetry breaking between VMs relatd to the same service (they are equivalent)
# NOTE: I am exploiting the fact the VMs related to to the same service are
# contiguous in the vm_svc list
for i in range(nvm-1):
    if vm_svc[i] == vm_svc[i+1]:
        slv.Add(x[i] > x[i+1])

# CPU capacity constraints
srv_usgs = []
for j in range(nservers):
    # Obtain an expression for the total CPU requirement
    # NOTE: I am using slv.Sum, since it is MUCH more efficient
    usg = slv.Sum(v * (x[i] == j) for i, v in enumerate(vm_cpu))
    # Store the expression that corresponds to the server usae
    srv_usgs.append(usg)
    # Post CPU capacity constraint
    slv.Add(usg <= cap_cpu)

# Basic symmetry breaking between servers: all servers are identical, therefore
# I can try to enforce an order on how they should be used. For example, I can
# force the first server to be more used than the second, and so on
# NOTE: this symmetry-breaking constraint may _severly_ affect the search strategy.
# Using a wrong order simply sinks the performance. Using the order employed here
# reduces the time to prove optimality, _but_ it also increases the time to find
# the best solution
for j in range(nservers-1):
    slv.Add(srv_usgs[j] >= srv_usgs[j+1])

# Add a static lower bound (i.e. the maximum number of VMs in a service)
lb1 = max(s['vm_num'] for s in services)
slv.Add(z >= lb1)

# Add a second, static lower bound (the overall CPU capacity should be sufficient
# house all VMs)
slv.Add(cap_cpu * z >= sum(vm_cpu))

# Cost definition (similar to the map coloring problem)
# NOTE: here I am using slv.Max to define the problem objective. It is more efficient
# and it allows better propagation (i.e. when a new bounding constraint in the for
# "z <= sol - 1" is posted, all domains are immediately pruned
# slv.Add(z == slv.Sum((srv_usgs[j] > 0 for j in range(nservers))))
slv.Add(z == 1 + slv.Max(x))

# DEFINE THE SEARCH STRATEGY
decision_builder = slv.Phase(x,
                             slv.INT_VAR_DEFAULT,
                             slv.INT_VALUE_DEFAULT)

# INIT THE SEARCH PROCESS
time_limit = 30000
search_monitors = [slv.Minimize(z, 1),
                   slv.TimeLimit(time_limit)]
# init search
slv.NewSearch(decision_builder, search_monitors)

# SEARCH FOR A FEASIBLE SOLUTION
zbest = None
while slv.NextSolution():
    #
    # PRINT SOLUTION
    #
    print '--- Solution found, time: %.3f (sec), branches: %d' % \
                (slv.WallTime()/1000.0, slv.Branches())
    print '--- z: %d' % z.Value()
    print '--- x:', ', '.join('%3d' % var.Value() for var in x)

    # NOTE: srv_usgs[i] corresponds to a sum, i.e. to an _expression_. Expressions
    # in or-tools do not have a "Value()" method, but they do have a "Min()" and
    # "Max()" method for accessing the bounds of their domain
    cpu_req = [srv_usgs[j].Min() for j in range(nservers)]
    print '--- usg:', ', '.join('%3d' % v for v in cpu_req)
    print

    # STORE SOLUTION VALUE
    zbest = z.Value()

# END THE SEARCH PROCESS
slv.EndSearch()

# obtain stats
branches, time = slv.Branches(), slv.WallTime()
# time capping
time = max(1, time)

# print stats
print '--- FINAL STATS'
if zbest == None:
    print '--- No solution found'
print '--- Number of branches: %d' % branches
print '--- Computation time: %.3f (sec)' % (time / 1000.0)
if slv.WallTime() > time_limit:
    print '--- Time limit exceeded'
