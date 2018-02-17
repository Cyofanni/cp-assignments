#
# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
#
from ortools.constraint_solver import pywrapcp


#
# For accessing system variables
#
import sys

#
# For parsing JSON data
#
import json

#
# Parse command line
#
if len(sys.argv) != 2:
    print 'Usage: python %s <data file>' % sys.argv[0]
    sys.exit()
else:
    fname = sys.argv[1]

#
# READ PROBLEM DATA
#
with open(fname) as fin:
    data = json.load(fin)

#
# CREATE A SOLVER INSTANCE
# Signature: Solver(<solver name>)
# 
slv = pywrapcp.Solver('tanks')

#
# CREATE VARIABLES
# Signature: IntVar(<min>, <max>, <name>)
#

# Cache some data for ease of access
tanks = data['tanks']
chemicals = data['chemicals']
nt = len(tanks)
nc = len(chemicals)

# One variable for each chemical
x = []
for c in chemicals:
    # determine the list of possible tanks
    values = [i for i, t in enumerate(tanks) if c['amount'] <= t['cap']
                                             and (not c['dangerous'] or t['safe'])]
    assert(len(values) > 0) # for debug: raises a run-time error if false
    # build the variable
    x.append(slv.IntVar(values, 'chem_%d' % i))

#
# BUILD CONSTRAINTS AND ADD THEM TO THE MODEL
# Signature: Add(<constraint>)
#

# Each chemical should go to a different tank
for i in range(0, nc-1): # loop over [0, 1, ... nc-2]
    for j in range(i+1, nc): # loop over [i+1, i+2, ... nc-1]
        slv.Add(x[i] != x[j])


# Chemicals with conflicting temperatures cannot be adjacent
for i, c1 in enumerate(chemicals):
    for j, c2 in enumerate(chemicals):
        if i != j and c1['tmax'] < c2['tmin']:
            slv.Add(abs(x[i] - x[j]) > 1)

#
# THOSE ARE THE VARIABLES THAT WE WANT TO USE FOR BRANCHING
#
all_vars = x

#
# DEFINE THE SEARCH STRATEGY
# we will keep this fixed for a few more lectures
#
decision_builder = slv.Phase(all_vars,
                                slv.INT_VAR_DEFAULT,
                                slv.INT_VALUE_DEFAULT)

#
# INIT THE SEARCH PROCESS
# we will keep this fixed for a few more lectures
#
time_limit = 20000
search_monitors = [slv.SearchLog(500000), slv.TimeLimit(time_limit)]
slv.NewSearch(decision_builder, search_monitors)

#
# Function to print a solution
# Input format: list with the tank index chosen for each chemical
#
def print_sol(tank_for_chemical):
    # Obtain and transform the solution data
    cap = ['%5d' % t['cap'] for t in tanks]
    idx = [' --- ' for i in range(nt)] # chemical index in ech tank
    amt = [' --- ' for i in range(nt)] # chemical amount in each tank
    tmin = [' --- ' for i in range(nt)] # chemical tmin in each tank
    tmax = [' --- ' for i in range(nt)] # chemical tmax in each tank
    for i in range(nt):
        for j in range(nc):
            if tank_for_chemical[j] == i:
                idx[i] = '%5d' % j
                amt[i] = '%5d' % chemicals[j]['amount']
                tmin[i] = '%5d' % chemicals[j]['tmin']
                tmax[i] = '%5d' % chemicals[j]['tmax']

    print 'CHEMICALS : %s' % ', '.join(idx)
    print 'CAPACITIES: %s' % ', '.join(cap)
    print 'AMOUNTS   : %s' % ', '.join(amt)
    print 'TMIN      : %s' % ', '.join(tmin)
    print 'TMAX      : %s' % ', '.join(tmax)

#
# Search for a solution
#
nsol = 0
while slv.NextSolution():
    print 'SOLUTION FOUND =========================='

    # Obtain the chosen tank for each chemical
    tank_for_chemical = [var.Value() for var in x]

    # print the solution
    print_sol(tank_for_chemical)

    print 'END OF SOLUTION =========================='

    # WE WANT A SINGLE SOLUTION
    nsol += 1
    break
#
# END THE SEARCH PROCESS
#
slv.EndSearch()

if nsol == 0:
    print 'no solution found'

# Print solution information
print 'Number of branches: %d' % slv.Branches()
print 'Computation time: %f (ms)' % slv.WallTime()
if slv.WallTime() > time_limit:
    print 'Time limit exceeded'