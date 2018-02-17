#
# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
#
from ortools.constraint_solver import pywrapcp
import sys
import json

#
# A FUNCTION TO BUILD AND SOLVE A MODEL
# time_limit: if None, not time limit is employed. If integer, a time limit is
#             enforced, but optimality is no longer guaranteed!
def solve_problem(data, time_limit = None, lb = None, ub = None):
    # Cache some useful data
    setups = data['setups']
    order_list = data['order_list']
    unit_list = data['unit_list']
    order_table = data['order_table']
    no = len(order_list)
    nu = len(unit_list)

    # Build solver instance
    slv = pywrapcp.Solver('production-scheduling')

    #
    # CREATE VARIABLES
    #

    # Compute a safe time horizon
    eoh = max(o['dline'] for o in order_list)

    # A variable for each unit to be produced
    x = [slv.IntVar(0, eoh, 's_%d' % i) for i in range(nu)]

    # Objective variable
    z = slv.IntVar(0, eoh, 'z')

    #
    # BUILD CONSTRAINTS AND ADD THEM TO THE MODEL
    #

    # no parallel manufacturing
    for i in range(nu):
        for j in range(i+1, nu):
            slv.Add(x[i] != x[j])

    # deadline constraints
    for i, unit in enumerate(unit_list):
        slv.Add(x[i] <= unit['dline'])

    # setup times
    for p1, p2 in setups:
        for i, u1 in enumerate(unit_list):
            for j, u2 in enumerate(unit_list):
                if i != j and u1['prod'] == p1 and u2['prod'] == p2:
                    slv.Add((x[i] + 1 < x[j]) != (x[j] < x[i]))

    # z definition constraints
    slv.Add(z == slv.Max(x))

    # Optional bounding constraints
    if lb != None:
        slv.Add(z >= lb)
    if ub != None:
        slv.Add(z <= ub)

    #
    # THOSE ARE THE VARIABLES THAT WE WANT TO USE FOR BRANCHING
    #
    all_vars = x

    # DEFINE THE SEARCH STRATEGY
    decision_builder = slv.Phase(all_vars,
                                 slv.INT_VAR_DEFAULT,
                                 slv.INT_VALUE_DEFAULT)

    # INIT THE SEARCH PROCESS

    # log monitor (just to have some feedback)
    # search_monitors = [slv.SearchLog(500000)]
    search_monitors = []
    # enforce a time limit (if requested)
    if time_limit:
       search_monitors.append(slv.TimeLimit(time_limit))
    # enable branch and bound
    if lb == None and ub == None:
        search_monitors.append(slv.Minimize(z, 1))

    # init search
    slv.NewSearch(decision_builder, search_monitors)

    # SEARCH FOR A FEASIBLE SOLUTION
    zbest = None
    while slv.NextSolution():
        #
        # PRINT SOLUTION
        #
        print ', '.join('s%d: %d' % (i, var.Value()) for i, var in enumerate(x))
        print 'z: %d' % z.Value()

        #
        # STORE SOLUTION VALUE
        #
        zbest = z.Value()

        # If not in branch & bound mode, stop after a solution is found
        if lb != None or ub != None:
            break

    # print something if no solution was found
    if zbest == None:
        print '*** No solution found'

    # print stats
    branches, time = slv.Branches(), slv.WallTime()
    print '*** Number of branches: %d' % branches
    print '*** Computation time: %f (ms)' % time
    if time_limit != None and slv.WallTime() > time_limit:
        print '*** Time limit exceeded'

    # END THE SEARCH PROCESS
    slv.EndSearch()

    # Return the solution value
    return zbest, branches, time


#
# LOAD PROBLEM DATA
#
if len(sys.argv) != 2:
    print 'Usage: python %s <data file>' % sys.argv[0]
    sys.exit()
else:
    fname = sys.argv[1]

with open(fname) as fin:
    data = json.load(fin)

#
# CALL THE SOLUTION APPROACH
#
solve_problem(data, time_limit=30000)
