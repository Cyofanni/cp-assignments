#!/usr/bin/env python

# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
from ortools.constraint_solver import pywrapcp
import sys
import json
import random

# =============================================================================
# A function to solve the main problem
# =============================================================================

def solve(acts, time_limit=None, strategy=0, res_mode=0,
          preorder=None, old_zbest=None, stop_after=None):
    '''
    Parameters:
    - acts: the list of activities (as dictionaries)
    - time_limit: the time limit in msec. If None, no time limit will be used
    - strategy: an integer to specify the desired search strategy
    - res_mod: if 0, a single disjunctive constraint will be used. If 1,
      multiple pairwise disjunctions will be posted. The second approach
      has weaker propagation, but can be much faster (especially in combination
      with "preorder")
    - preorder: pre-ordered sequence of activities. Precendece constraints will
      be posted to as to ensure that the sequence is respected. If "res_mode=1"
      is employed, pairwise disjuntions will not be posted between pairs of
      activities in the sequence
    - old_zbest: if not None, an additional bounding constraint will be posted
    - stop_after: stop after a certain number of solutions has been found. If
      None, the solver will run until completion (or a time limit)
    '''
    # Obtain some computed values (for convenience)
    n = len(acts) # number of activities
    eoh = sum(a['dur'] for a in acts) # end of horizon

    # Build the solver
    slv = pywrapcp.Solver('min-wt')

    # ------------------------------------------------------------------------ 
    # Build the model
    # ------------------------------------------------------------------------ 

    # Build a list of integer variables
    X = [slv.FixedDurationIntervalVar(0, eoh, a['dur'], False, 'X[%d]' % i)
            for i, a in enumerate(acts)]
    # Objective variable
    Z = slv.IntVar(0, sys.maxint, 'Z')

    # Define the resource constraints
    if res_mode == 0:
        # - version 1: a single, disjuntive constraint
        slv.Add(slv.DisjunctiveConstraint(X, 'the-resource'))
    elif res_mode == 1:
        # - version 2: post pairwise disjunctions
        for i in range(n):
            for j in range(i+1, n):
                if preorder is None or i not in preorder or j not in preorder:
                    slv.Add(slv.TemporalDisjunction(X[i], X[j]))
    else:
        print 'Invalid "res_mode": %d' % res_mode
        sys.exit()


    # Define the tardiness terms
    T = [slv.IntVar(0, sys.maxint, 'T[%d]' % i) for i in range(n)]
    for i, a in enumerate(acts):
        endexpr = X[i].EndExpr()
        slv.Add(T[i] == slv.Max(endexpr - a['dln'], 0))

    # Define the tardiness objective
    slv.Add(Z == slv.ScalProd(T, [a['wgt'] for a in acts]))
    # slv.Add(Z == slv.Sum([a['wgt'] * T[i] for i, a in enumerate(acts)]))

    # Add the preordered variables
    if preorder is not None:
        for i, j in zip(preorder[:-1], preorder[1:]):
            slv.Add(X[j].StartsAfterEnd(X[i]))

    # Add a cost constraint, if a previous value to be has been passed
    if old_zbest is not None:
        slv.Add(Z < old_zbest)

    # ------------------------------------------------------------------------ 
    # Define the search strategy
    # ------------------------------------------------------------------------ 

    if strategy == 0:
        # Example 1: search on the interval variables. Possible strategies:
        # - INTERVAL_SET_TIMES_FORWARD 
        # - INTERVAL_SET_TIMES_BACKWARD
        db = slv.Phase(X, slv.INTERVAL_SET_TIMES_FORWARD)
    elif strategy == 1:
        # Example 2: search on the start variables (use any usual strategy)
        # NOTE: this is just an example, branching in this way does not
        S = [slv.IntVar(0, eoh, 'S[%d]' % i) for i in range(n)]
        for i in range(n):
            slv.Add(S[i] == X[i].StartExpr())
        db = slv.Phase(S, slv.CHOOSE_MIN_SIZE, slv.ASSIGN_MIN_VALUE)
    else:
        print 'Invalid strategy %d' % strategy
        sys.exit()

    # Define the search monitors
    search_monitors = [slv.Minimize(Z, 1)]

    # Add a time limit, if requested
    if time_limit is not None:
       search_monitors.append(slv.TimeLimit(time_limit))

    # ------------------------------------------------------------------------ 
    # Start search
    # ------------------------------------------------------------------------ 

    # INIT SEARCH
    slv.NewSearch(db, search_monitors)

    # SEARCH FOR A FEASIBLE SOLUTION
    zbest = None
    sbest = None
    scnt = 0
    while slv.NextSolution():
        # Store the solution and its value. Format: sequence of activities
        # indices, sorted by increasing start time
        zbest = Z.Value()
        sbest = sorted(range(n), key=lambda i: X[i].StartMin())
        # print solution
        print '--- Solution found, time: %.3f (sec), branches: %d, cost: %d' % \
                    (slv.WallTime()/1000.0, slv.Branches(), Z.Value())
        print '    %s' % ' '.join('%d' % v for v in sbest)
        # Count one more solution (and possibly stop)
        scnt += 1
        if stop_after is not None and scnt >= stop_after:
            break

    # END THE SEARCH PROCESS
    slv.EndSearch()

    # obtain stats
    branches, time = slv.Branches(), slv.WallTime()
    # time capping
    time = max(1, time)

    # print stats
    if zbest == None:
        print '--- No solution found'
    else:
        print '--- Best solution found: %d' % (zbest)
    print '--- Number of branches: %d' % branches
    print '--- Computation time: %.3f (sec)' % (time / 1000.0)
    if time > time_limit:
        print '--- Time limit exceeded'

    return time, branches, zbest, sbest


# =============================================================================
# DEFINE SOME FUNCTIONS FOR CONVENIENCE
# =============================================================================

def random_draw(L, size, rand):
    L = [v for v in L] # make a copy of the input list
    R = []
    while len(R) < size and len(L) > 0:
        idx = rand.randint(0, len(L)-1)
        R.append(L[idx])
        L.pop(idx)
    return R


def compute_costs(acts, sbest):
    n = len(acts)
    starts = []
    for i in range(n):
        idx = sbest.index(i)
        starts.append(sum(acts[j]['dur'] for j in sbest[0:idx]))
    costs = [max(0, starts[i]-a['dln'])*a['wgt'] for i, a in enumerate(acts)]
    return costs


def compute_weighted_slack(acts, sbest):
    n = len(acts)
    starts = []
    for i in range(n):
        idx = sbest.index(i)
        starts.append(sum(acts[j]['dur'] for j in sbest[0:idx]))
    wslack = [(starts[i]-a['dln'])*a['wgt'] for i, a in enumerate(acts)]
    return wslack


# =============================================================================
# MAIN SCRIPT
# =============================================================================

# Load data
if len(sys.argv) != 2:
    print 'Usage: python %s <data file>' % sys.argv[0]
    sys.exit()
else:
    fname = sys.argv[1]

with open(fname) as fin:
    data = json.load(fin)

# Cache some useful data
acts = data['acts']

# Sort activities by some criterion
# NOTE: currently a dummy criterion is used (all scores are 0)
acts = sorted(acts, key=lambda a: 0)

# SEARCH FOR A FIRST SOLUTION
print '=== SEARCHING FOR AN INITIAL SOLUTION'
time, branches, zbest, sbest = solve(acts, time_limit=None, strategy=0,
                                     stop_after=1, res_mode=0)

# START AN LNS-LIKE PROCESS

# Parameters
global_time_limit = 40000
local_time_limit = 2000
neighborhood_size = 5
rng = random.Random(100) # RNG with fixed seed

cnt = 0
while time < global_time_limit:
    # Choose which variables to relax...
    allacts = range(len(acts))
    relaxed = random_draw(allacts, neighborhood_size, rng)

    # ...And by consequence the variables to fix
    preorder = [i for i in sbest if i not in relaxed]

    # Solve the subproblem
    print
    print '=== LNS ITERATION #%d' % cnt
    lt, lb, lz, ls = solve(acts, time_limit=local_time_limit,
                           strategy=0, res_mode=0,
                           preorder=preorder, old_zbest=zbest, stop_after=1)

    # Update stats
    cnt += 1
    time += lt
    branches += lb
    if lz is not None:
        zbest = lz
        sbest = ls

    # Early stop (when the known optimal solution is found)
    if zbest == data['best']:
        print
        print '=== Known optimum found! Stopping for convenience'
        break


# print stats
print
print '=== FINAL STATS'
if zbest == None:
    print '=== No solution found'
else:
    print '=== Best solution found: %d' % (zbest)
print '=== Best solution known: %d' % (data['best'])
print '=== Number of branches: %d' % branches
print '=== Computation time: %.3f (sec)' % (time / 1000.0)
if time > global_time_limit:
    print '=== Time limit exceeded'

