# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
from ortools.constraint_solver import pywrapcp
import sys
import json
import math

# ============================================================================== 
# Custom decision builder
# ============================================================================== 

class SetSuccBasedOnDates(pywrapcp.PyDecisionBuilder):
    '''
    A dedicated DecisionBuilder for our problem
    '''

    def __init__(self, succ, date):
        pywrapcp.PyDecisionBuilder.__init__(self)
        self.succ = succ
        self.date = date

    def Next(self, slv):
        # Assign the "succ" variables based on the valus of the "date" variablesk

        # Sort the unit indices by increasing deadline
        idx = sorted(range(len(self.date)), key=lambda i: self.date[i].Value())

        # Look for an unbound succ varible
        target = None
        target_pos = None
        for k, i in enumerate(idx):
            if not self.succ[i].Bound():
                target = i
                target_pos = k
                break
        # If no unbound unit was found, end search
        if target is None:
            return None

        # If the target index is that of the last (i.e. fake) unit, then assign
        # as successors the first unit
        if target_pos == len(self.date):
            return slv.AssignVariableValueOrFail(self.succ[target], idx[0])

        # Otherwise, assign as successor the next unit in the sequence
        return slv.AssignVariableValueOrFail(self.succ[target], idx[target_pos+1])


class SetDatesBasedOnSucc(pywrapcp.PyDecisionBuilder):
    '''
    A dedicated DecisionBuilder for our problem
    '''

    def __init__(self, succ, date):
        pywrapcp.PyDecisionBuilder.__init__(self)
        self.succ = succ
        self.date = date

    def Next(self, slv):
        # Assign the "date" variables based on the valus of the "succ" variables

        # Reconstruct the chain of successors
        nu = len(self.succ) - 1
        current = nu # Start from the fake unit
        Q = []
        # Follow the successor chain until the loop is closed
        while self.succ[current].Value() != nu:
            current = self.succ[current].Value()
            Q.append(current)

        # Process the chain from back to start
        for i in reversed(Q):
            # If the correspnding data variable is not bound...
            if not self.date[i].Bound():
                # ...Assign it to the maximum possible value
                return slv.AssignVariableValueOrFail(self.date[i], self.date[i].Max())

        # If not unbound variable was found, return None
        return None


class SetSuccByCost(pywrapcp.PyDecisionBuilder):
    '''
    A dedicated DecisionBuilder for our problem
    '''

    def __init__(self, succ, ext_tcosts):
        pywrapcp.PyDecisionBuilder.__init__(self)
        self.succ = succ
        self.ext_tcosts = ext_tcosts

    def Next(self, slv):
        # Choose the variable with the smallest domain. Rank values by increasing
        # transition cost

        # Select the indices of units with non-bound succ variable
        nu = len(self.succ)-1
        idx = [i for i in range(nu+1) if not self.succ[i].Bound()]

        # If all variables are bound, end this phase of search
        if len(idx) == 0:
            return None

        # Search for the succ variable with smallest domain.
        tgt = None
        tgt_size = None
        for i in idx:
            cur_size= self.succ[i].Size()
            if tgt is None or (cur_size < tgt_size):
                tgt = i
                tgt_size = cur_size

        # Assign the successor with minimum cost
        var = self.succ[tgt]
        val = None
        val_tcost = None
        for k in range(var.Min(), var.Max()+1):
            if var.Contains(k):
                cur_cost = self.ext_tcosts[k]
                if val is None or cur_cost < val_tcost:
                    val = k
                    val_tcost = cur_cost

        # Return a decision object
        return slv.AssignVariableValue(self.succ[tgt], val)



class PreferBoundVars(pywrapcp.PyDecisionBuilder):
    '''
    A dedicated DecisionBuilder for our problem
    '''

    def __init__(self, succ, ext_tcosts):
        pywrapcp.PyDecisionBuilder.__init__(self)
        self.succ = succ
        self.ext_tcosts = ext_tcosts

    def Next(self, slv):
        # Choose the variable with the smallest domain. Rank values by increasing
        # transition cost

        # Select the indices of units with non-bound succ variable
        nu = len(self.succ)-1
        idx1 = []
        idx2 = []
        for i in range(nu+1):
            var = self.succ[i]
            if self.succ[i].Bound():
                var2 = self.succ[var.Value()]
                if not var2.Bound():
                    idx1.append(var.Value())
            else:
                idx2.append(i)
        # print self.succ
        # print [(i, self.succ[i].Size()) for i in range(nu+1)]
        # print idx1
        # print idx2
        # if len(idx1) > 0:
        #     sys.exit()

        # If all variables are bound, end this phase of search
        if len(idx1) == 0 and len(idx2) == 0:
            return None

        # Search for the succ variable with smallest domain. Prefer the idx1
        # list over the idx2 list
        tgt = None
        tgt_size = None
        candidates = idx1 if len(idx1) > 0 else idx2
        for i in candidates:
            cur_size = self.succ[i].Size()
            if tgt is None or cur_size < tgt_size:
                tgt = i
                tgt_size = cur_size

        # Assign the successor with the smallest domain. Break ties by cost
        var = self.succ[tgt]
        val = var.Min()
        # val = None
        # val_size = None
        # val_cost = None
        # for k in range(var.Min(), var.Max()+1):
        #     if var.Contains(k):
        #         cur_size = self.succ[k].Size()
        #         cur_cost = self.ext_tcosts[i][k]
        #         if val is None or \
        #            (cur_size < val_size) or \
        #            (cur_size == val_size and cur_cost < val_cost):
        #             val = k
        #             val_size = cur_size
        #             val_cost = cur_cost

        # Return a decision object
        return slv.AssignVariableValue(var, val)




# LOAD PROBLEM DATA
if len(sys.argv) != 2:
    print 'Usage: python %s <data file>' % sys.argv[0]
    sys.exit()
else:
    fname = sys.argv[1]

with open(fname) as fin:
    data = json.load(fin)

# Cache some useful data
np = data['n_periods']
ni = data['n_items']
dlines = data['dlines']
stcost = data['stcost']
tcosts = data['tcosts']
bounds = data['bounds']

# unit descriptors
units = []
for k, l in enumerate(dlines):
    for d in l:
        units.append((k, d))

# Sort units by increasing deadline SORT
# units = sorted(units, key=lambda u: u[1])

# Flat list of deadlines and corresponding types (one element per item unit)
flat_dlines = [d for k, d in units]
flat_types = [k for k, d in units]

# Obtain the number of units
nu = len(flat_dlines)

# Build solver instance
slv = pywrapcp.Solver('lot-sizing')

# One position variable for each unit (+1 for a fake last unit)
# NOTE: the domains are also correspondingly enlarged
date = [slv.IntVar(0, np, 'date[%d]' % i) for i in range(nu+1)]

# One successor variable for each unit (+1 for a fake last unit)
# NOTE: the domains are also correspondingly enlarged
succ = [slv.IntVar(0, nu, 'succ[%d]' % i) for i in range(nu+1)]

# Cost variable
z = slv.IntVar(0, sys.maxint, 'z')

# The successors must form a cycle
slv.Add(slv.Circuit(succ))

# Connect the successor and position variables
for i in range(nu):
    slv.Add(date[i] < slv.Element(date, succ[i]))
# The fake item must come after all the rest
slv.Add(date[nu] == np)

# Enforce the capacity constraints
slv.Add(slv.AllDifferent(date))

# Enforce the deadline constraints
for i in range(nu):
    slv.Add(date[i] <= flat_dlines[i])

# Some symmetry breaking constraints
for i in range(nu):
    slv.Add(succ[i] != i)
    for j in range(nu):
        if i != j:
            if flat_types[i] == flat_types[j] and flat_dlines[i] < flat_dlines[j]:
                # We can force the scheduling order of the two units
                slv.Add(date[i] < date[j])
                slv.Add(succ[j] != i)

# Compute the transition costs (an expression)
# step 1: build an extended transition cost matrix (unit-to-unit, rather than
#         type-to-type).
ext_tcosts = [[tcosts[flat_types[i]][flat_types[j]] for j in range(nu)] for i in range(nu)]
# step 2: we add one extra column for transition costs to the fake unit
for i in range(nu):
    ext_tcosts[i].append(0)
# step 3: add one extra for the fake unit
ext_tcosts.append([0 for i in range(nu+1)])
# step 4: finally, we can build an expression for the transition costs
tot_tcosts = slv.Sum([succ[i].IndexOf(ext_tcosts[i]) for i in range(nu)])

# cum_tcosts = [slv.IntVar(0, sys.maxint, 'ct[%d]' % i) for i in range(nu+1)]
# active = [slv.IntVar(0, 1, 'act[%d]' % i) for i in range(nu+1)]
# slv.Add(slv.PathCumul(succ, active, cum_tcosts,
#                       [succ[i].IndexOf(ext_tcosts[i]) for i in range(nu+1)]))
# for i in range(nu):
#     slv.Add(active[i] == 1)
# slv.Add(active[nu] == 0)
# tot_tcosts = cum_tcosts[nu]

# Compute the stocking costs (an expression)
tot_stcost = stcost * slv.Sum([flat_dlines[i]-date[i] for i in range(nu)])

# Define the total cost
slv.Add(z == tot_tcosts + tot_stcost)
# slv.Add(z == tot_stcost)

# DEFINE THE SEARCH STRATEGY
db1 = slv.Phase(succ,
                slv.CHOOSE_MIN_SIZE,
                slv.ASSIGN_MIN_VALUE)
db1b = slv.Phase(succ,
                 slv.CHOOSE_FIRST_UNBOUND,
                 slv.ASSIGN_MIN_VALUE)
db2 = slv.Phase(date,
                slv.CHOOSE_MIN_SIZE,
                slv.ASSIGN_MAX_VALUE)
db2b = slv.Phase(date,
                 slv.CHOOSE_MAX_SIZE,
                 slv.ASSIGN_MAX_VALUE)

# Custom decision builders (unused)
db3 = SetSuccBasedOnDates(succ, date)
db4 = SetDatesBasedOnSucc(succ, date)
db5 = SetSuccByCost(succ, ext_tcosts)
db6 = PreferBoundVars(succ, ext_tcosts)
dbz = slv.Phase([z], slv.INT_VAR_DEFAULT, slv.SPLIT_LOWER_HALF)

# A possible, counterintuitive, solution
# 1) Branch first on the date variables, 
# 2) Choose the one with the maximum domain, i.e. with the maximum flexibility
# This works probably because by fixing this variable first we are (implicitly)
# constraining also the "succ" variables
decision_builder = slv.Compose([db2b, db1])

# A second possible solution
# 1) Branch first on the "succ" variables
# 2) Bbreak ties by increasing deadline. This requires to pre-sort the unit
#    (de-comment the line with the "SORT" label)
decision_builder = slv.Compose([db1, db2])

# INIT THE SEARCH PROCESS
time_limit = 30000
branches_limit = 2000000
search_monitors = [slv.Minimize(z, 1),
                   slv.BranchesLimit(branches_limit)]
# init search
slv.NewSearch(decision_builder, search_monitors)

# SEARCH FOR A FEASIBLE SOLUTION
zbest = None
zbest_branches = None
while slv.NextSolution():
    #
    # PRINT SOLUTION
    #
    print '--- Solution found, time: %.3f (sec), branches: %d' % \
                (slv.WallTime()/1000.0, slv.Branches())
    print '--- z: %d' % z.Value()

    # Obtain the date in a more easy to plot format
    unit_sched = [-1 for j in range(np)]
    type_sched = [-1 for j in range(np)]
    for i in range(nu):
        unit_sched[date[i].Value()] = i
        type_sched[date[i].Value()] = flat_types[i] 
    print '--- units: %s' % ' '.join(' X ' if v < 0 else '%3d' % v for v in unit_sched)
    print '--- types: %s' % ' '.join(' X ' if v < 0 else '%3d' % v for v in type_sched)
    print

    # STORE SOLUTION VALUE
    zbest = z.Value()
    zbest_branches = slv.Branches()

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
else:
    print '--- Best solution found: %d, at #branches: %d' % (zbest, zbest_branches)
    print '--- Best known solution (from the literature): %d' % bounds[-1]
print '--- Number of branches: %d' % branches
print '--- Computation time: %.3f (sec)' % (time / 1000.0)
if slv.WallTime() > time_limit:
    print '--- Time limit exceeded'
