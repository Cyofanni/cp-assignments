# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
from ortools.constraint_solver import pywrapcp
import sys
import json
import math

# ==============================================================================
# Load and pre-process problem data
# ==============================================================================

if len(sys.argv) != 2:
    print 'Usage: python %s <data file>' % sys.argv[0]
    sys.exit()
else:
    fname = sys.argv[1]

with open(fname) as fin:
    data = json.load(fin)

# Cache some useful data
shifts = data['shifts']
employees = data['employees']
onreq = data['onreq']
offreq = data['offreq']
cover = data['cover']
eoh = data['eoh']

ns = len(shifts) # Number of shifts
ne = len(employees) # Number of employees
non = len(onreq) # Number of "on" requests
noff = len(offreq) # Number of "off" requests
nc = len(cover) # Number of cover requirements

# Obtain a (possibly ordered) list of shift ids
sids = sorted([s for s in shifts.keys()])
# Obtain a (possibly ordered) list of employee ids
eids = sorted([e for e in employees.keys()])

# Convert shift ids to integers
# 1. build a map
smap = {s: i for i, s in enumerate(sids)}
# 2. rebuild the list of shifts
newshifts = [shifts[s] for s in sids]
for i, shift in enumerate(newshifts):
    newshifts[i]['inc'] = [smap[s] for s in shift['inc']]
shifts = newshifts
# 3. Update references in the employee list
for e, employee in employees.items():
    employees[e]['maxshifts'] = {smap[s]: v for s, v in employee['maxshifts'].items()}
# 4. Update references in the "on" requests
for i, req in enumerate(onreq):
    onreq[i]['shift'] = smap[req['shift']]
# 5. Update references in the "off" requests
for i, req in enumerate(offreq):
    offreq[i]['shift'] = smap[req['shift']]
# 6. Update references in the cover data
for i, cov in enumerate(cover):
    cover[i]['shift'] = smap[cov['shift']]

# Convert employee indices to integers
# 1. build a map
emap = {e: i for i, e in enumerate(eids)}
# 2. rebuild the list of employees
employees = [employees[e] for e in eids]
# 3. Update references in the "on" requests
for i, req in enumerate(onreq):
    onreq[i]['emp'] = emap[req['emp']]
# 5. Update references in the "off" requests
for i, req in enumerate(offreq):
    offreq[i]['emp'] = emap[req['emp']]

# ==============================================================================
# Start model construction
# ==============================================================================

# Build solver instance
slv = pywrapcp.Solver('shift-scheduling')

# One variable for each employee and day in the horizon, to represent the shift
# allocation. A day off is represented as "ns"
X = {(i,j):slv.IntVar(0, ns, 'X[%d,%d]' % (i,j)) for i in range(ne) for j in range(eoh)}

# One variable for each employee and day in the horizon, to represent whether the
# employee is working or not
W = {(i,j):slv.IntVar(0, 1, 'W[%d,%d]' % (i,j)) for i in range(ne) for j in range(eoh)}

# Cost variable
Z = slv.IntVar(0, sys.maxint, 'z')

# Chaining constraints between the X and W variables
for i in range(ne):
    for j in range(eoh):
        slv.Add(W[i,j] == (X[i,j] != ns))

# Compatibility constraint between consecutive shifts
# 1. Build a table with the compatible assignments
T = []
for k1 in range(ns+1):
    for k2 in range(ns+1):
        if k1 == ns or k2 not in shifts[k1]['inc']:
            T.append([k1, k2])
# 2. Post a bunch of table constraints
for i in range(ne):
    for j in range(eoh-1):
        slv.Add(slv.AllowedAssignments([X[i,j], X[i,j+1]], T))

# Enforce the maximum cardinality constraint for all shifts
for i, emp in enumerate(employees):
    # Build the cardinality variables
    cards = [slv.IntVar(0, emp['maxshifts'][k]) for k in range(ns)]
    cards = [slv.IntVar(0, emp['maxshifts'][k]) for k in range(ns)]
    cards.append(slv.IntVar(0, sys.maxint)) # Add one term for the days off
    # Build a Distribute constraint
    Xloc = [X[i,j] for j in range(eoh)]
    Vloc = range(ns+1)
    slv.Add(slv.Distribute(Xloc, Vloc, cards))

# Enforce the bounds on the number of minutes
# 1. build a list with the duration of each shift
M = [s['dur'] for s in shifts]
M.append(0) # Add one term for the day off
# 2. add one variable for each employee to keep track of the working time
D = {(i,j) : slv.IntVar(0, max(M), 'D[%d,%d]' % (i,j)) for i in range(ne) for j in range(eoh)}
for i, emp in enumerate(employees):
    # Use an element constrain to count the duration of a single shift
    for j in range(eoh):
        slv.Add(D[i,j] == X[i,j].IndexOf(M))
    # Constrain the sum
    Dtot = slv.Sum([D[i,j] for j in range(eoh)])
    slv.Add(Dtot <= emp['maxdur'])
    slv.Add(Dtot >= emp['mindur'])
    # slv.Add(Wt[i] == slv.Sum([X[i,j].IndexOf(M) for j in range(eoh)]))

# Enforce the bounds on the number of consecutive working days/days off
for i, emp in enumerate(employees):
    # Define an automaton to describe the feasible sequences
    # 1. States = "1 day off", "2 d.o", ... "enough d. o.",
    #             "1 working day", "2 w.d.", ... "min w.d.", ..., "max w.d."
    do1 = 0 # One day off
    doe = emp['minconsoff']-1 # Enough days off
    wd1 = doe+1 # One working day
    wdmin = emp['mincons'] + doe # Min. working days
    wdmax = emp['maxcons'] + doe # Max working days
    # 2. Define a table with the possible transitions (the transition variables
    # are the W variables). Each transition is in the form (current state,
    # variable value, new state)
    T2 = []
    for j in range(do1, doe): # Counting the days off
        T2.append((j, 0, j+1)) # One more day off (the other transition is forbidden)
    T2.append((doe, 0, doe)) # Stop counting the days off
    T2.append((doe, 1, wd1)) # One working day
    for j in range(wd1, wdmin): # Counting the working days (1)
        T2.append((j, 1, j+1)) # One more working day (the other trans. is forbidden)
    for j in range(wdmin, wdmax): # Counting the working days (2)
        T2.append((j, 1, j+1)) # One more working day
        T2.append((j, 0, do1)) # One day off
    T2.append((wdmax, 0, do1)) # One day off (the other transition is forbidden)
    # Buid a regular constraint (NOTE: it is assumed that there is an infinite
    # amount of days off both before and after the planning period. Therefore, the
    # initial state is "doe" and all the "doX" state are accepting states. The states
    # between "wdmin" and "wdmax" (included) are also accepting states
    slv.Add(slv.TransitionConstraint([W[i,j] for j in range(eoh)], # variables
                                     T2, # Transitions
                                     doe, # Initial state
                                     range(doe+1)+range(wdmin, wdmax+1))) # Accepting states

# Enforce the limit on the maximum number of working weekends
weekends = [(7*(h+1)-2, 7*(h+1)-1) for h in range(eoh/7)]
nwe = len(weekends)
Wwe = {(i,h) : slv.IntVar(0, 1, 'We[%d,%d]' % (i, h)) for i in range(ne) for h in range(nwe)}
for i, emp in enumerate(employees):
    # Obtain the value of the Wwe variable
    for h, (j1, j2) in enumerate(weekends):
        slv.Add(Wwe[i,h] == slv.Max(W[i,j1], W[i,j2]))
    # Constrain the sum
    slv.Add(slv.Sum([Wwe[i,h] for h in range(nwe)]) <= emp['maxweekend'])

# Enforce the mandatory days off
for i, emp in enumerate(employees):
    for j in emp['daysoff']:
        slv.Add(W[i,j] == 0)

# Compute the penalty for not satisfying positive preferences
Zon = slv.IntVar(0, sys.maxint, 'Zon')
Zon_wgts = [req['wgt'] for req in onreq]
Zon_vars = [(X[req['emp'],req['day']] != req['shift']) for req in onreq]
slv.Add(Zon == slv.ScalProd(Zon_vars, Zon_wgts))

# Compute the penalty for not satisfying negative preferences
Zoff = slv.IntVar(0, sys.maxint, 'Zoff')
Zoff_wgts = [req['wgt'] for req in offreq]
Zoff_vars = [(X[req['emp'],req['day']] == req['shift']) for req in offreq]
slv.Add(Zoff == slv.ScalProd(Zoff_vars, Zoff_wgts))

# Compute counters for all the required shift types, for each day.
C = []
for cov in cover:
    k = cov['shift']
    j = cov['day']
    C.append(slv.IntVar(0, ne, 'C[%d,%d]' % (k, j)))
    slv.Add(slv.Count([X[i,j] for i in range(ne)], k, C[-1]))
# Use the counters to compute the under and over coverage costs
Zunder = slv.IntVar(0, sys.maxint, 'Zunder')
Zunder_wgt = [cov['wgtu'] for cov in cover]
Zunder_vars = [slv.Max(cov['req'] - C[h], 0) for h, cov in enumerate(cover)]
slv.Add(Zunder == slv.ScalProd(Zunder_vars, Zunder_wgt))
Zover = slv.IntVar(0, sys.maxint, 'Zover')
Zover_wgt = [cov['wgto'] for cov in cover]
Zover_vars = [slv.Max(C[h] - cov['req'], 0) for h, cov in enumerate(cover)]
slv.Add(Zover == slv.ScalProd(Zover_vars, Zover_wgt))
# Zover = slv.IntVar(0, sys.maxint, 'Zover')

# Define the value of the cost variable
slv.Add(Z == Zon + Zoff + Zunder + Zover)

# Streamlining constraints
# for h, cov in enumerate(cover):
#     # slv.Add(Zover_vars[h] == 0)
#     slv.Add(C[h] <= cov['req']+3)
#     slv.Add(C[h] >= cov['req']-3)


# slv.Add(Z == 828)

# DEFINE THE SEARCH STRATEGY
# cover_score = total number of cover requirements for one day
L = sorted(range(eoh), key=lambda j: -sum([c['req'] for c in cover if c['day'] == j]))
# L = range(eoh)
db1 = slv.Phase([W[i,j] for j in L for i in range(ne)],
                slv.CHOOSE_FIRST_UNBOUND, slv.ASSIGN_MAX_VALUE)
db2 = slv.Phase([X[i,j] for j in range(eoh) for i in range(ne)],
                 slv.CHOOSE_FIRST_UNBOUND, slv.ASSIGN_MIN_VALUE)
decision_builder = slv.Compose([db1, db2])

# INIT THE SEARCH PROCESS
time_limit = 60000
# branches_limit = 2000000
search_monitors = [slv.Minimize(Z, 1),
                   slv.TimeLimit(time_limit)]
# init search
slv.NewSearch(decision_builder, search_monitors)

# SEARCH FOR A FEASIBLE SOLUTION
zbest = None
zbest_branches = None
Wdays = {0: 'M', 1: 'T', 2:'W', 3:'T', 4:'F', 5:'S', 6:'S'}
while slv.NextSolution():
    #
    # PRINT SOLUTION
    #
    print '=== Solution found, time: %.3f (sec), branches: %d, cost: %d' % \
                (slv.WallTime()/1000.0, slv.Branches(), Z.Value())

    # Plot the schedule for each employee
    print '%s %s' % (' '*8, ' '.join(Wdays[j%7] for j in range(eoh)))
    for i in range(ne):
        sched = ['-' if X[i,j].Value() == ns else sids[X[i,j].Value()] for j in range(eoh)]
        print '--- %s: %s' % ("{:<3}".format(eids[i]), ' '.join(sched))
    print

    # STORE SOLUTION VALUE
    zbest = Z.Value()
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
print '--- Number of branches: %d' % branches
print '--- Computation time: %.3f (sec)' % (time / 1000.0)
if slv.WallTime() > time_limit:
    print '--- Time limit exceeded'
