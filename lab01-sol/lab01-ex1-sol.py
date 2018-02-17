#
# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
#
from ortools.constraint_solver import pywrapcp

#
# PROBLEM DATA
#

regions = [
    {'name' : 'Abruzzo',               'borders': [9, 10, 6]},
    {'name' : 'Basilicata',            'borders': [12, 2, 3]},
    {'name' : 'Calabria',              'borders': [1]},
    {'name' : 'Campania',              'borders': [6, 10, 12, 1]},
    {'name' : 'Emilia-Romagna',        'borders': [8, 19, 9, 15, 7, 11]},
    {'name' : 'Friuli-Venezia Giulia', 'borders': [19]},
    {'name' : 'Lazio',                 'borders': [15, 17, 9, 0, 10, 3]},
    {'name' : 'Liguria',               'borders': [11, 4, 15]},
    {'name' : 'Lombardia',             'borders': [16, 19, 4, 11]},
    {'name' : 'Marche',                'borders': [4, 0, 6, 17, 4, 15]},
    {'name' : 'Molise',                'borders': [0, 12, 3, 6]},
    {'name' : 'Piemonte',              'borders': [18, 8, 4, 7]},
    {'name' : 'Puglia',                'borders': [10, 1, 3]},
    {'name' : 'Sardegna',              'borders': []},
    {'name' : 'Sicilia',               'borders': []},
    {'name' : 'Toscana',               'borders': [4, 17, 9, 6, 7]},
    {'name' : 'Trentino-Alto Adige',   'borders': [19, 8]},
    {'name' : 'Umbria',                'borders': [15, 9, 6]},
    {'name' : "Valle d'Aosta",         'borders': [11]},
    {'name' : 'Veneto',                'borders': [16, 5, 4, 8]}
]

#
# CREATE A SOLVER INSTANCE
# Signature: Solver(<solver name>)
# 
#
slv = pywrapcp.Solver('map-coloring')

#
# CREATE VARIABLES
# Signature: IntVar(<min>, <max>, <name>)
#

n = 4
x = [slv.IntVar(0,n-1, r['name']) for r in regions]

#
# BUILD CONSTRAINTS AND ADD THEM TO THE MODEL
# Signature: Add(<constraint>)
#

for i, r in enumerate(regions):
    for j in r['borders']:
        slv.Add(x[i] != x[j])

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
slv.NewSearch(decision_builder)

#
# Enumerate all solutions!
#
nsol = 0
while slv.NextSolution():
    # Here, a solution has just been found. Hence, all variables are bound
    # and their "Value()" method can be called without errors.
    # The notation %2d prints an integer using always two "spaces"

    # ------- DO THIS YOURSELVES! -------
    print x

    # WE WANT A SINGLE SOLUTION
    nsol += 1
    break

if nsol == 0:
    print 'no solution found'

#
# END THE SEARCH PROCESS
#
slv.EndSearch()