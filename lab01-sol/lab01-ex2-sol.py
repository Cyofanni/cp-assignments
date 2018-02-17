"""
Model & Solve the zebra problem as invented by Lewis Caroll:

There are five houses.
The Englishman lives in the red house.
The Spaniard owns the dog.
Coffee is drunk in the green house.
The Ukrainian drinks tea.
The green house is immediately to the right of the ivory house.
The Old Gold smoker owns snails.
Kools are smoked in the yellow house.
Milk is drunk in the middle house.
The Norwegian lives in the first house.
The man who smokes Chesterfields lives in the house next to the man
   with the fox.
Kools are smoked in the house next to the house where the horse is kept.
The Lucky Strike smoker drinks orange juice.
The Japanese smokes Parliaments.
The Norwegian lives next to the blue house.

Who owns a zebra and who drinks water?
"""

#
# IMPORT THE OR-TOOLS CONSTRAINT SOLVER
#
from ortools.constraint_solver import pywrapcp

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

houses = range(5) # this is my domain
eng, spa, ukr, nor, jap = [slv.IntVar(houses, 'n%d' % i) for i in houses]
red, green, ivory, yellow, blue = [slv.IntVar(houses, 'c%d' % i) for i in houses]
dog, snails, fox, horse, zebra = [slv.IntVar(houses, 'a%d' % i) for i in houses]
coffee, tea, milk, orange, water = [slv.IntVar(houses,  'd%d' % i) for i in houses]
oldgold, kools, chesterfields, luckystrike, parliaments = [slv.IntVar(houses, 's%d' % i) for i in houses]

# ------- DO THIS YOURSELVES! -------

#
# BUILD CONSTRAINTS AND ADD THEM TO THE MODEL
# Signature: Add(<constraint>)
#

# ------- DO THIS YOURSELVES! -------

slv.Add(eng == red)
slv.Add(spa == dog)
slv.Add(coffee == green)
slv.Add(ukr == tea)
slv.Add(green == ivory+1)
slv.Add(oldgold == snails)
slv.Add(kools == yellow)
slv.Add(milk == 2)
slv.Add(nor == 0)
slv.Add(abs(chesterfields - fox) == 1)
slv.Add(abs(kools - horse) == 1)
slv.Add(luckystrike == orange)
slv.Add(abs(nor - blue) == 1)

# Implied difference constraints
for i, x in enumerate([eng, spa, ukr, nor, jap]):
    for j, y in enumerate([eng, spa, ukr, nor, jap]):
        if i != j: slv.Add(x != y)

for i, x in enumerate([red, green, ivory, yellow, blue]):
    for j, y in enumerate([red, green, ivory, yellow, blue]):
        if i != j: slv.Add(x != y)

for i, x in enumerate([dog, snails, fox, horse, zebra]):
    for j, y in enumerate([dog, snails, fox, horse, zebra]):
        if i != j: slv.Add(x != y)

for i, x in enumerate([coffee, tea, milk, orange, water]):
    for j, y in enumerate([coffee, tea, milk, orange, water]):
        if i != j: slv.Add(x != y)

for i, x in enumerate([oldgold, kools, chesterfields, luckystrike, parliaments]):
    for j, y in enumerate([oldgold, kools, chesterfields, luckystrike, parliaments]):
        if i != j: slv.Add(x != y)


#
# THOSE ARE THE VARIABLES THAT WE WANT TO USE FOR BRANCHING
#
all_vars = [
    eng, spa, ukr, nor, jap,
    red, green, ivory, yellow, blue,
    dog, snails, fox, horse, zebra,
    coffee, tea, milk, orange, water,
    oldgold, kools, chesterfields, luckystrike, parliaments    
]

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
    print 'The Englishman is in house %d' % eng.Value()
    print 'The Spaniard is in house %d' % spa.Value()
    print 'The Ukranian is in house %d' % ukr.Value()
    print 'The Norwegian is in house %d' % nor.Value()
    print 'The Japanese is in house %d' % jap.Value()
    print 'Water is drinked in house %d' % water.Value()
    print 'The zebra is in house %d' % zebra.Value()

    # WE WANT A SINGLE SOLUTION
    nsol += 1
    break

if nsol == 0:
    print 'no solution found'

#
# END THE SEARCH PROCESS
#
slv.EndSearch()