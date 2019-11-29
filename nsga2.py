from pymoo.algorithms.nsga2 import NSGA2
from pymoo.factory import get_problem
from pymoo.optimize import minimize
from pymoo.visualization.scatter import Scatter
from pymoo.model.problem import Problem
from GSOptProblem import GSOptProblem

#problem = get_problem("zdt2")
problem = GSOptProblem()

algorithm = NSGA2(pop_size=10, eliminate_duplicates=True)

res = minimize(problem,
               algorithm,
               ('n_gen', 10),
               seed=1,
               verbose=True)

plot = Scatter()
plot.add(problem.pareto_front(), plot_type="line", color="black", alpha=0.7)
plot.add(res.F, color="red")
plot.show()