import autograd.numpy as anp
import numpy as np
from pymoo.util.misc import stack
from pymoo.model.problem import Problem

class GSOptProblem(Problem):
	def __init__(self, n_var, n_obj, n_constr, xl, xu, simulation_function):
		super().__init__(n_var=n_var, n_obj=n_obj, n_constr=n_constr, xl=xl, xu=xu)  #xl/xu: lower/upperboundaries
		self.simulation_function = simulation_function

	def _evaluate(self, x, out, *args, **kwargs):
		# x = array with n rows and m columns as an input. 
		#Each row represents an individual and each column an optimization variable
		print("evaluate")
		
		#objective functions. Depends on n of var
		#f1 = x[:,0]**2 + x[:,1]**2
		f1 = self.simulation_function(x)
		#f2 = (x[:,0]-1)**2 + x[:,1]**2

		#constraints
		#g1 = 2*(x[:, 0]-0.1) * (x[:, 0]-0.9) / 0.18
		#g2 = - 20*(x[:, 0]-0.4) * (x[:, 0]-0.6) / 4.8

		#out is the dictionary to add results with the key F
		#and the constraints with key G.
		
		#out["F"] = anp.column_stack([f1, f2]) 
		#out["G"] = anp.column_stack([g1, g2])

		out["F"] = anp.column_stack([f1]) 
		#out["G"] = anp.column_stack([g1])
		
    
    # --------------------------------------------------
    # Pareto-front - not necessary but used for plotting
    # --------------------------------------------------
	def _calc_pareto_front(self, flatten=True, **kwargs):
		f1_a = np.linspace(0.1**2, 0.4**2, 100)
		#f2_a = (np.sqrt(f1_a) - 1)**2

		f1_b = np.linspace(0.6**2, 0.9**2, 100)
		#f2_b = (np.sqrt(f1_b) - 1)**2

		#a, b = np.column_stack([f1_a, f2_a]), np.column_stack([f1_b, f2_b])
		a, b = np.column_stack([f1_a]), np.column_stack([f1_b])
		return stack(a, b, flatten=flatten)

    # --------------------------------------------------
    # Pareto-set - not necessary but used for plotting
    # --------------------------------------------------
	def _calc_pareto_set(self, flatten=True, **kwargs):
		x1_a = np.linspace(0.1, 0.4, 50)
		x1_b = np.linspace(0.6, 0.9, 50)
		x2 = np.zeros(50)

		a, b = np.column_stack([x1_a, x2]), np.column_stack([x1_b, x2])
		return stack(a,b, flatten=flatten)

