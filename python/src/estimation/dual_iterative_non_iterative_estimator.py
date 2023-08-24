import numpy as np
from estimation.iterative_estimator import iterative_estimator
from estimation.non_iterative_estimator import non_iterative_estimator


class iter_non_iter_estimator:
    def __init__(self, sensors):

        # Initialize parameters
        self.non_it_est = non_iterative_estimator(sensors)
        self.it_est = iterative_estimator(sensors, [0,0,0])

    def estimate_path(self):
        # First estimate path with non-iterative estimator to get initial guesses
        initial_guess_path = self.non_it_est.estimate_path()
        # Now use initial guesses to estimate actual path
        estimated_path = self.it_est.estimate_path(override_path_init=initial_guess_path)
        
        return estimated_path
