import numpy as np
from abc import ABC, abstractmethod 
from typing import Callable
from multipledispatch import dispatch

class Optimizer(ABC):    
    def __init__(self) -> None:
        super().__init__()
        
    @abstractmethod    
    def optimize(self) -> np.ndarray:
        """_summary_

        Returns:
            np.ndarray: optimized solution
        """
        pass


class ACOR:
    def __init__(self, 
                max_fes: int = 2000,
                m: int = 30,
                k: int = 45, 
                dim: int = 10,
                q: float  = 0.7, 
                zeta: float = 0.8, 
                ub: float = 0.1,
                lb: float = -0.1,
                f: Callable[[np.ndarray], float] = None) -> None:
        # algorithm properties
        self.q: float = q
        self.zeta: float = zeta
        self.dim: int = dim
        self.k: int = k
        self.m: int = m
        # + 2 for the function value and the weight
        self.archive = np.zeros((self.k, self.dim + 2))
        self.max_fes = max_fes
        self.ub: float = ub
        self.lb: float = lb
        self.f: Callable[[np.ndarray], float] = f
        
        # initialize the archive
        self.archive[:, :self.dim]  = np.random.uniform(lb, ub, (self.k, self.dim))
        for i in range(self.k): 
            self._estimate(i)    
            self.cal_weight(i)
        return

    def optimize(self) -> np.ndarray:
        # fes: function evaluations
        fes = 0
        
        while fes <= self.max_fes:
            print(f"number of fes: {fes}")
            print(f"fes: {self.archive[0, self.dim]}")
            if self.archive[0, self.dim] == 0.0:
                print(self.archive)
                return self.archive[0, :self.dim]
            
            # initialize new population
            init_pop = np.zeros((self.m, self.dim + 2)) 
            for i in range(self.m):
                new_sols = np.zeros((self.dim + 2)) 
                # select guiding solution
                guiding_sol_idx = self.get_guiding_solution_index() 
                # generate new population
                new_sols[:self.dim] = self.generate_solution(guiding_sol_idx)
                # estimate the new population
                f = self._estimate(new_sols) 
                # update population
                new_sols[self.dim] = f
                init_pop[i, :] = new_sols
                fes += 1
            # concatenate the archive and the new population
            self.archive = np.vstack((self.archive, new_sols)) 
            # sort the archive by evaluation value
            self.archive = self.archive[self.archive[:, self.dim].argsort()]   
            # removal of the worst solutions    
            self.archive = self.archive[:self.k, :] 
            # update weight
            for i in range(self.k):
                self.cal_weight(i)  
        
        return self.archive[0, :self.dim]
    
    @dispatch(int)
    def _estimate(self, i: int) -> None:
        sols = self.archive[i, :self.dim]           
        self.archive[i, self.dim] = self.f(sols)
        return
    
    @dispatch(np.ndarray)
    def _estimate(self, sols: np.ndarray) -> None:
        return self.f(sols)
    
    def cal_weight(self, i: int) -> None:
        numerator: float = np.exp(-(np.square(i - 1)/(2 * np.square(self.q) * np.square(self.k))))  
        denominator: float = self.q * self.k * np.sqrt(2 * np.pi)
        
        self.archive[i, self.dim + 1] = numerator / denominator
        return

    def get_guiding_solution_index(self) -> int:   
        p: float = self.archive[:, self.dim + 1] / np.sum(self.archive[:, self.dim + 1])
        i: int =  np.random.choice(self.k, p=p)
        return i
    
    def cal_deviation(self, l: int) -> None:    
        deviation: float = 0
        deviation = np.sum(np.abs(self.archive - self.archive[l])) * (self.zeta * ((self.k - 1) ** -1))    
        
        return deviation

    def generate_solution(self, l: int) -> np.ndarray:    
        new_sols: np.ndarray = np.zeros((self.dim)) 
        
        for j in range(self.dim):
            mu: float = self.archive[l, j]
            div: float = self.cal_deviation(l)
            sol = np.random.normal(mu, div)
            clipped_sol = np.clip(sol, self.lb, self.ub)    
            new_sols[j] = clipped_sol
        
        return new_sols
