import numpy as np  
from OCC.Core.gp import gp_Pnt  

from src.optimizer import Optimizer, ACOR
from src.cable.line.spline import Spline    

class SplineOptimizer:
    def __init__(self, spline: Spline, optimizer: Optimizer = None) -> None:
        self.spline: Spline = spline
        self.optimizer =  optimizer
        if optimizer is None:
            self.optimizer = ACOR(dim = 3 * len(spline.intermediate_pnts),
                                    f = self.evaluate)
        self.optimize_spline_shape()
        
    def optimize_spline_shape(self) -> None:
        self.optimizer.optimize()
        return      
    
    def evaluate(self, sols: np.ndarray) -> int:
        adjust_pnts = [gp_Pnt(*sols[i:i+3]) for i in range(0, len(sols), 3)]
        self.spline.adjust_intermidiate_pnts(adjust_pnts)
        
        
        return 100
