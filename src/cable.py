import numpy as np
from typing import Optional, Tuple, List, Set
from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.BRepClass3d import BRepClass3d_SolidClassifier    
from OCC.Core.TopAbs import TopAbs_IN

from src.grids.grids3d import Node, Grids3D
from src.grids_util import CollisionChecker
from src.line.spline import Spline
from src.pathfinding import JumpPointSearch, AstarAlgorithmOp, GridAlgorithm
from src.brep.brep_util import ShapeToMeshConvertor
from src.optimizer.continuous_optimizer import ACOR


class Cable:
    def __init__(self) -> None:
        self.start_terminal: Optional[Node] = None
        self.goal_terminal: Optional[Node] = None
        self.intermidiate_terminals: Optional[Node] = []
        self.spline: Optional[Spline] = None
        self.splines_pnts: List[gp_Pnt] = []
        
    
    def set_start_terminal(self, start_pnt: gp_Pnt, start_vec: Tuple[int, int, int],grids: Grids3D) -> None:
        gap: float = grids[0, 0, 0].gap
        
        i: int = int((start_pnt.X() - grids.corner_min.X()) / gap)  
        j: int = int((start_pnt.Y() - grids.corner_min.Y()) / gap)
        k: int = int((start_pnt.Z() - grids.corner_min.Z()) / gap)
        self.start_terminal = grids[i, j, k]
        self.start_terminal.is_obstacle = False 
        grids.set_start_node(self.start_terminal)   
        self.init_terminal_around_nodes(node=self.start_terminal, grids=grids, vec=start_vec)   
        return  
    
    def set_goal_terminal(self, goal_pos: gp_Pnt, goal_vec: Tuple[int, int, int] ,grids: Grids3D) -> None:
        gap: float = grids[0, 0, 0].gap
        
        i: int = int((goal_pos.X() - grids.corner_min.X()) / gap)  
        j: int = int((goal_pos.Y() - grids.corner_min.Y()) / gap)
        k: int = int((goal_pos.Z() - grids.corner_min.Z()) / gap)
        self.goal_terminal = grids[i, j, k]
        self.goal_terminal.is_obstacle = False  
        grids.set_goal_node(self.goal_terminal)
        self.init_terminal_around_nodes(node=self.goal_terminal, grids=grids, vec=goal_vec)
        return
    
    def set_spline(self, diameter: float, grids: Grids3D) -> None:
        pathfinder: GridAlgorithm = JumpPointSearch(grids = grids)

        if pathfinder.search():
            path_nodes = pathfinder.get_path_nodes()
            path_pnts = [node.center_pnt for node in path_nodes]
            self.spline = Spline(path_pnts=path_pnts, diameter=diameter)
        
        return 
    
    def set_intermediate_pnts(self, pnts: List[gp_Pnt], grids: Grids3D, diameter: float) -> None:   
        pnts.remove(pnts[1])    
        #pnts.remove(pnts[-1])   

        for pnt in pnts:
            gap: float = grids[0, 0, 0].gap
            i: int = int((pnt.X() - grids.corner_min.X()) / gap)  
            j: int = int((pnt.Y() - grids.corner_min.Y()) / gap)
            k: int = int((pnt.Z() - grids.corner_min.Z()) / gap)
            node = grids[i, j, k]
            node.is_obstacle = False
            self.intermidiate_terminals.append(node)    
            
        for idx in range(len(self.intermidiate_terminals) - 1):
            grids.set_start_node(self.intermidiate_terminals[idx])
            grids.set_goal_node(self.intermidiate_terminals[idx + 1])   
            
            pathfinder: GridAlgorithm = JumpPointSearch(grids = grids)  
            if pathfinder.search(): 
                path_nodes = pathfinder.get_smoothed_path_nodes()
                path_pnts = [node.center_pnt for node in path_nodes]
                print(f"length of path node: {len(path_pnts)}")
                for pnt in path_pnts:
                    if pnt not in self.splines_pnts:    
                        self.splines_pnts.append(pnt)
            
        intermidiate_pnts = [node.center_pnt for node in self.intermidiate_terminals]   
        #self.spline = Spline(path_pnts=intermidiate_pnts, diameter=diameter)s
        return
    
    def cable_optimization(self, fused_shape: TopoDS_Shape, grids: Grids3D) -> None:
        self.fused_shape = fused_shape
        self.grids = grids
        acor = ACOR(f = self.esitimate, dim=len(self.spline.intermediate_pnts) * 3, ub = self.spline.diameter * 2, lb=(-self.spline.diameter)*2, max_fes = 2000) 
        sols = acor.optimize()
        gp_sols = [gp_Pnt(sols[i], sols[i * 2], sols[i * 3]) for i in range(len(self.spline.intermediate_pnts))]
        
        self.spline.adjust_intermidiate_pnts(gp_sols)  
        return
    
    def esitimate(self, sols: np.ndarray) -> float:
        gp_sols = [gp_Pnt(sols[i], sols[i * 2], sols[i * 3]) for i in range(len(self.spline.intermediate_pnts))]
        self.spline.adjust_intermidiate_pnts(gp_sols)
        collusion_count = CollisionChecker.check_collision_points_number(grids = self.grids, shape=self.spline.spline_shape, points_number=10000)
        if collusion_count is None:
            return 10000
        return collusion_count
    
    def init_terminal_around_nodes(self, node: Node, grids: Grids3D, vec: Tuple[int, int ,int]) -> None:
        map_size: int = grids.map_size
        cur_node: Node = node
        dir_i, dir_j, dir_k = vec   
        while True:
            i, j, k = cur_node.i, cur_node.j, cur_node.k
            nxt_i, nxt_j, nxt_k = i + dir_i, j + dir_j, k + dir_k
            if nxt_i < 0 or nxt_i >= map_size:
                break   
            if nxt_j < 0 or nxt_j >= map_size:
                break
            if nxt_k < 0 or nxt_k >= map_size:
                break
            
            nxt_node: Node = grids[nxt_i, nxt_j, nxt_k]
            if nxt_node.is_obstacle:
                nxt_node.is_obstacle = False    
            else:
                break
            cur_node = nxt_node 
        return 


class CableBuilder:
    @classmethod
    def get_cable(cls, start_pos: gp_Pnt, goal_pos: gp_Pnt, diameter: float, grids: Grids3D) -> Cable:
        cable = Cable()
        cable.set_start_terminal(start_pnt=start_pos, grids=grids)
        cable.set_goal_terminal(goal_pos=goal_pos, grids=grids)
        cable.set_spline(diameter=diameter, grids=grids)
        return cable    