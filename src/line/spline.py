from scipy.spatial import KDTree

from typing import List, Optional
import numpy as np
from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Core.TopoDS import TopoDS_Shape    
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeWire
from OCC.Core.gp import gp_Circ, gp_Ax2, gp_Dir
from OCC.Core.TopoDS import TopoDS_Edge, TopoDS_Wire
from OCC.Core.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCC.Core.TColgp import TColgp_HArray1OfPnt, TColgp_Array1OfVec
from OCC.Core.TColStd import TColStd_HArray1OfBoolean
from OCC.Core.GeomAPI import GeomAPI_Interpolate
from OCC.Core.TopoDS import TopoDS_Shell



class Spline:
    def __init__(self, path_pnts: List[gp_Pnt], diameter: float = 0.1) -> None:
        self.start_pnt: gp_Pnt = path_pnts[0]    
        self.end_pnt: gp_Pnt = path_pnts[-1] 
        self.intermediate_pnts: List[gp_Pnt] = path_pnts[1:-1]
        self.diameter: float = diameter 
        self.spline_shape: Optional[TopoDS_Shape] = None
        self.update_spline_shape()    
        
        
    def set_intermediate_pnts(self, intermediate_pnts: List[gp_Pnt]) -> None:
        """
        Args:
            intermediate_pnts (List[gp_Pnt]): new intermediate points
        description:
            set new intermediate points and update spline shape
        """
        self.intermediate_pnts = intermediate_pnts
        self.update_spline_shape()
        return
    
    def adjust_intermidiate_pnts(self, adjust_pnts: List[gp_Pnt]) -> None:    
        """
        Args:
            adjust_pnts (List[gp_Pnt]): adjust intermediate points
        description: 
            adjust intermediate points by adjust pnts list 
            in  dx, dy, dz and update spline shape
        """
        for i in range(len(adjust_pnts)):
            if not isinstance(adjust_pnts[i], gp_Pnt):
                raise ValueError("path pnts in gp_Pnt is not an instance of gp_Pnt")    
            self.intermediate_pnts[i] = gp_Pnt(adjust_pnts[i].X() + self.intermediate_pnts[i].X(), 
                                                adjust_pnts[i].Y() + self.intermediate_pnts[i].Y(), 
                                                adjust_pnts[i].Z() + self.intermediate_pnts[i].Z())   

        
        self.update_spline_shape()
        return  
    
    def update_spline_shape(self) -> None:
        """
        Raises:
            ValueError: path pnts in gp_Pnt is not an instance of gp_Pnt 
        
        description:
            update spline shape by 
            path pnts(start_pnt + intermidiate_pnts + end_pnt) and diameter
        """
        if self.spline_shape is not None:
            del self.spline_shape
        
        path_pnts = [self.start_pnt] + self.intermediate_pnts + [self.end_pnt]

        
        self.spline_shape = SplineBuilder.get_spline_shape(path_pnts, self.diameter)
        return
    
class SplineBuilder:    
    @classmethod    
    def remove_duplicate_points(cls, pnts: gp_Pnt) -> List[gp_Pnt]:    
        unique_pnts = set()
        not_duplicate_pnts: List[gp_Pnt] = [] 
        for pnt in pnts:
            if (pnt.X(), pnt.Y(), pnt.Z()) in unique_pnts:  
                continue
            else:
                not_duplicate_pnts.append(pnt)  
                unique_pnts.add((pnt.X(), pnt.Y(), pnt.Z()))    
        return not_duplicate_pnts 
    
    @classmethod
    def optimize_path_using_previous_point(cls, pnts: gp_Pnt, threshold=0.001)->List[gp_Pnt]:
        pnts = cls.remove_duplicate_points(pnts)    
        pnts = [(pnt.X(), pnt.Y(), pnt.Z()) for pnt in pnts]

        tree = KDTree(pnts)  
        optimized_pnts: gp_Pnt = []
        skipped_indices = set()

        for i, pnt in enumerate(pnts):
            if i in skipped_indices:
                continue        
            optimized_pnts.append(gp_Pnt(*pnt)) 
            distances, indices = tree.query(pnt, k=2) 
            for dist, idx in zip(distances[1:], indices[1:]):
                if dist < threshold:
                    skipped_indices.add(idx)

        return optimized_pnts
    
    @classmethod    
    def get_spline_shape(cls, pnt_list: List[gp_Pnt], diameter: float) -> Optional[TopoDS_Shell]:
        pnt_list = cls.remove_duplicate_points(pnts=pnt_list)

        if len(pnt_list) < 2:   
            print(ValueError("Spline: pnt_list should have at least 2 points."))
            return None
        
        if len(pnt_list) == 2:
            cls.add_middle_pnt(pnt_list)    
        
        tcol_pnt = TColgp_HArray1OfPnt(1, len(pnt_list))
        tcol_vec = TColgp_Array1OfVec(1, len(pnt_list))
        tcol_std_bool = TColStd_HArray1OfBoolean(1, len(pnt_list))
        
        for i in range(1, len(pnt_list) + 1):
            tcol_pnt.SetValue(i, pnt_list[i - 1])
            
            if (len(pnt_list) == i):
                continue
            tcol_vec.SetValue(i, gp_Vec(pnt_list[i - 1], pnt_list[i]))
            tcol_std_bool.SetValue(i, True)
        
        end_gradient_vec = gp_Vec(pnt_list[len(pnt_list) - 1], pnt_list[len(pnt_list) - 2])
        tcol_vec.SetValue(len(pnt_list), end_gradient_vec)

        tcol_std_bool.SetValue(1, True)
        tcol_std_bool.SetValue(len(pnt_list), False)
        tolenrance = 1e-3
        interpolate = GeomAPI_Interpolate(tcol_pnt, False, tolenrance)

        interpolate.Load(tcol_vec, tcol_std_bool)
        interpolate.Perform()
        try:
            if (interpolate.IsDone()):
                curve_edge = interpolate.Curve()
                edge_shape = BRepBuilderAPI_MakeEdge(curve_edge).Edge()

                wire_shape = BRepBuilderAPI_MakeWire(edge_shape).Wire()
                circle: gp_Circ = gp_Circ(gp_Ax2(pnt_list[0], gp_Dir(pnt_list[1].XYZ().Subtracted(pnt_list[0].XYZ()))), diameter)
                circle_edge: TopoDS_Edge = BRepBuilderAPI_MakeEdge(circle).Edge()
                circle_wire: TopoDS_Wire = BRepBuilderAPI_MakeWire(circle_edge).Wire()

                return BRepOffsetAPI_MakePipe(wire_shape, circle_wire).Shape()
        except RuntimeError:
            print("Spline Builder: RuntimeError[Spline Shape]")
            return
    
    @classmethod
    def get_pipe_shape(cls, pnt_list: List[gp_Pnt], diameter: float) -> TopoDS_Shell:
        """

        Args:
            pnt_list (List[gp_Pnt]): _description_
            diameter (float): _description_

        Returns:
            TopoDS_Shell: _description_
        description:    
            Spline Points가 2개일 경우 Pipe Shape을 반환    
        """
        try:
            pnt1 = pnt_list[0]  
            pnt2 = pnt_list[-1]

            edge = BRepBuilderAPI_MakeEdge(pnt1, pnt2).Edge()
            wire = BRepBuilderAPI_MakeWire(edge).Wire()

            circle_ax2 = gp_Ax2(pnt1, gp_Dir(pnt1.XYZ().Subtracted(pnt2.XYZ())))
            circle = gp_Circ(circle_ax2, diameter)
            
            circle_edge = BRepBuilderAPI_MakeEdge(circle).Edge()
            circle_wire = BRepBuilderAPI_MakeWire(circle_edge).Wire()
            
            return BRepOffsetAPI_MakePipe(wire, circle_wire).Shape()
        except RuntimeError:
            print("Spline Builder: RuntimeError[Pipe Shape]")
            return  
        
    @classmethod
    def add_middle_pnt(cls, pnt_list: List[gp_Pnt]) -> None:
        start_pnt = pnt_list[0]
        goal_pnt = pnt_list[-1]
        
        middle_pnt = gp_Pnt((start_pnt.X() + goal_pnt.X()) * 0.4,    
                            (start_pnt.Y() + goal_pnt.Y()) * 0.3,    
                            (start_pnt.Z() + goal_pnt.Z()) * 0.5)   
        
        pnt_list.insert(1, middle_pnt)  
        return