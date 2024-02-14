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
    def get_spline_shape(self, pnt_list: List[gp_Pnt], diameter: float) -> None:
        tcol_pnt = TColgp_HArray1OfPnt(1, len(pnt_list))
        tcol_vec = TColgp_Array1OfVec(1, len(pnt_list))
        tcol_std_bool = TColStd_HArray1OfBoolean(1, len(pnt_list))
        for i in range(1, len(pnt_list) + 1):
            tcol_pnt.SetValue(i, pnt_list[i - 1])
            if (len(pnt_list) == i):
                continue

            tcol_vec.SetValue(
                i, gp_Vec(pnt_list[i - 1], pnt_list[i]))
            tcol_std_bool.SetValue(i, True)
        end_gradient_vec = gp_Vec(pnt_list[len(pnt_list) - 1], pnt_list[len(pnt_list) - 2])
        tcol_vec.SetValue(len(pnt_list), end_gradient_vec)

        tcol_std_bool.SetValue(1, True)
        tcol_std_bool.SetValue(len(pnt_list), False)
        tolenrance = 1e-3
        interpolate = GeomAPI_Interpolate(tcol_pnt, False, tolenrance)

        interpolate.Load(tcol_vec, tcol_std_bool)
        interpolate.Perform()

        if (interpolate.IsDone()):
            curve_edge = interpolate.Curve()
            edge_shape = BRepBuilderAPI_MakeEdge(curve_edge).Edge()

            wire_shape = BRepBuilderAPI_MakeWire(edge_shape).Wire()
            circle: gp_Circ = gp_Circ(gp_Ax2(pnt_list[0], gp_Dir(pnt_list[1].XYZ().Subtracted(pnt_list[0].XYZ()))), diameter)
            circle_edge: TopoDS_Edge = BRepBuilderAPI_MakeEdge(circle).Edge()
            circle_wire: TopoDS_Wire = BRepBuilderAPI_MakeWire(circle_edge).Wire()
            try:
                spline_shape = BRepOffsetAPI_MakePipe(wire_shape, circle_wire).Shape()
                return spline_shape

            except RuntimeError:
                print("Spline Builder: RuntimeError")
                return None



