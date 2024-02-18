import os
from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Display.SimpleGui import init_display

from src.grids.grids3d import Grids3D   
from src.grids_util import CollisionChecker, Voxelization   
from src.brep.brep_util import STPFileReader


def display_voxelization():
    cabinet_shape: TopoDS_Shape = STPFileReader.read_stp_file_by_occ("CABINET.step")  
    grids = Grids3D(
        corner_max=gp_Pnt(200, 200, 200),
        corner_min=gp_Pnt(-200, -200, -200), 
        map_size=30
    )   

    Voxelization.voxelize(grids=grids, 
                        shape=cabinet_shape)

    display, start_display, add_menu, add_function_to_menu = init_display() 

    for node in grids:
        if node.is_obstacle:
            display.DisplayShape(node.get_box_shape()) 
            
    display.FitAll()
    start_display()

display_voxelization()