from typing import List
from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display  

from src.brep.brep_util import STPFileReader
from src.grids_util import Voxelization
from src.grids.grids3d import Grids3D, Node
from src.pathfinding import JumpPointSearch
from src.line.spline import Spline

def display_pathfinding():
    cabinet_shape = STPFileReader.read_stp_file_by_occ("CABINET.step")  
    grids = Grids3D(corner_max=gp_Pnt(200, 200, 200),
            corner_min=gp_Pnt(-200, -200, -200),
            map_size=30)
    Voxelization.voxelize(grids=grids, shape=cabinet_shape) 

    map_size: int = grids.map_size  
    grids.set_goal_node(grids[map_size - 1, map_size - 1, map_size - 1])
    grids.set_start_node(grids[0, 0, 0])    
    jps = JumpPointSearch(grids=grids)
    
    path_pnts: List[gp_Pnt] = []
    if jps.search2():
        path_nodes: List[Node] = jps.get_path_nodes()
        
        for node in path_nodes:
            path_pnts.append(node.center_pnt)
            

    display, start_display, _, _ = init_display()
    for node in grids:
        if node.is_obstacle:
            display.DisplayShape(node.get_box_shape())  
        if node.is_start_node:
            display.DisplayShape(node.get_box_shape(), color="green")
        if node.is_goal_node:
            display.DisplayShape(node.get_box_shape(), color="blue")    
    
    for node in path_nodes:
        display.DisplayShape(node.get_box_shape(), color="red") 
    spline = Spline(path_pnts=path_pnts, diameter=1)
    if spline.spline_shape:
        display.DisplayShape(spline.spline_shape, color="red")
    start_display()

display_pathfinding()