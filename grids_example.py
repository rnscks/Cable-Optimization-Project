from OCC.Core.gp import gp_Pnt  
from OCC.Display.SimpleGui import init_display  

from src.grids.grids3d import Box, Node, Grids3D


def display_box():
    box = Box()
    box.create_box(gp_Pnt(0, 0, 0), 10)

    display, start_display, _, _ = init_display()

    display.DisplayShape(box.get_box_shape(), update=True)
    start_display()

def display_grids3d():
    display, start_display, _, _ = init_display()
    grids = Grids3D(
        corner_max=gp_Pnt(10, 10, 10),
        corner_min=gp_Pnt(0, 0, 0),
        map_size=10
    )

    grids[0, 0, 0].is_obstacle = True   
    grids[grids.map_size - 1, 
            grids.map_size - 1, 
            grids.map_size - 1].is_obstacle = True

    for node in grids:
        if node.is_obstacle:
            display.DisplayShape(node.get_box_shape(), update=True, color = "red")
    
    grids_shape = grids.get_box_shape()
    display.DisplayShape(grids_shape, update=True, color = "blue", transparency=0.5)  
    start_display()

# display_box()
display_grids3d()