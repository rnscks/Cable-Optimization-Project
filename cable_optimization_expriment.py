from OCC.Display.SimpleGui import init_display
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_NOC_WHITE,
)

from src.cabinet import Cabinet 


cabinet = Cabinet("cabinet_grid_map.npy")
print("cabient is created")
display, start_display, _, _ = init_display()   
display.View.SetBgGradientColors(
    Quantity_Color(Quantity_NOC_WHITE),
    Quantity_Color(Quantity_NOC_WHITE),
    2,
    True,
)


for node in cabinet.grids:
    if node.is_obstacle:
        display.DisplayShape(node.get_box_shape(), color="black", transparency=0.9)  

cables = cabinet.cables
for cable in cables:
    display.DisplayShape(cable.start_terminal.get_box_shape(), color="green") 
    display.DisplayShape(cable.goal_terminal.get_box_shape(), color="green") 
    for node in cable.intermidiate_terminals:
        display.DisplayShape(node.get_box_shape(), color="blue")   

    if cable.spline is not None and cable.spline.spline_shape is not None:   
        display.DisplayShape(cable.spline.spline_shape, color="blue")    
        print("spline is created")  


start_display()