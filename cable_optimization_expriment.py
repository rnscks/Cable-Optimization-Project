from OCC.Display.SimpleGui import init_display
from OCC.Core.Quantity import (
    Quantity_Color,
    Quantity_NOC_WHITE,
)

from src.cabinet import Cabinet 


cabinet = Cabinet()
print("cabient is created")
display, start_display, _, _ = init_display()   
display.View.SetBgGradientColors(
    Quantity_Color(Quantity_NOC_WHITE),
    Quantity_Color(Quantity_NOC_WHITE),
    2,
    True,
)

display.DisplayShape(cabinet.cabinet_shape, update=True)

for node in cabinet.grids:
    if node.is_obstacle:
        display.DisplayShape(node.get_box_shape(), color="black", transparency=0.9)  

cable = cabinet.cables[0]
display.DisplayShape(cable.start_terminal.get_box_shape(), color="green") 
display.DisplayShape(cable.goal_terminal.get_box_shape(), color="green") 

if cable.spline is not None and cable.spline.spline_shape is not None:   
    display.DisplayShape(cable.spline.spline_shape, color="blue")    
    print("spline is created")  
    
for node in cable.intermidiate_terminals:
    display.DisplayShape(node.get_box_shape(), color="blue")    
for pnt in cable.splines_pnts:
    display.DisplayShape(pnt, color="red")
    
    # cable.cable_optimization(fused_shape=cabinet.cabinet_shape, grids=cabinet.grids)    
    # if cable.spline.spline_shape is not None:
    #     display.DisplayShape(cable.spline.spline_shape, color="blue")    
    # print("cable optimization is done")

# writer = STPFileWriter()
# for node in cabinet.grids:
#     if node.is_obstacle:
#         writer.add_compound(node.get_box_shape())   

start_display()