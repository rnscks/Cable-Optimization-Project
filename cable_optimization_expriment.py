from OCC.Display.SimpleGui import init_display
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse   

from src.cabinet import Cabinet 
from src.brep.brep_util import STPFileWriter


cabinet = Cabinet()

display, start_display, _, _ = init_display()   
#display.DisplayShape(cabinet.cabinet_shape, update=True)
for node in cabinet.grids:
    if node.is_obstacle:
        display.DisplayShape(node.get_box_shape(), color="black", transparency=0)  

for cable in cabinet.cables:    

    display.DisplayShape(cable.start_terminal.get_box_shape(), color="green") 
    display.DisplayShape(cable.goal_terminal.get_box_shape(), color="green") 

    if cable.spline is not None and cable.spline.spline_shape is not None:
        display.DisplayShape(cable.spline.spline_shape, color="red") 
    cable.cable_optimization(fused_shape=cabinet.cabinet_shape, grids=cabinet.grids)    
    if cable.spline.spline_shape is not None:
        display.DisplayShape(cable.spline.spline_shape, color="blue")    
    print("cable optimization is done")
    
writer = STPFileWriter()
for node in cabinet.grids:
    if node.is_obstacle:
        writer.add_compound(node.get_box_shape())   

for cable in cabinet.cables:    
    if cable.spline is not None and cable.spline.spline_shape is not None:  
        writer.add_compound(cable.spline.spline_shape)

writer.save()

start_display()