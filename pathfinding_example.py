from OCC.Display.SimpleGui import init_display  
from OCC.Core.Quantity import Quantity_Color, Quantity_NOC_WHITE
from OCC.Display.SimpleGui import init_display

from src.cabinet import Cabinet
from src.pathfinding import JumpPointSearch, AstarAlgorithmOp, GridAlgorithm, JumpPointSearchTheta
from src.line.spline import Spline

cabinet = Cabinet("cabinet_grid_map.npy")
grids = cabinet.grids   
map_size = grids.map_size   
print(cabinet.__doc__)
start_node = grids[10, 25, 25]
goal_node = grids[15, 10, 29]        
grids.set_start_node(start_node)    
grids.set_goal_node(goal_node)  

jps: GridAlgorithm = JumpPointSearchTheta(cabinet.grids)
display, start_display, _, _ = init_display()
display.View.SetBgGradientColors(
    Quantity_Color(Quantity_NOC_WHITE),
    Quantity_Color(Quantity_NOC_WHITE),
    2,
    True,
)

display.DisplayShape(start_node.get_box_shape(), color="green")
display.DisplayShape(goal_node.get_box_shape(), color="green")    
if jps.search():
    path_nodes = jps.get_path_nodes()
    # path_nodes = jps.get_smoothed_path_nodes()
    path_pnts =  []
    for node in path_nodes:
        display.DisplayShape(node.get_box_shape(), color="blue")
        path_pnts.append(node.center_pnt) 
    spline = Spline(path_pnts=path_pnts, diameter=1)
    
    if spline.spline_shape is not None:
        display.DisplayShape(spline.spline_shape, color="blue")  
else:
    print("No path found")

for node in cabinet.grids:
    if node.is_obstacle:
        display.DisplayShape(node.get_box_shape(), color="black")

start_display()