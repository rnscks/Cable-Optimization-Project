import os
import sys
# path 조정
project_path = os.path.dirname(os.path.abspath(__file__))
project_path = project_path.split(os.sep)  
project_path = os.sep.join(project_path[:-1])   
sys.path.append(os.path.abspath(project_path))

import random
from OCC.Core.gp import gp_Pnt  
from OCC.Display.SimpleGui import init_display

from src.line.spline import Spline  
from OCC.Core.Quantity import Quantity_Color, Quantity_NOC_WHITE

def generate_random_path(num_points: int, coordinate_range: tuple) -> list:
    min_coord, max_coord = coordinate_range
    path = [
        gp_Pnt(random.uniform(min_coord, max_coord),
                random.uniform(min_coord, max_coord),
                random.uniform(min_coord, max_coord))
        for _ in range(num_points)
    ]
    return path

coordinate_range = (-10, 10) 
num_points = 10
splines = []

for _ in range(100):
    random_path = generate_random_path(num_points=num_points, coordinate_range=coordinate_range)
    spline = Spline(
        path_pnts=random_path,
        diameter=0.1
    )
    splines.append(spline)

display, start_display, _, _ = init_display()
display.View.SetBgGradientColors(
    Quantity_Color(Quantity_NOC_WHITE),
    Quantity_Color(Quantity_NOC_WHITE),
    2,
    True,
)
err_counts = 0
for spline in splines:
    if spline.spline_shape:
        display.DisplayShape(spline.spline_shape, color="blue") 
    else:
        err_counts += 1 
        
print(f"Error counts: {err_counts}")    
display.FitAll()
start_display()