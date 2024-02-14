from OCC.Core.gp import gp_Pnt  
from OCC.Display.SimpleGui import init_display

from src.line.spline import Spline

def display_spline():
    spline = Spline(path_pnts = [gp_Pnt(0, 0, 10), 
                        gp_Pnt(0, 10, 10), 
                        gp_Pnt(0, 10, 15), 
                        gp_Pnt(10, 10, 20)], 
                    diameter = 0.1)

    display,start_display, _ ,_ = init_display()
    display.DisplayShape(spline.spline_shape)
    display.FitAll()
    start_display()

display_spline()