from OCC.Core.gp import gp_Pnt
from OCC.Display.SimpleGui import init_display

from src.brep.brep_util import STPFileReader
from src.brep.brep_util import ShapeToMeshConvertor

def display_occ_file_read():
    cabinet_shape = STPFileReader.read_stp_file_by_occ('CABINET.step')
    display, start_display, _, _ = init_display()

    display.DisplayShape(cabinet_shape, update=True)    
    start_display()
    return

def display_point_clouds(): 
    cabinet_shape = STPFileReader.read_stp_file_by_occ('CABINET.step')
    point_clouds = ShapeToMeshConvertor.convert_to_point_clouds(cabinet_shape)
    display, start_display, _, _ = init_display()
    
    display.DisplayShape(cabinet_shape)    
    for pnt in point_clouds:
        display.DisplayShape(gp_Pnt(*pnt))
    
    start_display()
    return  


# display_occ_file_read()
display_point_clouds()