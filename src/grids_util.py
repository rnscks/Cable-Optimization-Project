from typing import List, Optional
from OCC.Core.TopoDS import TopoDS_Shape

from src.brep.brep_util import ShapeToMeshConvertor 
from src.grids.grids3d import Grids3D   

class CollisionChecker:
    @classmethod    
    def check_collision_points_number(cls, grids: Grids3D, shape: TopoDS_Shape, points_number: int = 5000) -> Optional[int]:
        if shape is None:
            print("CollisionChecker.check_collision_points_number: shape is None.")
            return None
        
        gap: float = grids[0, 0, 0].gap
        points_clouds: List[List[float]] = ShapeToMeshConvertor.convert_to_point_clouds(shape, number_of_points=points_number) 
        collision_points: int = 0
        for point in points_clouds:
            i, j, k = point
            i = int((i - grids.corner_min.X()) / gap)
            j = int((j - grids.corner_min.Y()) / gap)  
            k = int((k - grids.corner_min.Z()) / gap)
            
            if i < 0 or j < 0 or k < 0:
                continue
            if i >= grids.map_size or j >= grids.map_size or k >= grids.map_size:
                continue    
            if grids[i, j, k].is_obstacle:
                collision_points += 1
            
        return collision_points
    
    def check_collision(cls, grids: Grids3D, shape: TopoDS_Shape) -> bool:
        if shape is None:
            print("CollisionChecker.check_collision: shape is None.")
            return None
        
        gap: float = grids[0, 0, 0].gap
        points_clouds: List[List[float]] = ShapeToMeshConvertor.convert_to_point_clouds(shape) 
        for point in points_clouds:
            i, j, k = point
            i = int((i - grids.corner_min.X()) / gap)
            j = int((j - grids.corner_min.Y()) / gap)  
            k = int((k - grids.corner_min.Z()) / gap)
            
            if i < 0 or j < 0 or k < 0:
                continue
            if i >= grids.map_size or j >= grids.map_size or k >= grids.map_size:
                continue    
            if grids[i, j, k].is_obstacle:
                return True
        return False    

class Voxelization:
    @classmethod    
    def voxelize(cls, grids: Grids3D, shape: TopoDS_Shape) -> None:
        if shape is None:
            print("Voxelization.voxelize: shape is None.")
            return
        
        gap: float = grids[0, 0, 0].gap
        points_clouds: List[List[float]] = ShapeToMeshConvertor.convert_to_point_clouds(shape) 
        for point in points_clouds:
            i, j, k = point
            i = int((i - grids.corner_min.X()) / gap)
            j = int((j - grids.corner_min.Y()) / gap)  
            k = int((k - grids.corner_min.Z()) / gap)
            
            if i < 0 or j < 0 or k < 0:
                continue
            if i >= grids.map_size or j >= grids.map_size or k >= grids.map_size:
                continue    
            grids[i, j, k].is_obstacle = True
        return
