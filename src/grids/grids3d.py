from multipledispatch import dispatch
from typing import Optional
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox    
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.gp import gp_Pnt


class Box:  
    # initialize by creator pattern
    def __init__(self) -> None:
        super().__init__()
        # for geometry
        self.center_pnt: Optional[gp_Pnt] = None
        self.corner_min: Optional[gp_Pnt] = None
        self.corner_max: Optional[gp_Pnt] = None
        self.gap: Optional[float] = None   
        # for display
        self.color: str = "black"
    
    
    # creator pattern   
    # initialize geometry properties
    @dispatch(gp_Pnt, int)
    def create_box(self, center_pnt: gp_Pnt, gap: int) -> None:
        self.center_pnt = center_pnt
        self.gap = gap
        self.corner_min = self.get_corner_min(center_pnt, gap)
        self.corner_max = self.get_corner_max(center_pnt, gap)  
        return
    
    
    @dispatch(gp_Pnt, float)
    def create_box(self, center_pnt: gp_Pnt, gap: float) -> None:
        self.center_pnt = center_pnt
        self.gap = gap
        self.corner_min = self.get_corner_min(center_pnt, gap)
        self.corner_max = self.get_corner_max(center_pnt, gap)  
        return
    
    @dispatch(gp_Pnt, gp_Pnt)
    def create_box(self, corner_min: gp_Pnt, corner_max: gp_Pnt) -> None:         
        self.corner_min = corner_min
        self.corner_max = corner_max
        self.center_pnt = self.get_center_pnt(corner_min, corner_max)  
        self.gap = self.get_gap(corner_min, corner_max)     
        return
    
    # lazy initialization
    def get_box_shape(self) -> TopoDS_Shape:
        corner_min = self.corner_min
        corner_max = self.corner_max
        return BRepPrimAPI_MakeBox(corner_min, corner_max).Shape()
    # for deep copy
    def copy_geom_properties_from(self, other: 'Box') -> None:
        if not isinstance(other, Box):
            raise ValueError("Other must be an instance of Box")
        self.corner_min = gp_Pnt(*other.corner_min.Coord())  
        self.corner_max = gp_Pnt(*other.corner_max.Coord()) 
        self.shape = BRepPrimAPI_MakeBox(self.corner_min, self.corner_max).Shape()    
        self.gap = other.gap
        return
    
    def get_corner_min(self, center_pnt: gp_Pnt, gap: float) -> gp_Pnt:
        x, y, z = center_pnt.XYZ().Coord()
        
        corner_x_min = x - gap / 2 
        corner_y_min = y - gap / 2 
        corner_z_min = z - gap / 2 
        return gp_Pnt(corner_x_min, corner_y_min, corner_z_min)
    
    def get_corner_max(self, center_pnt: gp_Pnt, gap: float) -> gp_Pnt:
        x, y, z = center_pnt.XYZ().Coord()
        
        corner_x_max = x + gap / 2 
        corner_y_max = y + gap / 2 
        corner_z_max = z + gap / 2 
        return gp_Pnt(corner_x_max, corner_y_max, corner_z_max)  
    
    def get_center_pnt(self, corner_min: gp_Pnt, corner_max: gp_Pnt) -> gp_Pnt:
        corner_min_x, corner_min_y, corner_min_z = corner_min.XYZ().Coord()  
        corner_max_x, corner_max_y, corner_max_z = corner_max.XYZ().Coord()  
        
        x = (corner_min_x + corner_max_x) / 2
        y = (corner_min_y + corner_max_y) / 2
        z = (corner_min_z + corner_max_z) / 2
        return gp_Pnt(x, y, z)
    
    def get_gap(self, corner_min: gp_Pnt, corner_max: gp_Pnt) -> float:
        gap:float = corner_max.X() - corner_min.X()
        return gap

class Node(Box):
    # geometry properties are initialized in grids3d initialization step
    def __init__(self, i: int = 0, j: int = 0, k: int = 0) -> None:
        super().__init__()
        self.i: int = i
        self.j: int = j
        self.k: int = k
        
        self.f: float = 0.0
        self.g: float = 0.0
        
        self.parent: Optional[Node] = None 
        self.is_start_node: bool = False   
        self.is_goal_node: bool = False
        self.is_obstacle: bool = False
    
    def copy_from(self, other: "Node") -> None:
        self.i, self.j, self.k = other.i, other.j, other.k
        self.f, self.g = other.f, other.g
        self.parent = other.parent
        self.is_start_node = other.is_start_node
        self.is_goal_node = other.is_goal_node
        self.is_obstacle = other.is_obstacle
        
        if isinstance(other.gap, float) and\
            isinstance(other.corner_max, gp_Pnt) and\
            isinstance(other.corner_min, gp_Pnt) and\
            isinstance(other.center_pnt, gp_Pnt):
            return self.copy_geom_properties_from(other)
        return 

    def reset(self) -> None:
        self.parent = None
        self.f, self.g = 0.0, 0.0
        return

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Node):
            return self.i == other.i and self.j == other.j and self.k == other.k
        else:
            return False        
        
    def __hash__(self) -> int:  
        return hash((self.i, self.j, self.k))
    
    def __str__(self) -> str:
        return f"voxel Node ({self.i},{self.j},{self.k}), g = {self.g}, f = {self.f}"
    
    def __lt__(self, other: "Node") -> bool:
        return self.f < other.f

from typing import Tuple, Optional, List
import numpy as np

class Grids3D(Box):
    def __init__(self, 
                corner_min: gp_Pnt, 
                corner_max: gp_Pnt, 
                map_size: int) -> None:
        super().__init__()          
        super().create_box(corner_min, corner_max)  
        self.start_node: Optional[Node] = None
        self.goal_node: Optional[Node] = None
        self.map_size: int = map_size   
        self.nodes_map: List[List[List[Node]]]  = \
            [[[Node(i, j, k) for k in range(map_size)] for j in range(map_size)] for i in range(map_size)] 
        self.node_gap: float = self.gap / map_size 
        for i in range(map_size):
            for j in range(map_size):
                for k in range(map_size):
                    
                    
                    node_corner_min = gp_Pnt(corner_min.X() + i * self.node_gap, 
                                             corner_min.Y() +  j * self.node_gap, 
                                             corner_min.Z() +  k * self.node_gap)
                    node_corner_max = gp_Pnt(corner_min.X() + (i + 1) * self.node_gap, 
                                             corner_min.Y() + (j + 1) * self.node_gap, 
                                             corner_min.Z() + (k + 1) * self.node_gap)  
                    self.nodes_map[i][j][k].create_box(node_corner_min, node_corner_max)  

    def reset_nodes(self) -> None:    
        for node in self:
            node.reset()
        return
    
    @dispatch(int, int, int)    
    def set_start_node(self, i: int, j: int, k: int) -> None:   
        if isinstance(self.start_node, Node):
            self.start_node.is_start_node = False
        self.start_node = self.nodes_map[i][j][k]
        self.start_node.is_start_node = True
        return  
    
    @dispatch(Node)
    def set_start_node(self, node: Node) -> None:
        if isinstance(self.start_node, Node):
            self.start_node.is_start_node = False
        self.start_node = node
        node.is_start_node = True
        return
    
    @dispatch(int, int, int)
    def set_goal_node(self, i: int, j: int, k: int) -> None:
        if isinstance(self.goal_node, Node):
            self.goal_node.is_goal_node = False
        self.goal_node = self.nodes_map[i][j][k]
        self.goal_node.is_goal_node = True
        return
    
    @dispatch(Node)
    def set_goal_node(self, node: Node) -> None:
        if isinstance(self.goal_node, Node):
            self.goal_node.is_goal_node = False
        self.goal_node = node
        node.is_goal_node = True
        return  
    
    def __iter__(self):
        for i in range(self.map_size):
            for j in range(self.map_size):
                for k in range(self.map_size):
                    yield self.nodes_map[i][j][k]    
                    
    def __getitem__(self, index: Tuple[int, int, int]) -> Node:
        return self.nodes_map[index[0]][index[1]][index[2]]
        
    def __len__(self):
        return len(self.nodes_map)
    
    def save_grid_map(self, file_path: str) -> None:
        """_summary_

        Args:
            file_path (str): .npy must be included
        description:
            save the grid map as a numpy array
            1 is obstacle, 0 is not
        """
        np_arr = np.zeros((self.map_size, self.map_size, self.map_size))    
        # 1 is obstacle, 0 is not
        for node in self:
            np_arr[node.i, node.j, node.k] = 1 if node.is_obstacle else 0
        
        return np.save(file_path, np_arr)
    
    def load_grid_map(self, file_path: str) -> None:
        """_summary_

        Args:
            file_path (str): .npy must be included
        discription:
            load the grid map from a numpy array
            goal node is updated to the last node
            start node is updated to the first node 
        """
        self.reset_nodes()
        np_arr = np.load(file_path)    
        # 1 is obstacle, 0 is not   
        for node in self:
            node.is_obstacle = True if np_arr[node.i, node.j, node.k] == 1 else False
        
        # basic update for start and goal nodes
        start_node = self[0, 0, 0]
        self.set_start_node(start_node)
        
        goal_node = self[self.map_size - 1, self.map_size - 1, self.map_size - 1]
        self.set_goal_node(goal_node)
        return
