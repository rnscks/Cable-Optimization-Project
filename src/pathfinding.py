from typing import Optional, Tuple, List
import heapq        

from src.grids.grids3d import Grids3D, Node

class GridAlgorithm:
    def __init__(self, grids: Optional[None]) -> None:
        self.grids: Grids3D = grids 
    
    
    def __iter__(self):
        cur_node: Node = self.grids.goal_node
        start_node: Node = self.grids.start_node
        start_node.parent = None
        
        while cur_node is not None:
            yield cur_node
            cur_node = cur_node.parent
    
    def get_path_nodes(self) -> List[Node]:
        path_nodes = []
        for path_node in self:
            path_nodes.append(path_node)
        path_nodes.reverse()
        return path_nodes 
    
    def get_path_distance(self) -> float:
        distance_sum: float = 0.0
        for path_node in self:
            if isinstance(path_node.parent, Node):
                distance_sum += self.get_distance(path_node, path_node.parent)
        
        return distance_sum


    def has_line_of_sight(self, src_node: Node, dst_node: Node) -> bool:
        x1, y1, z1 = src_node.i, src_node.j, src_node.k
        x2, y2, z2 = dst_node.i, dst_node.j, dst_node.k

        if (x2 > x1):
            xs = 1
            dx = x2 - x1
        else:
            xs = -1
            dx = x1 - x2

        if (y2 > y1):
            ys = 1
            dy = y2 - y1
        else:
            ys = -1
            dy = y1 - y2

        if (z2 > z1):
            zs = 1
            dz = z2 - z1
        else:
            zs = -1
            dz = z1 - z2

        if (dx >= dy and dx >= dz):
            p1 = 2 * dy - dx
            p2 = 2 * dz - dx
            while (x1 != x2):
                x1 += xs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dx
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dx
                p1 += 2 * dy
                p2 += 2 * dz
                if self.grids[x1, y1, z1].is_obstacle: 
                    return False

        elif (dy >= dx and dy >= dz):
            p1 = 2 * dx - dy
            p2 = 2 * dz - dy
            while (y1 != y2):
                y1 += ys
                if (p1 >= 0):
                    x1 += xs
                    p1 -= 2 * dy
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dy
                p1 += 2 * dx
                p2 += 2 * dz
                if self.grids[x1, y1, z1].is_obstacle:  
                    return False
        else:
            p1 = 2 * dy - dz
            p2 = 2 * dx - dz
            while (z1 != z2):
                z1 += zs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dz
                if (p2 >= 0):
                    x1 += xs
                    p2 -= 2 * dz
                p1 += 2 * dy
                p2 += 2 * dx
                if self.grids[x1, y1, z1].is_obstacle:  
                    return False
        return True
    
    def get_smoothed_path_nodes(self) -> List[Node]:
        dst_node = self.grids.goal_node
        src_node = self.grids.start_node    
        src_node.parent = None  
        
        smoothed_node_list = []
        finder: Node = dst_node
        smoothed_node_list.append(finder)
        tmp_node: Node = finder.parent

        while tmp_node != src_node:
            while self.has_line_of_sight(finder, tmp_node.parent):
                tmp_node = tmp_node.parent
                if tmp_node == src_node: 
                    break
            finder = tmp_node
            if tmp_node == src_node:
                break   
            tmp_node = finder.parent
            smoothed_node_list.append(finder)
        
        smoothed_node_list.append(src_node)
        return smoothed_node_list   
    
    
class Direction:    
    def __init__(self, dir_i: int, dir_j: int, dir_k: int) -> None:
        self.dir_i: int = dir_i 
        self.dir_j: int = dir_j 
        self.dir_k: int = dir_k
    
    
    def get_nxt_direction(self, dir_i: int, dir_j: int, dir_k: int) -> 'Direction': 
        return Direction(self.dir_i + dir_i, self.dir_j + dir_j, self.dir_k + dir_k)    
    
    def get_direction_components(self) -> Tuple[int, int, int]:
        if self.dir_i != 0 and self.dir_j != 0 and self.dir_k != 0: 
            directions = [Direction(self.dir_i, 0, 0), 
                            Direction(0, self.dir_j, 0), 
                            Direction(0, 0, self.dir_k),
                            Direction(self.dir_i, self.dir_j, 0),
                            Direction(self.dir_i, 0, self.dir_k),
                            Direction(0, self.dir_j, self.dir_k),]
            return directions
        elif self.dir_i != 0 and self.dir_j != 0 and self.dir_k == 0:   
            directions = [Direction(self.dir_i, 0, 0),  
                            Direction(0, self.dir_j, 0)]   
            return directions
        elif self.dir_i != 0 and self.dir_j == 0 and self.dir_k != 0:
            directions = [Direction(self.dir_i, 0, 0),
                            Direction(0, 0, self.dir_k)]
            return directions
        elif self.dir_i == 0 and self.dir_j != 0 and self.dir_k != 0:
            directions = [Direction(0, self.dir_j, 0),
                            Direction(0, 0, self.dir_k)]
            return directions
        
    def __lt__(self, other):
        return self.dir_i < other.dir_i

class JumpPointSearch(GridAlgorithm):
    def __init__(self, grids: Grids3D) -> None:
        self.grids: Grids3D = grids
        self.goal_node: Node = grids.goal_node
        self.start_node: Node = grids.start_node    
        self.open_list: list[Node] = [] 
        self.close_list = set()
        self.scaned_node_set = set()   
        self.grids.reset_nodes()
        self.scan_directions = [Direction(1, 0, 0),
                                Direction(0, 1, 0),
                                Direction(0, 0, 1),
                                Direction(0, 0, -1),  
                                Direction(0, -1, 0),    
                                Direction(-1, 0, 0), 
                                
                                Direction(1, 1, 0), 
                                Direction(1, 0, 1),
                                Direction(0, 1, 1),
                                
                                Direction(-1, 1, 0),    
                                Direction(-1, 0, 1),
                                Direction(0, -1, 1),
                                Direction(1, -1, 0),
                                Direction(1, 0, -1),
                                Direction(0, 1, -1),
                                
                                Direction(-1, -1, 0),
                                Direction(-1, 0, -1),
                                Direction(0, -1, -1),
                                
                                Direction(1, 1, 1),  
                                Direction(1, 1, -1),
                                Direction(1, -1, 1),
                                Direction(-1, 1, 1),
                                
                                Direction(1, -1, -1),
                                Direction(-1, 1, -1),
                                Direction(-1, -1, 1),
                                Direction(-1, -1, -1)]
        self.direction_orthogonal_i_col = \
                            [(0, 1, 0), 
                            (0, -1, 0), 
                            (0, 0, 1), 
                            (0, 0, -1),]

        self.direction_orthogonal_j_col = \
                            [(1, 0, 0), 
                            (-1, 0, 0), 
                            (0, 0, 1), 
                            (0, 0, -1)]
        self.direction_orthogonal_k_col = \
                            [(1, 0, 0),
                            (-1, 0, 0),    
                            (0, 1, 0),
                            (0, -1, 0)]

    def search(self):
        self.open_list.clear()
        self.close_list.clear()
        
        heapq.heappush(self.open_list, self.start_node) 
        
        while self.open_list:
            cur_node: Node = heapq.heappop(self.open_list)         
            if cur_node.is_goal_node == True:
                return True
            if cur_node in self.close_list:
                continue
            
            self.close_list.add(cur_node)
            self.jump_diagonal3d(cur_node, 1, 1, 1)
            self.jump_diagonal3d(cur_node, 1, 1, -1)
            self.jump_diagonal3d(cur_node, 1, -1, 1)
            self.jump_diagonal3d(cur_node, -1, 1, 1)
            self.jump_diagonal3d(cur_node, 1, -1, -1)
            self.jump_diagonal3d(cur_node, -1, 1, -1)
            self.jump_diagonal3d(cur_node, -1, -1, 1)
            self.jump_diagonal3d(cur_node, -1, -1, -1)
            
            self.jump_diagonal2d(cur_node, 1, 1, 0)
            self.jump_diagonal2d(cur_node, 1, 0, 1)
            self.jump_diagonal2d(cur_node, 0, 1, 1)
            self.jump_diagonal2d(cur_node, 1, -1, 0)
            self.jump_diagonal2d(cur_node, 1, 0, -1)
            self.jump_diagonal2d(cur_node, 0, 1, -1)
            self.jump_diagonal2d(cur_node, -1, 1, 0)
            self.jump_diagonal2d(cur_node, -1, 0, 1)
            self.jump_diagonal2d(cur_node, 0, -1, 1)
            self.jump_diagonal2d(cur_node, -1, -1, 0)
            self.jump_diagonal2d(cur_node, -1, 0, -1)
            self.jump_diagonal2d(cur_node, 0, -1, -1)
            
            self.jump_orthogonal(cur_node, 1, 0, 0) 
            self.jump_orthogonal(cur_node, 0, 1, 0)
            self.jump_orthogonal(cur_node, 0, 0, 1)
            self.jump_orthogonal(cur_node, -1, 0, 0)
            self.jump_orthogonal(cur_node, 0, -1, 0)
            self.jump_orthogonal(cur_node, 0, 0, -1) 
            
        return False
            
    def jump_orthogonal(self, cur_node: Node, dir_i: int, dir_j: int, dir_k: int) -> None:     
        if dir_i == 0 and dir_j == 0 and dir_k == 0:
            return
        
        map_size = self.grids.map_size
        cur_i, cur_j, cur_k = cur_node.i, cur_node.j, cur_node.k    
        while True:
            nxt_i, nxt_j, nxt_k = cur_i + dir_i, cur_j + dir_j, cur_k + dir_k
            if nxt_i < 0 or nxt_i >= map_size or nxt_j < 0 or nxt_j >= map_size or nxt_k < 0 or nxt_k >= map_size:
                return
            
            nxt_node = self.grids[nxt_i, nxt_j, nxt_k] 
            if nxt_node.is_obstacle == True:
                return
            if nxt_node in self.close_list:
                return

            if nxt_node.is_goal_node == True or self.has_forced_neighbor(nxt_node, dir_i, dir_j, dir_k):
                ng = cur_node.g + cur_node.center_pnt.Distance(nxt_node.center_pnt)
                    
                if nxt_node.parent is None or ng < nxt_node.g:
                    nxt_node.g = ng
                    nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                    nxt_node.parent = cur_node
                    heapq.heappush(self.open_list, nxt_node)
                return
            
            nxt_node.parent = cur_node
            cur_i, cur_j, cur_k = nxt_i, nxt_j, nxt_k   
            self.close_list.add(nxt_node)   

            
    def jump_diagonal2d(self, cur_node: Node, dir_i: int, dir_j: int, dir_k: int) -> Optional[Node]:     
        map_size = self.grids.map_size
        cur_i, cur_j, cur_k = cur_node.i, cur_node.j, cur_node.k    
            
        while True:
            nxt_i, nxt_j, nxt_k = cur_i + dir_i, cur_j + dir_j, cur_k + dir_k
            if nxt_i < 0 or nxt_i >= map_size or nxt_j < 0 or nxt_j >= map_size or nxt_k < 0 or nxt_k >= map_size:
                return None
            
            nxt_node = self.grids[nxt_i, nxt_j, nxt_k] 
            if nxt_node.is_obstacle == True:
                return None
            if nxt_node in self.close_list:
                return None
            
            if nxt_node.is_goal_node == True or self.has_forced_neighbor(nxt_node, dir_i, dir_j, dir_k):
                ng = cur_node.g + cur_node.center_pnt.Distance(nxt_node.center_pnt)
                    
                if nxt_node.parent is None or ng < nxt_node.g:
                    nxt_node.g = ng
                    nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                    nxt_node.parent = cur_node
                    heapq.heappush(self.open_list, nxt_node)
                return

            self.jump_orthogonal(nxt_node, dir_i, 0, 0)
            self.jump_orthogonal(nxt_node, 0, dir_j, 0)
            self.jump_orthogonal(nxt_node, 0, 0, dir_k)
            
            cur_i, cur_j, cur_k = nxt_i, nxt_j, nxt_k   
            nxt_node.parent = cur_node
            self.close_list.add(nxt_node)   
    
    def jump_diagonal3d(self, cur_node: Node, dir_i: int, dir_j: int, dir_k: int) -> None:     
        map_size = self.grids.map_size
        cur_i, cur_j, cur_k = cur_node.i, cur_node.j, cur_node.k    
            
        while True:
            nxt_i, nxt_j, nxt_k = cur_i + dir_i, cur_j + dir_j, cur_k + dir_k
            if nxt_i < 0 or nxt_i >= map_size or nxt_j < 0 or nxt_j >= map_size or nxt_k < 0 or nxt_k >= map_size:
                return
            
            nxt_node = self.grids[nxt_i, nxt_j, nxt_k] 
            if nxt_node.is_obstacle == True:
                return
            if nxt_node in self.close_list:
                return
            
            if nxt_node.is_goal_node == True or self.has_forced_neighbor(nxt_node, dir_i, dir_j, dir_k):
                ng = cur_node.g + cur_node.center_pnt.Distance(nxt_node.center_pnt)
                    
                if nxt_node.parent is None or ng < nxt_node.g:
                    nxt_node.g = ng
                    nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                    nxt_node.parent = cur_node
                    heapq.heappush(self.open_list, nxt_node)
                return
            
            
            self.jump_orthogonal(nxt_node, dir_i, 0, 0)
            self.jump_orthogonal(nxt_node, 0, dir_j, 0)
            self.jump_orthogonal(nxt_node, 0, 0, dir_k)
                
            
            self.jump_diagonal2d(nxt_node, dir_i, dir_j, 0)
            self.jump_diagonal2d(nxt_node, dir_i, 0, dir_k)
            self.jump_diagonal2d(nxt_node, 0, dir_j, dir_k)

            cur_i, cur_j, cur_k = nxt_i, nxt_j, nxt_k   
            nxt_node.parent = cur_node
            self.close_list.add(nxt_node)   
    
    def has_forced_neighbor(self, node: Node, dir_i, dir_j, dir_k) -> bool:
        map_size = self.grids.map_size
        
        if dir_i != 0 and dir_j == 0 and dir_k == 0:
            node_i, node_j, node_k = node.i, node.j, node.k    
            for nxt_direction_col in self.direction_orthogonal_i_col:
                col_i, col_j, col_k = node_i + nxt_direction_col[0], node_j + nxt_direction_col[1], node_k + nxt_direction_col[2]
                if col_i < 0 or col_i >= map_size or col_j < 0 or col_j >= map_size or col_k < 0 or col_k >= map_size:
                    continue
                free_i, free_j, free_k = col_i + dir_i, col_j, col_k
                if free_i < 0 or free_i >= map_size or free_j < 0 or free_j >= map_size or free_k < 0 or free_k >= map_size:    
                    continue
                collusion_node = self.grids[col_i, col_j, col_k]
                free_node = self.grids[free_i, free_j, free_k] 
                
                if collusion_node.is_obstacle == True and free_node.is_obstacle == False:
                    return True
            return False
        
        if dir_i == 0 and dir_j != 0 and dir_k == 0:
            node_i, node_j, node_k = node.i, node.j, node.k    
            for nxt_direction_col in self.direction_orthogonal_j_col:
                col_i, col_j, col_k = node_i + nxt_direction_col[0], node_j + nxt_direction_col[1], node_k + nxt_direction_col[2]
                if col_i < 0 or col_i >= map_size or col_j < 0 or col_j >= map_size or col_k < 0 or col_k >= map_size:
                    continue
                free_i, free_j, free_k = col_i, col_j + dir_j, col_k
                if free_i < 0 or free_i >= map_size or free_j < 0 or free_j >= map_size or free_k < 0 or free_k >= map_size:    
                    continue
                collusion_node = self.grids[col_i, col_j, col_k]
                free_node = self.grids[free_i, free_j, free_k] 
                
                if collusion_node.is_obstacle == True and free_node.is_obstacle == False:
                    return True
            return False
        
        if dir_i == 0 and dir_j == 0 and dir_k != 0:
            node_i, node_j, node_k = node.i, node.j, node.k    
            for nxt_direction_col in self.direction_orthogonal_k_col:
                col_i, col_j, col_k = node_i + nxt_direction_col[0], node_j + nxt_direction_col[1], node_k + nxt_direction_col[2]
                if col_i < 0 or col_i >= map_size or col_j < 0 or col_j >= map_size or col_k < 0 or col_k >= map_size:
                    continue
                free_i, free_j, free_k = col_i, col_j, col_k + dir_k
                if free_i < 0 or free_i >= map_size or free_j < 0 or free_j >= map_size or free_k < 0 or free_k >= map_size:    
                    continue
                collusion_node = self.grids[col_i, col_j, col_k]
                free_node = self.grids[free_i, free_j, free_k] 
                
                if collusion_node.is_obstacle == True and free_node.is_obstacle == False:
                    return True
            return False
        return True

class AstarAlgorithmOp(GridAlgorithm):
    def __init__(self, grids: Grids3D) -> None:
        super().__init__(grids)
        self.open_list: list[Node] = []
        self.close_list = set()
        self.grids.reset_nodes()
    
    
    def search(self) -> bool:
        start_node: Node = self.grids.start_node
        start_node.f = start_node.center_pnt.Distance(self.grids.goal_node.center_pnt)
        goal_node: Node = self.grids.goal_node  
        heapq.heappush(self.open_list, start_node)
        map_size = self.grids.map_size
        while self.open_list:
            cur_node: Node = heapq.heappop(self.open_list)

            if cur_node == goal_node:
                return True
            self.close_list.add(cur_node)
            cur_i, cur_j, cur_k = cur_node.i, cur_node.j, cur_node.k
            for i in range(-1, 2):
                for j in range(-1, 2):
                    for k in range(-1 ,2):
                        if i == j == k == 0:    
                            continue
                        nei_i, nei_j, nei_k = cur_i + i, cur_j + j, cur_k + k   
                        if nei_i < 0 or nei_j < 0 or nei_k < 0:    
                            continue
                        if map_size <= nei_i or map_size <= nei_j or map_size <= nei_k:
                            continue
                        
                        nei_node = self.grids[nei_i, nei_j, nei_k]

                        if nei_node in self.close_list or nei_node.is_obstacle:
                            continue           
                        
                        nxt_node: Node = nei_node
                        nxt_g = cur_node.g + cur_node.center_pnt.Distance(nxt_node.center_pnt)  

                        if nei_node.parent is None or nxt_g < nei_node.g:
                            nei_node.parent = cur_node
                            nei_node.g = nxt_g
                            nei_node.f = nxt_g + nxt_node.center_pnt.Distance(goal_node.center_pnt)
                            heapq.heappush(self.open_list, nei_node)
        return False

class JumpPointSearchTheta(GridAlgorithm):
    def __init__(self, grids: Grids3D) -> None:
        self.grids: Grids3D = grids
        self.goal_node: Node = grids.goal_node
        self.start_node: Node = grids.start_node    
        self.open_list: list[Node] = [] 
        self.close_list = set()
        self.scaned_node_set = set()   
        self.grids.reset_nodes()
        self.scan_directions = [Direction(1, 0, 0),
                                Direction(0, 1, 0),
                                Direction(0, 0, 1),
                                Direction(0, 0, -1),  
                                Direction(0, -1, 0),    
                                Direction(-1, 0, 0), 
                                
                                Direction(1, 1, 0), 
                                Direction(1, 0, 1),
                                Direction(0, 1, 1),
                                
                                Direction(-1, 1, 0),    
                                Direction(-1, 0, 1),
                                Direction(0, -1, 1),
                                Direction(1, -1, 0),
                                Direction(1, 0, -1),
                                Direction(0, 1, -1),
                                
                                Direction(-1, -1, 0),
                                Direction(-1, 0, -1),
                                Direction(0, -1, -1),
                                
                                Direction(1, 1, 1),  
                                Direction(1, 1, -1),
                                Direction(1, -1, 1),
                                Direction(-1, 1, 1),
                                
                                Direction(1, -1, -1),
                                Direction(-1, 1, -1),
                                Direction(-1, -1, 1),
                                Direction(-1, -1, -1)]
        self.direction_orthogonal_i_col = \
                            [(0, 1, 0), 
                            (0, -1, 0), 
                            (0, 0, 1), 
                            (0, 0, -1),]

        self.direction_orthogonal_j_col = \
                            [(1, 0, 0), 
                            (-1, 0, 0), 
                            (0, 0, 1), 
                            (0, 0, -1)]
        self.direction_orthogonal_k_col = \
                            [(1, 0, 0),
                            (-1, 0, 0),    
                            (0, 1, 0),
                            (0, -1, 0)]

    def search(self):
        self.open_list.clear()
        self.close_list.clear()
        
        heapq.heappush(self.open_list, self.start_node) 
        
        while self.open_list:
            cur_node: Node = heapq.heappop(self.open_list)         
            if cur_node.is_goal_node == True:
                return True
            if cur_node in self.close_list:
                continue
            
            self.close_list.add(cur_node)
            self.jump_diagonal3d(cur_node, 1, 1, 1)
            self.jump_diagonal3d(cur_node, 1, 1, -1)
            self.jump_diagonal3d(cur_node, 1, -1, 1)
            self.jump_diagonal3d(cur_node, -1, 1, 1)
            self.jump_diagonal3d(cur_node, 1, -1, -1)
            self.jump_diagonal3d(cur_node, -1, 1, -1)
            self.jump_diagonal3d(cur_node, -1, -1, 1)
            self.jump_diagonal3d(cur_node, -1, -1, -1)
            
            self.jump_diagonal2d(cur_node, 1, 1, 0)
            self.jump_diagonal2d(cur_node, 1, 0, 1)
            self.jump_diagonal2d(cur_node, 0, 1, 1)
            self.jump_diagonal2d(cur_node, 1, -1, 0)
            self.jump_diagonal2d(cur_node, 1, 0, -1)
            self.jump_diagonal2d(cur_node, 0, 1, -1)
            self.jump_diagonal2d(cur_node, -1, 1, 0)
            self.jump_diagonal2d(cur_node, -1, 0, 1)
            self.jump_diagonal2d(cur_node, 0, -1, 1)
            self.jump_diagonal2d(cur_node, -1, -1, 0)
            self.jump_diagonal2d(cur_node, -1, 0, -1)
            self.jump_diagonal2d(cur_node, 0, -1, -1)
            
            self.jump_orthogonal(cur_node, 1, 0, 0) 
            self.jump_orthogonal(cur_node, 0, 1, 0)
            self.jump_orthogonal(cur_node, 0, 0, 1)
            self.jump_orthogonal(cur_node, -1, 0, 0)
            self.jump_orthogonal(cur_node, 0, -1, 0)
            self.jump_orthogonal(cur_node, 0, 0, -1) 
            
        return False
            
    def jump_orthogonal(self, cur_node: Node, dir_i: int, dir_j: int, dir_k: int) -> None:     
        if dir_i == 0 and dir_j == 0 and dir_k == 0:
            return
        
        map_size = self.grids.map_size
        cur_i, cur_j, cur_k = cur_node.i, cur_node.j, cur_node.k    
        while True:
            nxt_i, nxt_j, nxt_k = cur_i + dir_i, cur_j + dir_j, cur_k + dir_k
            if nxt_i < 0 or nxt_i >= map_size or nxt_j < 0 or nxt_j >= map_size or nxt_k < 0 or nxt_k >= map_size:
                return
            
            nxt_node = self.grids[nxt_i, nxt_j, nxt_k] 
            if nxt_node.is_obstacle == True:
                return
            if nxt_node in self.close_list:
                return

            if nxt_node.is_goal_node == True or self.has_forced_neighbor(nxt_node, dir_i, dir_j, dir_k):
                if cur_node.parent is not None and self.has_line_of_sight(cur_node.parent, nxt_node):
                    ng: float = cur_node.parent.g + cur_node.parent.center_pnt.Distance(nxt_node.center_pnt)  
                    if nxt_node.parent is None or ng < nxt_node.g:
                        nxt_node.g = ng
                        nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                        nxt_node.parent = cur_node.parent
                        heapq.heappush(self.open_list, nxt_node)    
                else:
                    ng = cur_node.g + cur_node.center_pnt.Distance(nxt_node.center_pnt)
                    
                    if nxt_node.parent is None or ng < nxt_node.g:
                        nxt_node.g = ng
                        nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                        nxt_node.parent = cur_node
                        heapq.heappush(self.open_list, nxt_node)
                return
            
            nxt_node.parent = cur_node
            cur_i, cur_j, cur_k = nxt_i, nxt_j, nxt_k   
            self.close_list.add(nxt_node)   

            
    def jump_diagonal2d(self, cur_node: Node, dir_i: int, dir_j: int, dir_k: int) -> Optional[Node]:     
        map_size = self.grids.map_size
        cur_i, cur_j, cur_k = cur_node.i, cur_node.j, cur_node.k    
            
        while True:
            nxt_i, nxt_j, nxt_k = cur_i + dir_i, cur_j + dir_j, cur_k + dir_k
            if nxt_i < 0 or nxt_i >= map_size or nxt_j < 0 or nxt_j >= map_size or nxt_k < 0 or nxt_k >= map_size:
                return None
            
            nxt_node = self.grids[nxt_i, nxt_j, nxt_k] 
            if nxt_node.is_obstacle == True:
                return None
            if nxt_node in self.close_list:
                return None
            
            if nxt_node.is_goal_node == True or self.has_forced_neighbor(nxt_node, dir_i, dir_j, dir_k):
                if cur_node.parent is not None and self.has_line_of_sight(cur_node.parent, nxt_node):
                    ng: float = cur_node.parent.g + cur_node.parent.center_pnt.Distance(nxt_node.center_pnt)  
                    if nxt_node.parent is None or ng < nxt_node.g:
                        nxt_node.g = ng
                        nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                        nxt_node.parent = cur_node.parent
                        heapq.heappush(self.open_list, nxt_node)    
                else:
                    ng = cur_node.g + cur_node.center_pnt.Distance(nxt_node.center_pnt)
                    
                    if nxt_node.parent is None or ng < nxt_node.g:
                        nxt_node.g = ng
                        nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                        nxt_node.parent = cur_node
                        heapq.heappush(self.open_list, nxt_node)
                return

            self.jump_orthogonal(nxt_node, dir_i, 0, 0)
            self.jump_orthogonal(nxt_node, 0, dir_j, 0)
            self.jump_orthogonal(nxt_node, 0, 0, dir_k)
            
            cur_i, cur_j, cur_k = nxt_i, nxt_j, nxt_k   
            nxt_node.parent = cur_node
            self.close_list.add(nxt_node)   
    
    def jump_diagonal3d(self, cur_node: Node, dir_i: int, dir_j: int, dir_k: int) -> None:     
        map_size = self.grids.map_size
        cur_i, cur_j, cur_k = cur_node.i, cur_node.j, cur_node.k    
            
        while True:
            nxt_i, nxt_j, nxt_k = cur_i + dir_i, cur_j + dir_j, cur_k + dir_k
            if nxt_i < 0 or nxt_i >= map_size or nxt_j < 0 or nxt_j >= map_size or nxt_k < 0 or nxt_k >= map_size:
                return
            
            nxt_node = self.grids[nxt_i, nxt_j, nxt_k] 
            if nxt_node.is_obstacle == True:
                return
            if nxt_node in self.close_list:
                return
            
            if nxt_node.is_goal_node == True or self.has_forced_neighbor(nxt_node, dir_i, dir_j, dir_k):
                if cur_node.parent is not None and self.has_line_of_sight(cur_node.parent, nxt_node):
                    ng: float = cur_node.parent.g + cur_node.parent.center_pnt.Distance(nxt_node.center_pnt)  
                    if nxt_node.parent is None or ng < nxt_node.g:
                        nxt_node.g = ng
                        nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                        nxt_node.parent = cur_node.parent
                        heapq.heappush(self.open_list, nxt_node)    
                else:
                    ng = cur_node.g + cur_node.center_pnt.Distance(nxt_node.center_pnt)
                    
                    if nxt_node.parent is None or ng < nxt_node.g:
                        nxt_node.g = ng
                        nxt_node.f = ng + nxt_node.center_pnt.Distance(self.goal_node.center_pnt)
                        nxt_node.parent = cur_node
                        heapq.heappush(self.open_list, nxt_node)
                return
            
            
            self.jump_orthogonal(nxt_node, dir_i, 0, 0)
            self.jump_orthogonal(nxt_node, 0, dir_j, 0)
            self.jump_orthogonal(nxt_node, 0, 0, dir_k)
                
            
            self.jump_diagonal2d(nxt_node, dir_i, dir_j, 0)
            self.jump_diagonal2d(nxt_node, dir_i, 0, dir_k)
            self.jump_diagonal2d(nxt_node, 0, dir_j, dir_k)

            cur_i, cur_j, cur_k = nxt_i, nxt_j, nxt_k   
            nxt_node.parent = cur_node
            self.close_list.add(nxt_node)   
    
    def has_forced_neighbor(self, node: Node, dir_i, dir_j, dir_k) -> bool:
        map_size = self.grids.map_size
        
        if dir_i != 0 and dir_j == 0 and dir_k == 0:
            node_i, node_j, node_k = node.i, node.j, node.k    
            for nxt_direction_col in self.direction_orthogonal_i_col:
                col_i, col_j, col_k = node_i + nxt_direction_col[0], node_j + nxt_direction_col[1], node_k + nxt_direction_col[2]
                if col_i < 0 or col_i >= map_size or col_j < 0 or col_j >= map_size or col_k < 0 or col_k >= map_size:
                    continue
                free_i, free_j, free_k = col_i + dir_i, col_j, col_k
                if free_i < 0 or free_i >= map_size or free_j < 0 or free_j >= map_size or free_k < 0 or free_k >= map_size:    
                    continue
                collusion_node = self.grids[col_i, col_j, col_k]
                free_node = self.grids[free_i, free_j, free_k] 
                
                if collusion_node.is_obstacle == True and free_node.is_obstacle == False:
                    return True
            return False
        
        if dir_i == 0 and dir_j != 0 and dir_k == 0:
            node_i, node_j, node_k = node.i, node.j, node.k    
            for nxt_direction_col in self.direction_orthogonal_j_col:
                col_i, col_j, col_k = node_i + nxt_direction_col[0], node_j + nxt_direction_col[1], node_k + nxt_direction_col[2]
                if col_i < 0 or col_i >= map_size or col_j < 0 or col_j >= map_size or col_k < 0 or col_k >= map_size:
                    continue
                free_i, free_j, free_k = col_i, col_j + dir_j, col_k
                if free_i < 0 or free_i >= map_size or free_j < 0 or free_j >= map_size or free_k < 0 or free_k >= map_size:    
                    continue
                collusion_node = self.grids[col_i, col_j, col_k]
                free_node = self.grids[free_i, free_j, free_k] 
                
                if collusion_node.is_obstacle == True and free_node.is_obstacle == False:
                    return True
            return False
        
        if dir_i == 0 and dir_j == 0 and dir_k != 0:
            node_i, node_j, node_k = node.i, node.j, node.k    
            for nxt_direction_col in self.direction_orthogonal_k_col:
                col_i, col_j, col_k = node_i + nxt_direction_col[0], node_j + nxt_direction_col[1], node_k + nxt_direction_col[2]
                if col_i < 0 or col_i >= map_size or col_j < 0 or col_j >= map_size or col_k < 0 or col_k >= map_size:
                    continue
                free_i, free_j, free_k = col_i, col_j, col_k + dir_k
                if free_i < 0 or free_i >= map_size or free_j < 0 or free_j >= map_size or free_k < 0 or free_k >= map_size:    
                    continue
                collusion_node = self.grids[col_i, col_j, col_k]
                free_node = self.grids[free_i, free_j, free_k] 
                
                if collusion_node.is_obstacle == True and free_node.is_obstacle == False:
                    return True
            return False
        return True