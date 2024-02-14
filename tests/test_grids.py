import os
import unittest
from OCC.Core.gp import gp_Pnt
from OCC.Core.TopoDS import TopoDS_Shape

from src.grids.grids3d import Box, Node, Grids3D


class TestBox(unittest.TestCase):
    def setUp(self):
        self.box = Box()

    def test_create_box_with_int_gap(self):
        center_pnt = gp_Pnt(0, 0, 0)
        gap = 10
        self.box.create_box(center_pnt, gap)
        self.assertEqual(self.box.gap, gap)
        self.assertEqual(self.box.corner_min.X(), -5)
        self.assertEqual(self.box.corner_min.Y(), -5)
        self.assertEqual(self.box.corner_min.Z(), -5)
        self.assertEqual(self.box.corner_max.X(), 5)
        self.assertEqual(self.box.corner_max.Y(), 5)
        self.assertEqual(self.box.corner_max.Z(), 5)

    def test_create_box_with_float_gap(self):
        center_pnt = gp_Pnt(0, 0, 0)
        gap = 10.5
        self.box.create_box(center_pnt, gap)
        self.assertEqual(self.box.gap, gap)
        self.assertEqual(self.box.corner_min.X(), -5.25)
        self.assertEqual(self.box.corner_min.Y(), -5.25)
        self.assertEqual(self.box.corner_min.Z(), -5.25)
        self.assertEqual(self.box.corner_max.X(), 5.25)
        self.assertEqual(self.box.corner_max.Y(), 5.25)
        self.assertEqual(self.box.corner_max.Z(), 5.25)
        

    def test_create_box_with_corner_points(self):
        corner_min = gp_Pnt(-5, -5, -5)
        corner_max = gp_Pnt(5, 5, 5)
        self.box.create_box(corner_min, corner_max)
        self.assertEqual(self.box.corner_min, corner_min)
        self.assertEqual(self.box.corner_max, corner_max)
        self.assertEqual(self.box.gap, 10)

    def test_get_box_shape(self):
        corner_min = gp_Pnt(-5, -5, -5)
        corner_max = gp_Pnt(5, 5, 5)
        self.box.create_box(corner_min, corner_max)
        shape = self.box.get_box_shape()
        self.assertIsInstance(shape, TopoDS_Shape)

    def test_copy_geom_properties_from(self):
        box1 = Box()
        box1.create_box(gp_Pnt(0, 0, 0), 10)
        box2 = Box()
        box2.copy_geom_properties_from(box1)
        # check deep copy
        self.assertNotEqual(box2.corner_min, box1.corner_min)
        self.assertNotEqual(box2.corner_max, box1.corner_max)
        self.assertNotEqual(box2.get_box_shape(), box1.get_box_shape())
        # check numrical properties
        self.assertEqual(box2.gap, box1.gap)
        self.assertEqual(box2.corner_min.X(), box1.corner_min.X())
        self.assertEqual(box2.corner_min.Y(), box1.corner_min.Y())  
        self.assertEqual(box2.corner_min.Z(), box1.corner_min.Z())
        self.assertEqual(box2.corner_max.X(), box1.corner_max.X())
        self.assertEqual(box2.corner_max.Y(), box1.corner_max.Y())
        self.assertEqual(box2.corner_max.Z(), box1.corner_max.Z())

    def test_get_corner_min(self):
        center_pnt = gp_Pnt(0, 0, 0)
        gap = 10
        corner_min = self.box.get_corner_min(center_pnt, gap)
        self.assertEqual(corner_min.X(), -5)
        self.assertEqual(corner_min.Y(), -5)
        self.assertEqual(corner_min.Z(), -5)

    def test_get_corner_max(self):
        center_pnt = gp_Pnt(0, 0, 0)
        gap = 10
        corner_max = self.box.get_corner_max(center_pnt, gap)
        self.assertEqual(corner_max.X(), 5)
        self.assertEqual(corner_max.Y(), 5)
        self.assertEqual(corner_max.Z(), 5)

    def test_get_center_pnt(self):
        corner_min = gp_Pnt(-5, -5, -5)
        corner_max = gp_Pnt(5, 5, 5)
        center_pnt = self.box.get_center_pnt(corner_min, corner_max)
        self.assertEqual(center_pnt.X(), 0)
        self.assertEqual(center_pnt.Y(), 0)
        self.assertEqual(center_pnt.Z(), 0)

    def test_get_gap(self):
        corner_min = gp_Pnt(-5, -5, -5)
        corner_max = gp_Pnt(5, 5, 5)
        gap = self.box.get_gap(corner_min, corner_max)
        self.assertEqual(gap, 10)

class TestNode(unittest.TestCase):
    def setUp(self):
        self.node = Node()

    def test_copy_from(self):
        other, other.parent = Node(i=1, j=2, k=3), Node(i=4, j=5, k=6)
        other.f, other.g = 1.5, 2.5
        other.is_start_node = True
        other.is_goal_node = True
        other.is_obstacle = True

        self.node.copy_from(other)

        self.assertEqual(self.node.i, other.i)
        self.assertEqual(self.node.j, other.j)
        self.assertEqual(self.node.k, other.k)
        self.assertEqual(self.node.f, other.f)
        self.assertEqual(self.node.g, other.g)
        self.assertEqual(self.node.parent, other.parent)
        self.assertEqual(self.node.is_start_node, other.is_start_node)
        self.assertEqual(self.node.is_goal_node, other.is_goal_node)
        self.assertEqual(self.node.is_obstacle, other.is_obstacle)

    def test_reset(self):
        self.node.i, self.node.j, self.node.k = 1, 2, 3
        self.node.parent = Node(i=0, j=0, k=0)
        self.node.f, self.node.g = 1.5,2.5
        self.node.is_start_node = True
        self.node.is_goal_node = False

        self.node.reset()
        # target properties should be reset 
        self.assertIsNone(self.node.parent)
        self.assertEqual(self.node.f, 0.0)
        self.assertEqual(self.node.g, 0.0)
        # other properties should remain the same
        self.assertEqual(self.node.i, 1)
        self.assertEqual(self.node.j, 2)
        self.assertEqual(self.node.k, 3)
        self.assertTrue(self.node.is_start_node)
        self.assertFalse(self.node.is_goal_node)

    def test_eq(self):
        node1 = Node(i=1, j=2, k=3)
        node2 = Node(i=1, j=2, k=3)
        node3 = Node(i=4, j=5, k=6)

        self.assertEqual(node1, node2)
        self.assertNotEqual(node1, node3)

    def test_hash(self):
        node = Node(i=1, j=2, k=3)
        expected_hash = hash((node.i, node.j, node.k))

        self.assertEqual(hash(node), expected_hash)

    def test_str(self):
        self.node.i = 1
        self.node.j = 2
        self.node.k = 3
        self.node.g = 1.5
        self.node.f = 2.5

        expected_str = "voxel Node (1,2,3), g = 1.5, f = 2.5"

        self.assertEqual(str(self.node), expected_str)

    def test_lt(self):
        node1 = Node()
        node1.f = 1.0
        node2 = Node()
        node2.f = 2.0

        self.assertLess(node1, node2)

class TestGrids3D(unittest.TestCase):
    def setUp(self):
        corner_min = gp_Pnt(-5, -5, -5)
        corner_max = gp_Pnt(5, 5, 5)
        map_size = 10
        self.grid = Grids3D(corner_min, corner_max, map_size)

    def test_reset_nodes(self):
        self.grid.set_start_node(self.grid[0, 0, 0])
        self.grid.set_goal_node(self.grid[9, 9, 9])

        for node in self.grid:
            node.f = 1.0
            node.g = 2.0
            node.parent = Node()

        self.grid.reset_nodes()

        for node in self.grid:
            self.assertEqual(node.f, 0.0)
            self.assertEqual(node.g, 0.0)
            self.assertIsNone(node.parent)

    def test_set_start_node(self):
        start_node = self.grid[0, 0, 0]
        self.grid.set_start_node(start_node)
        
        self.assertEqual(self.grid.start_node, start_node)
        self.assertTrue(start_node.is_start_node)
        
        self.grid.set_start_node(1,1,1) 
        start_node = self.grid[1,1,1]   
        self.assertEqual(self.grid.start_node, start_node)
        self.assertTrue(start_node.is_start_node)

    def test_set_goal_node(self):
        goal_node = self.grid[9, 9, 9]
        self.grid.set_goal_node(goal_node)

        self.assertEqual(self.grid.goal_node, goal_node)
        self.assertTrue(goal_node.is_goal_node)
        
        self.grid.set_goal_node(1,1,1)
        goal_node = self.grid[1,1,1]    
        self.assertEqual(self.grid.goal_node, goal_node)
        self.assertTrue(goal_node.is_goal_node)

    def test_iter(self):
        count = 0
        for node in self.grid:
            expected_str = f"voxel Node ({node.i},{node.j},{node.k}), g = {node.g}, f = {node.f}"   
            self.assertEqual(str(node), expected_str)
            count += 1

        self.assertEqual(count, len(self.grid) ** 3)

    def test_getitem(self):
        node = self.grid[5, 5, 5]
        self.assertIsInstance(node, Node)
        self.assertEqual(node.i, 5)
        self.assertEqual(node.j, 5)
        self.assertEqual(node.k, 5)

    def test_len(self):
        self.assertEqual(len(self.grid), 10)

    def test_save_grid_map(self):
        file_path = "test_grid_map.npy"
        self.grid.save_grid_map(file_path)

        # Check if the file exists
        import os
        self.assertTrue(os.path.exists(file_path))

        # Clean up
        os.remove(file_path)

    def test_load_grid_map(self):
        file_path = "test_grid_map.npy"
        grids = Grids3D(
            corner_max=gp_Pnt(5, 5, 5),
            corner_min=gp_Pnt(-5, -5, -5),
            map_size=3
        )
        grids[1,1,1].is_obstacle = True
        grids.save_grid_map(file_path)
        loaded_grids = Grids3D(
            corner_max=gp_Pnt(5, 5, 5),
            corner_min=gp_Pnt(-5, -5, -5),
            map_size=3
        )
        loaded_grids.load_grid_map(file_path)

        for i in range(3):
            for j in range(3):
                for k in range(3):
                    node = loaded_grids[i, j, k]
                    if i == 1 and j == 1 and k == 1:
                        self.assertTrue(node.is_obstacle)
                    else:
                        self.assertFalse(node.is_obstacle)

        # clean up
        os.remove(file_path)
        
        
if __name__ == '__main__':
    unittest.main()
