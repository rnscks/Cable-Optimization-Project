import os
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepTools import breptools
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.Graphic3d import Graphic3d_Vec3d
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.Prs3d import prs3d
from OCC.Core.Precision import precision
from OCC.Core.Bnd import Bnd_Box 
from OCC.Core.TopoDS import TopoDS_Compound
from OCC.Extend.DataExchange import STEPControl_Reader
import torch
from pytorch3d.structures import Meshes
from pytorch3d.ops import sample_points_from_meshes


class STPFileReader:
    @classmethod  
    def read_stp_file_by_occ(cls, file_name:str) -> TopoDS_Compound:
        dir_name = "./brep_model"
        file_path = os.path.join(dir_name, file_name)  
        step_reader = STEPControl_Reader()
        step_reader.ReadFile(file_path)
        step_reader.TransferRoots()  
        
        return step_reader.Shape()

class ShapeToMeshConvertor:
    @classmethod    
    def convert_to_point_clouds(cls, brep_shape: TopoDS_Shape, number_of_points: int = 50000): 
        cls.init_brep_mesh(brep_shape)
        explorer = TopExp_Explorer()
        explorer.Init(brep_shape, TopAbs_FACE)
        
        mesh_faces = []
        mesh_vertices = []

        while (explorer.More()):
            face = explorer.Current()
            cls.init_triangle_mesh(mesh_vertices, mesh_faces, face)
            explorer.Next()
        
        torch_points = torch.tensor(mesh_vertices, dtype=torch.float32).view(1, -1, 3)  
        torch_faces = torch.tensor(mesh_faces, dtype=torch.int64).view(1, -1, 3)    
        torch_mesh = Meshes(torch_points, torch_faces)  
        
        sampled_points = sample_points_from_meshes(torch_mesh, number_of_points)

        return sampled_points[0].tolist()
    
    @classmethod    
    def init_brep_mesh(cls, shape: TopoDS_Shape) -> BRepMesh_IncrementalMesh:    
        def calculate_angle_deflection() -> float:
            angle_deflection_max, angle_deflection_min = 0.8, 0.2
            quality = 5.0

            angle_deflection_gap = (angle_deflection_max - angle_deflection_min) / 10
            angle_deflection = \
                max(angle_deflection_max - (quality * angle_deflection_gap), angle_deflection_min)
            return angle_deflection

        def calculate_line_deflection(bnd_box: Bnd_Box) -> float:
            gvec1 = Graphic3d_Vec3d(*bnd_box.CornerMin().Coord())
            gvec2 = Graphic3d_Vec3d(*bnd_box.CornerMax().Coord())
            deflection = prs3d.GetDeflection(gvec1, gvec2, 0.001)

            line_deflaction = max(deflection, precision.Confusion())

            return line_deflaction
        
        bnd_box = Bnd_Box()
        brepbndlib.Add(shape, bnd_box)
        
        angle_deflection = calculate_angle_deflection()
        line_deflaction = calculate_line_deflection(bnd_box)
        breptools.Clean(shape)
        bmesh = BRepMesh_IncrementalMesh(shape, line_deflaction, False, angle_deflection, False)
        return bmesh

    @classmethod    
    def init_triangle_mesh(cls, mesh_vertices, mesh_faces, face) -> None:
        before_vertices_number = len(mesh_vertices)
        loc = TopLoc_Location()
        
        poly = BRep_Tool.Triangulation(face, loc)
        if (poly is None):
            return
        
        node_numbers = poly.NbNodes()
        nodes = poly.InternalNodes()

        for node_number in range(node_numbers):
            pnt = nodes.Value(node_number).Transformed(loc.Transformation())
            mesh_vertices.append(pnt.Coord())

        triangles = poly.InternalTriangles()
        mesh_triangle_indicies = \
            list(range(before_vertices_number, len(mesh_vertices) + before_vertices_number))
            
        for triangle_number in range(triangles.Lower(), triangles.Upper() + 1):
            triangle_indicies = triangles.Value(triangle_number).Get()
            if len(triangle_indicies) == 0:
                continue
            
            triangle_index = \
                [mesh_triangle_indicies[index - 1] for index in triangle_indicies]
            mesh_faces.append(triangle_index)
        return


import random

from OCC.Core.gp import gp_Pnt
from OCC.Core.STEPControl import STEPControl_Writer
from OCC.Core.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Sewing


class STPFileWriter:
    def __init__(self, file_name: str) -> None:        
        self.file_name = file_name
        self.compound_builder = BRepBuilderAPI_Sewing()
    
    
    def add_compound(self, shape: TopoDS_Shape) -> None:
        self.compound_builder.Add(shape)
        return  

    def save(self):
        self.compound_builder.Perform()
        compound = self.compound_builder.SewedShape()

        # STP 파일 작성
        stp_writer = STEPControl_Writer()
        stp_writer.Transfer(compound, STEPControl_AsIs)
        status = stp_writer.Write(self.file_name)
        if status == IFSelect_RetDone:
            print(f"STP 파일이 성공적으로 저장되었습니다: {self.file_name}")
        else:
            print("STP 파일 저장 중 오류가 발생하였습니다.")