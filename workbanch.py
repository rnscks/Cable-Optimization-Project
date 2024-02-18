from typing import List 
from OCC.Core.gp import gp_Pnt
from scipy.spatial import KDTree
import numpy as np


def remove_duplicate_points(pnts: gp_Pnt) -> List[gp_Pnt]:    
    unique_pnts = set()
    not_duplicate_pnts: List[gp_Pnt] = [] 
    for pnt in pnts:
        if (pnt.X(), pnt.Y(), pnt.Z()) in unique_pnts:  
            continue
        else:
            not_duplicate_pnts.append(pnt)  
            unique_pnts.add((pnt.X(), pnt.Y(), pnt.Z()))    
    return not_duplicate_pnts 

def optimize_path_using_previous_point(pnts: gp_Pnt, threshold=0.001):
    pnts = remove_duplicate_points(pnts)    
    pnts = [(pnt.X(), pnt.Y(), pnt.Z()) for pnt in pnts]

    tree = KDTree(pnts)  
    optimized_pnts: gp_Pnt = []
    skipped_indices = set()

    for i, pnt in enumerate(pnts):
        if i in skipped_indices:
            continue        
        optimized_pnts.append(gp_Pnt(*pnt)) 
        distances, indices = tree.query(pnt, k=2) 
        for dist, idx in zip(distances[1:], indices[1:]):
            if dist < threshold:
                skipped_indices.add(idx)

    return optimized_pnts


pnts = [gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0.0, 0.0, 0.0), gp_Pnt(0,0,1), gp_Pnt(0,0,1), gp_Pnt(0,0,0.00001)]  
rets = optimize_path_using_previous_point(pnts)   

for ret in rets:
    print(ret.X(), ret.Y(), ret.Z())    
