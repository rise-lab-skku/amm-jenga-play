import open3d as o3d
from .utils import *
from tf_conversions import *
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from copy import deepcopy as dcp

class Commander:
    def __init__(self,file,num):
        self.mesh = o3d.io.read_triangle_mesh(file)
        self.mesh.compute_vertex_normals()
        self.pcd = o3d.geometry.sample_points_uniformly(self.mesh, num)

    def get_transform(self,pcd):
        self.pcd_real.transform(INIT_TF)
        move = np.array(
            self.pcd.get_oriented_bounding_box().get_center()
            - pcd.get_oriented_bounding_box().get_center()
        ).tolist()


        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd,
            self.pcd,
            10,
            toMatrix(fromTf(move+toTf(fromMatrix(INIT_TF))[1])),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000),
        )

        return reg_p2p.transformation
    
    def build(self):
        self.origin=toMsg(fromTf(listener.lookupTransform('world','mesh')))
        self.bases=[listener.transformPoint('mesh',PointStamped(Header(frame_id='world'),Point(x,y,0))).point for x,y in ((BLOCK_X/2,BLOCK_Y/2),(BLOCK_X/-2,BLOCK_Y/2),(BLOCK_X/-2,BLOCK_Y/-2),(BLOCK_X/2,BLOCK_Y/-2))]
        # self.map=

        # pcd=o3d.geometry.PointCloud()
        # Get axis aligned bounding box's extents [x, y, z]
        box_extent = (
            pcd.get_axis_aligned_bounding_box().get_extent()
        )

        # Get axis aligned bounding box's center coordinate [x, y, z]
        aabb_center_coordinate = np.array(
            pcd.get_axis_aligned_bounding_box().get_box_points()
        ).mean(axis=0)

        # AABB center coordinate x, y, z
        x_mean = aabb_center_coordinate[0]
        y_mean = aabb_center_coordinate[1]
        z_mean = aabb_center_coordinate[2]

        floors = int(z_mean // 0.015)
        true_z = floors * 0.015 + 0.0075
        center = Point(dcp(x_mean), dcp(y_mean), dcp(true_z))
        target = dcp(center)

        # Determine whether to push or pull the block and calculate the target coordinates
        if box_extent[1] > 0.070:
            # print("PULL DIRECTION : X")
            push = False
            target.x += 0.12
            tower_map[floors][0] = color_idx

        elif box_extent[0] > 0.070:
            # print("PULL DIRECTION : Y")
            push = False
            target.y += 0.12
            tower_map[floors][0] = color_idx

        elif abs(aabb_center_coordinate[0]) < 0.010 and box_extent[1] < 0.015:
            # print("PUSH DIRECTION : Y or -Y")
            push = True
            center.y += -0.075 / 2
            target.y += 0.12
            tower_map[floors][1] = color_idx

        elif abs(aabb_center_coordinate[1]) < 0.010 and box_extent[0] < 0.015:
            # print("PUSH DIRECTION : X or -X")
            push = True
            center.x += -0.075 / 2
            target.x += 0.12
            tower_map[floors][1] = color_idx

        elif box_extent[1] < 0.015:
            # print("PULL DIRECTION : -X")
            push = False
            center.y += -0.075 / 2
            target.x += -0.12
            tower_map[floors][2] = color_idx

        elif box_extent[0] < 0.015:
            # print("PULL DIRECTION : -Y")
            push = False
            center.x += -0.075 / 2
            target.y += -0.120
            tower_map[floors][2] = color_idx

        else:
            print("Cannot Find Box Coordinates")
