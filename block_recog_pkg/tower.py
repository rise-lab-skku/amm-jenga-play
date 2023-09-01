import open3d as o3d
from .utils import *
from os.path import join
from tf_conversions import *

class Commander:
    def __init__(self,pcds):
        self.mesh = o3d.io.read_triangle_mesh(
            join(pkg_path,TOWER_MODEL)
        )  # Jenga Tower Mesh
        self.mesh.compute_vertex_normals()
        self.pcd_real=o3d.geometry.PointCloud()
        for pcd in pcds:
            self.pcd_real+=pcd
        # Mesh to Pointcloud
        self.pcd_ideal = copy.deepcopy(
            self.mesh.sample_points_uniformly(number_of_points=len(self.pcd_real.points))
        )

    def get_transform(self):
        self.pcd_real.transform(INIT_TF)
        move = np.array(
            self.pcd_ideal.get_oriented_bounding_box().get_center()
            - self.pcd_real.get_oriented_bounding_box().get_center()
        ).tolist()


        reg_p2p = o3d.pipelines.registration.registration_icp(
            self.pcd_real,
            self.pcd_ideal,
            10,
            toMatrix(fromTf(move+toTf(fromMatrix(INIT_TF))[1])),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000),
        )

        return reg_p2p.transformation