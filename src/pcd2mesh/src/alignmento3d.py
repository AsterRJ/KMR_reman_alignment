#!/usr/bin/env python

import rospy
import pcl
import numpy as np
from sensor_msgs.msg import PointCloud2
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import threading
#from pcl import PointCloud, IterativeClosestPoint

class PointCloudAlignment:
    def __init__(self):
        self.stl_mesh = pcl.load("path/to/stl/file.stl")  # Load STL file
        self.icp = pcl.IterativeClosestPoint()  # ICP object for alignment

    def point_cloud_callback(self, ros_pointcloud):
        # Convert ROS PointCloud2 to PCL PointCloud
        pcl_pc = pcl.PointCloud()
        pcl_pc.from_list([[p.x, p.y, p.z] for p in ros_pointcloud])

        # Perform alignment
        converged, estimate, fitness = self.icp.align(pcl_pc, self.stl_mesh)
        if converged:
            aligned_cloud = self.icp.getFinalTransformation() * pcl_pc
            self.visualize_result(self.stl_mesh, aligned_cloud)

    def visualize_result(self, stl_mesh, aligned_cloud):
        visualization_thread = threading.Thread(target=self.show_visualization, args=(stl_mesh, aligned_cloud))
        visualization_thread.daemon = True
        visualization_thread.start()

    def show_visualization(self, stl_mesh, aligned_cloud):
        # Visualize the STL mesh and aligned point cloud using Matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot STL mesh (example - adjust based on STL data format)
        # plot_stl_mesh(ax, stl_mesh)

        # Plot aligned point cloud (example - adjust based on PCL PointCloud format)
        points = np.asarray(aligned_cloud)[:, :3]
        ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='.')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Aligned Point Cloud and STL Mesh')

        plt.show()

def main():
    rospy.init_node('point_cloud_aligner_node', anonymous = True)
    aligner = PointCloudAlignment()

    # Subscribe to PointCloud2 messages
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, aligner.point_cloud_callback)

    rospy.spin()

if __name__ == '__main__':
    main()





# #!/usr/bin/env python3

# import copy

# import numpy as np
# #import open3d as o3d
# import pcl
# from ctypes import * # convert float to uint32

# import rospy
# from std_msgs.msg import Header
# from sensor_msgs.msg import PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2

# import threading
# #from pcl import visualization

# # def view_mesh(mesh):
# #     visual = visualization.CloudViewing()
# #     visual.ShowMonochromeCloud(mesh)
# #     while not visual.WasStopped():
# #         visual.SpinOnce(100)

# # def visualize_mesh_thread(mesh):
# #     visualization_thread = threading.Thread(target=view_mesh, args=(mesh,))
# #     visualization_thread.daemon = True  # Daemonize the thread so it won't block program exit
# #     visualization_thread.start()


# # def draw_registration_result(source, target, transformation):
# #     source_temp = copy.deepcopy(source)
# #     target_temp = copy.deepcopy(target)
# #     source_temp.paint_uniform_color([1, 0.706, 0])
# #     target_temp.paint_uniform_color([0, 0.651, 0.929])
# #     source_temp.transform(transformation)
# #     o3d.visualization.draw_geometries([source_temp, target_temp])


# def preprocess_point_cloud(pcd, voxel_size):
#     print(":: Downsample with a voxel size %.3f." % voxel_size)
#     pcd_down = pcd.voxel_down_sample(voxel_size)

#     radius_normal = voxel_size * 2
#     print(":: Estimate normal with search radius %.3f." % radius_normal)
#     pcd_down.estimate_normals(
#         o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

#     radius_feature = voxel_size * 5
#     print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
#     pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
#         pcd_down,
#         o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
#     return pcd_down, pcd_fpfh


# def execute_global_registration(source_down, target_down, source_fpfh,
#                                 target_fpfh, voxel_size):
#     distance_threshold = voxel_size * 1.5
#     print(":: RANSAC registration on downsampled point clouds.")
#     print("   Since the downsampling voxel size is %.3f," % voxel_size)
#     print("   we use a liberal distance threshold %.3f." % distance_threshold)
#     result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#         source_down, target_down, source_fpfh, target_fpfh, True,
#         distance_threshold,
#         o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
#         3, [
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
#                 0.9),
#             o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
#                 distance_threshold)
#         ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
#     return result


# def main():
#     voxel_size = 0.01
#     print(":: Load two mesh.")
#     target_mesh = o3d.io.read_triangle_mesh('/home/arjp/DISASSEMBLY_PROJECT/src/test_part_model/4.stl')
#     source_mesh = copy.deepcopy(target_mesh)
#     #source_mesh.rotate(source_mesh.get_rotation_matrix_from_xyz((np.pi / 4, 0, np.pi / 4)), center=(0, 0, 0))
#     #source_mesh.translate((0, 0.05, 0))
#     #draw_registration_result(target_mesh, source_mesh, np.identity(4))

#     print(":: Sample mesh to point cloud")
#     target = target_mesh.sample_points_uniformly(1000)
#     source = source_mesh.sample_points_uniformly(1000)
#     #draw_registration_result(source, target, np.identity(4))

#     source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
#     target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
#     result_ransac = execute_global_registration(source_down, target_down,
#                                                 source_fpfh, target_fpfh,
#                                                 voxel_size)
#     print(result_ransac)
#     #draw_registration_result(source_down, target_down, result_ransac.transformation)
#     #draw_registration_result(source_mesh, target_mesh, result_ransac.transformation)

# FIELDS_XYZ = [
#     PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
# ]
# FIELDS_XYZRGB = FIELDS_XYZ + \
#     [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# # Bit operations
# BIT_MOVE_16 = 2**16
# BIT_MOVE_8 = 2**8
# convert_rgbUint32_to_tuple = lambda rgb_uint32: (
#     (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
# )
# convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
#     int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
# )



# def convertCloudFromRosToOpen3d(ros_cloud):
    
#     # Get cloud data from ros_cloud
#     field_names=[field.name for field in ros_cloud.fields]
#     cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

#     # Check empty
#     open3d_cloud = o3d.PointCloud()
#     if len(cloud_data)==0:
#         print("Converting an empty cloud")
#         return None

#     # Set open3d_cloud
#     if "rgb" in field_names:
#         IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
#         # Get xyz
#         xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

#         # Get rgb
#         # Check whether int or float
#         if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
#             rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
#         else:
#             rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

#         # combine
#         open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))
#         #open3d_cloud.colors = open3d.Vector3dVector(np.array(rgb)/255.0)
#     else:
#         xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
#         open3d_cloud.points = o3d.Vector3dVector(np.array(xyz))

#     # return
#     return open3d_cloud

# def convert_ros_pointcloud_to_pcl(ros_pointcloud):
#     rospy.loginfo("This is an information message3.")
#     # Initialize PCL PointCloud object
#     pcl_pointcloud = pcl.PointCloud()

#     # Convert ROS PointCloud2 to PCL PointCloud
#     gen = pc2.read_points(ros_pointcloud, skip_nans=True)
#     points = []
#     for p in gen:
#         points.append([p[0], p[1], p[2]])

#     pcl_pointcloud.from_list(points)

#     return pcl_pointcloud

# def align_clouds(pcd_):
#     stl_data = pcl.load('/home/arjp/DISASSEMBLY_PROJECT/src/test_part_model/4.stl')
#     #visualize_mesh_thread(stl_data)
#     reg = pcl.registration.GeneralizedIterativeClosestPoint()
#     reg.setMaxCorrespondenceDistance(0.05) # Example parameter setting
#     reg.setMaximumIterations(50)
#     reg.setMaxCorrespondenceDistance(0.05) # Example parameter setting
#     reg.setMaximumIterations(50) # Example parameter setting
#     point_cloud = convert_ros_pointcloud_to_pcl(pcd_)
#     transform, converged, estimate, fitness = reg.align(point_cloud, stl_data)

#     if converged:
#         # Perform local registration
#         # ...

#         # Retrieve transformation
#         transformation_matrix = transform.getTransformationMatrix()

#         # Apply transformation to STL mesh
#         aligned_stl_data = stl_data.transform(transformation_matrix)
#     else:
#         print("Registration did not converge.")



#     # voxel_size = 0.01
#     # print(":: Load two mesh.")
#     # target_mesh = o3d.io.read_triangle_mesh('/home/arjp/DISASSEMBLY_PROJECT/src/test_part_model/4.stl')
#     # #source_mesh = copy.deepcopy(target_mesh)
#     # #source_mesh.rotate(source_mesh.get_rotation_matrix_from_xyz((np.pi / 4, 0, np.pi / 4)), center=(0, 0, 0))
#     # #source_mesh.translate((0, 0.05, 0))
#     # #draw_registration_result(target_mesh, source_mesh, np.identity(4))

#     # print(":: Sample mesh to point cloud")
#     # target = target_mesh.sample_points_uniformly(1000)
#     # #source = source_mesh.sample_points_uniformly(1000)
#     # #draw_registration_result(source, target, np.identity(4))

#     # source = convertCloudFromRosToOpen3d(pcd_)

#     # source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
#     # target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
#     # result_ransac = execute_global_registration(source_down, target_down,
#     #                                             source_fpfh, target_fpfh,
#     #                                             voxel_size)
#     # print(result_ransac)
#     #draw_registration_result(source_down, target_down, result_ransac.transformation)
#     #draw_registration_result(source_mesh, target_mesh, result_ransac.transformation)




# if __name__ == '__main__':
#     #main()
#     rospy.init_node('alignment_node', anonymous = True)
#     rospy.loginfo("This is an information message.")
#     rospy.Subscriber("/camera/depth/color/points", PointCloud2,align_clouds)
#     rospy.loginfo("This is an information message2.")
#     rate = rospy.Rate(10)
#     while not rospy.is_shutdown():
#         rate.sleep()




