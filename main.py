import numpy as np
import pyrender
import matplotlib.pyplot as plt
from utils import *
import cv2
import open3d as o3d


class Depth_render():
	def __init__(self, faces, cam_intr, img_size=[480, 680]):
		self.faces = faces
		self.cam_intr = cam_intr
		self.img_size = np.array([img_size[0], img_size[1]], dtype="int32")

	def render(self, verts, pose):
		N = verts.shape[0]
		R = pose[:3, :3]
		trans = pose[:, 3:][:3].T.reshape(3)

		rotated_verts = verts @ R.T
		for i in range(N):
			rotated_verts[i, 0] = rotated_verts[i, 0] + trans[0]
			rotated_verts[i, 1] = rotated_verts[i, 1] + trans[1]
			rotated_verts[i, 2] = rotated_verts[i, 2] + trans[2]

		depth = pyrender.render(rotated_verts, self.faces, self.cam_intr, self.img_size)
		depth = post_process_depth(depth)
		depth_map = convert_depth2depthmap(depth)
		
		return depth, depth_map


if __name__ == '__main__':
	# Load data
	file = "./data/bunny.obj"
	model_verts, model_faces = load_obj_file(file)
	init_pose, cam_intr = load_cam_params()

	# Init renderer
	depth_render = Depth_render(model_faces, cam_intr)
	
	# Get target pose
	axis = "z"
	degree = 10
	add_trans = np.array([0., 0., 10.])
	target_pose = get_target_pose(init_pose, axis, degree, add_trans)
	target_verts = get_rotated_verts(model_verts, target_pose)
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(target_verts)
	o3d.io.write_point_cloud("../../point_to_plane/Open3D/examples/Python/Basic/target2.ply", pcd)

	# Render depth
	depth, depth_map = depth_render.render(model_verts, init_pose)
	target_depth, target_depth_map = depth_render.render(model_verts, target_pose)


	# Save depth 
	save_flag = True
	if save_flag == True:
		save_depth_as_csv(target_depth, "target_depth")
		save_pose(target_pose)

	# Visualize depth map
	cv2.imshow('depth',depth_map)
	cv2.imshow('target_depth_map',target_depth_map)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


	

	












