import csv
import numpy as np
import pymesh
import cv2
from scipy.spatial.transform import Rotation as R


def post_process_depth(depth):
	for i in range(depth.shape[0]):
		for k in range(depth.shape[1]):
			if depth[i, k]==-1.:
				depth[i, k] = 0.0
			else:
				continue
	depth = np.array(depth).astype("uint16")
	return depth


def convert_depth2depthmap(depth):
	return cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.48), cv2.COLORMAP_JET)


def save_depth_as_csv(depth, file_name):
	f = open('./data/target/{}.csv'.format(file_name), 'w')
	writer = csv.writer(f, lineterminator='\n')
	writer.writerows(depth)
	f.close()

	f = open('../debug/ICP-Energy/data/{}.csv'.format(file_name), 'w')
	writer = csv.writer(f, lineterminator='\n')
	writer.writerows(depth)
	f.close()

	f = open('../debug/vis_ICP-Energy/data/{}.csv'.format(file_name), 'w')
	writer = csv.writer(f, lineterminator='\n')
	writer.writerows(depth)
	f.close()



def save_pose(pose):
	np.save('data/target/target_pose', pose)
	np.save('../debug/ICP-Energy/data/target_pose', pose)


def get_homogeneous_matrix(rotation, translation):
	T = np.eye(4)
	T[0, 0] = rotation[0, 0]; T[0, 1] = rotation[0, 1]; T[0, 2] = rotation[0, 2];
	T[1, 0] = rotation[1, 0]; T[1, 1] = rotation[1, 1]; T[1, 2] = rotation[1, 2];
	T[2, 0] = rotation[2, 0]; T[2, 1] = rotation[2, 1]; T[2, 2] = rotation[2, 2];
	T[0, 3] = translation[0];
	T[1, 3] = translation[1];
	T[2, 3] = translation[2];
	print(translation)
	return T


def load_obj_file(file):
	template_shape = pymesh.load_mesh(file)
	verts = template_shape.vertices
	faces = template_shape.faces
	return np.copy(verts), np.copy(faces)


def load_cam_params():
	translation =  np.array([ 16.800812, 104.01688834, 586.28166456])
	rotation = np.array([[ 1.        ,  0.        ,  0.        ],
					   [ 0.        ,  -0.93969262, 0.34202021],
					  [ 0.        ,  -0.34202021, -0.93969262]])
	
	T = get_homogeneous_matrix(rotation, translation)
	camera_intrinsic = np.array([612., 612., 310., 243.])

	return T, camera_intrinsic

def get_target_pose(init_pose, ax, degree, add_trans):
	init_rotation = init_pose[:3, :3]
	init_translation =  init_pose[:, 3:][:3].T.reshape(3)
	# corrdinate system is z up.
	add_rotvec = R.from_euler(ax, degree, degrees=True).as_rotvec()
	
	
	# rotation
	r = R.from_dcm(init_rotation)
	rot_vec = r.as_rotvec()
	target_rot_vec = np.copy(rot_vec)
	target_rot_vec = target_rot_vec + add_rotvec
	target_rotation  = R.from_rotvec(target_rot_vec).as_dcm()
	# translation
	target_tanslation = np.copy(init_translation)
	target_tanslation = target_tanslation + add_trans

	target_T = get_homogeneous_matrix(target_rotation, target_tanslation)

	return target_T


def get_rotated_verts(verts, pose):
	N = verts.shape[0]
	R = pose[:3, :3]
	trans = pose[:, 3:][:3].T.reshape(3)

	rotated_verts = verts @ R.T
	for i in range(N):
		rotated_verts[i, 0] = rotated_verts[i, 0] + trans[0]
		rotated_verts[i, 1] = rotated_verts[i, 1] + trans[1]
		rotated_verts[i, 2] = rotated_verts[i, 2] + trans[2]

	return rotated_verts

"""
source_verts = get_rotated_verts(model_verts, init_pose)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(source_verts)
o3d.io.write_point_cloud("source.ply", pcd)

target_verts = get_rotated_verts(model_verts, target_pose)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(target_verts)
o3d.io.write_point_cloud("target.ply", pcd)
"""
