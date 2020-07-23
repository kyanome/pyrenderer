import pymesh
import numpy as np
import pyrender
import matplotlib.pyplot as plt
from PIL import Image

# load obj file
obj_name = "bunny"
file = "bunny.obj"
template_shape = pymesh.load_mesh(file)
verts = template_shape.vertices
faces = template_shape.faces


init_trans =  np.array([ 16.800812, 104.01688834, 586.28166456])
init_R = np.array([[ 1.        ,  0.        ,  0.        ],
      			   [ 0.        ,  -0.93969262, 0.34202021],
       			  [ 0.        ,  -0.34202021, -0.93969262]])


rotate_verts = verts @ init_R.T
for i in range(len(verts)):
	rotate_verts[i, 0] = rotate_verts[i, 0] + init_trans[0]
	rotate_verts[i, 1] = rotate_verts[i, 1] + init_trans[1]
	rotate_verts[i, 2] = rotate_verts[i, 2] + init_trans[2]

c_verts = np.copy(rotate_verts)
c_faces = np.copy(faces)
cam_intr = np.array([612., 612., 310., 243.])
img_size = np.array([480, 640], dtype="int32")
depth = pyrender.render(c_verts, c_faces, cam_intr, img_size)

for i in range(depth.shape[0]):
	for k in range(depth.shape[1]):
		if depth[i, k]==-1.:
			depth[i, k] = 0.0
		else:
			print(depth[i, k])
		

import csv

f = open('some.csv', 'w')

writer = csv.writer(f, lineterminator='\n')
writer.writerows(depth)

f.close()

#import pdb; pdb.set_trace()
pil_img_gray = Image.fromarray(depth)
pil_img_gray = pil_img_gray.convert("L")
print(pil_img_gray.mode)
pil_img_gray.save('gray.jpg')
plt.imshow(depth, cmap="gray")
plt.show()


