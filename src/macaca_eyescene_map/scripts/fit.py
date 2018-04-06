#! /usr/bin/env python
import sys
import yaml
from yaml import load, dump
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF
import numpy as np

point3d_array = []
left_eye_point2d_array = []
right_eye_point2d_array = []
left_scene_point2d_array = []

def read_calib_data(filename):
    with open(filename) as stream:
        for i in [0,1]:
            _ = stream.readline()
        data = yaml.load(stream)
    count = data['count']
    for i in range(count):
        point3d = data['point3d_' + str(i)]
        left_eye_point2d = data['left_eye_point2d_' + str(i)]
        right_eye_point2d = data['right_eye_point2d_' + str(i)]
        left_scene_point2d = data['left_scene_point2d_' + str(i)]
        point3d_array.append(point3d)
        left_eye_point2d_array.append(left_eye_point2d)
        right_eye_point2d_array.append(right_eye_point2d)
        left_scene_point2d_array.append(left_scene_point2d)
    print count


length_scale = [2, 2, 2, 2]
length_scale_bounds = [(1e-05, 100000.0), (1e-05, 100000.0), (1e-05, 100000.0), (1e-05, 100000.0)]

gp_kernel_lsx = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
gp_kernel_lsy = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
gp_kernel_rsx = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
gp_kernel_rsy = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
gp_kernel_x = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
gp_kernel_y = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)
gp_kernel_z = RBF(length_scale=length_scale, length_scale_bounds=length_scale_bounds)

gpr_lsx = GaussianProcessRegressor(kernel=gp_kernel_lsx)
gpr_lsy = GaussianProcessRegressor(kernel=gp_kernel_lsy)
gpr_rsx = GaussianProcessRegressor(kernel=gp_kernel_rsx)
gpr_rsy = GaussianProcessRegressor(kernel=gp_kernel_rsy)
gpr_x = GaussianProcessRegressor(kernel=gp_kernel_x)
gpr_y = GaussianProcessRegressor(kernel=gp_kernel_y)
gpr_z = GaussianProcessRegressor(kernel=gp_kernel_z)

def gaze_estimation_model_fitting():
    gpr_lsx.fit(np.concatenate((left_eye_point2d_array, right_eye_point2d_array), axis=1), left_scene_point2d_array[:,0])
    gpr_lsy.fit(np.concatenate((left_eye_point2d_array, right_eye_point2d_array), axis=1), left_scene_point2d_array[:,1])
    gpr_x.fit(np.concatenate((left_eye_point2d_array, right_eye_point2d_array), axis=1), point3d_array[:,0])
    gpr_y.fit(np.concatenate((left_eye_point2d_array, right_eye_point2d_array), axis=1), point3d_array[:,1])
    gpr_z.fit(np.concatenate((left_eye_point2d_array, right_eye_point2d_array), axis=1), point3d_array[:,2])


def handle_gaze_estimation():
    lsx = gpr_lsx.predict(np.reshape(np.concatenate((left_eye_point2d_array[0,:], right_eye_point2d_array[0,:])),[1,4]), return_std=False)
    lsy = gpr_lsy.predict(np.reshape(np.concatenate((left_eye_point2d_array[0,:], right_eye_point2d_array[0,:])),[1,4]), return_std=False)
    x = gpr_x.predict(np.reshape(np.concatenate((left_eye_point2d_array[0,:], right_eye_point2d_array[0,:])),[1,4]), return_std=False)
    y = gpr_y.predict(np.reshape(np.concatenate((left_eye_point2d_array[0,:], right_eye_point2d_array[0,:])),[1,4]), return_std=False)
    z = gpr_z.predict(np.reshape(np.concatenate((left_eye_point2d_array[0,:], right_eye_point2d_array[0,:])),[1,4]), return_std=False)
    return [lsx, lsy,x,y,z]


if __name__ == '__main__':
    read_calib_data(sys.argv[1])
    point3d_array = np.array(point3d_array)
    left_eye_point2d_array = np.array(left_eye_point2d_array)
    right_eye_point2d_array = np.array(right_eye_point2d_array)
    left_scene_point2d_array = np.array(left_scene_point2d_array)

    gaze_estimation_model_fitting()
    print left_scene_point2d_array[0,:]
    print point3d_array[0,:]
    print (handle_gaze_estimation())


