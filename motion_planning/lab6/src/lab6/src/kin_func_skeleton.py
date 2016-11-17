#!/usr/bin/env python
"""Kinematic function skeleton code for Prelab 3.
Course: EE 106A, Fall 2015
Written by: Aaron Bestick, 9/10/14
Used by: EE106A, 9/11/15

This Python file is a code skeleton for Pre lab 3. You should fill in 
the body of the eight empty methods below so that they implement the kinematic 
functions described in the homework assignment.

When you think you have the methods implemented correctly, you can test your 
code by running "python kin_func_skeleton.py at the command line.

This code requires the NumPy and SciPy libraries. If you don't already have 
these installed on your personal computer, you can use the lab machines or 
the Ubuntu+ROS VM on the course page to complete this portion of the homework.
"""

import numpy as np
from math import sin, cos
import math
import scipy as sp
from scipy import linalg

np.set_printoptions(precision=4,suppress=True)

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    x = omega[0]
    y = omega[1]
    z = omega[2]
    return np.matrix([[0, -z, y],
                        [z, 0, -x],
                        [-y,x,0]])

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """
    
    return np.matrix([[cos(theta), -sin(theta)],
                        [sin(theta), cos(theta)]])

def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """
    if not omega.shape == (3,):
        raise TypeError('omega must be a 3-vector')
    
    w_1 = omega[0]
    w_2 = omega[1]
    w_3 = omega[2]
    w_mag = np.linalg.norm(omega)
    costheta = (1 - cos(theta * w_mag))
    sintheta = sin(theta * w_mag)
    Identity = np.eye(3)
    matrix1 = skew_3d(np.array([w_1,w_2,w_3]))
    matrix2 = matrix1**2
    matrix1 = matrix1 / w_mag
    matrix1 *= sintheta
    matrix2 = matrix2 / (w_mag**2)
    matrix2 *= costheta
    return Identity + matrix1 + matrix2

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    #YOUR CODE HERE
    x = xi[0]
    y = xi[1]
    omeg = xi[2]
    return np.matrix([[0, -omeg, x],
                        [omeg, 0, y],
                        [0, 0, 0]])


def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    v_x = xi[0]
    v_y = xi[1]
    v_z = xi[2]
    w_x = xi[3]
    w_y = xi[4]
    w_z = xi[5]
    return np.matrix([[0, -w_z, w_y, v_x],
                         [w_z, 0, -w_x, v_y],
                         [-w_y, w_x, 0, v_z],
                         [0,0,0,0]])

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    x = xi[0]
    y = xi[1]
    w = xi[2]
    R = rotation_2d(w*theta).A
    p = np.matrix([[1-cos(w*theta),sin(w*theta)],
                     [-sin(w*theta), 1- cos(w*theta)]])\
    * np.matrix([[0, -1], [1, 0]]) * np.matrix([[x/w],[y/w]])
    return np.matrix([[R[0][0], R[0][1], p[0]],
                        [R[1][0], R[1][1], p[1]],
                        [0, 0, 1]])

def RHScalc(rotation_matrix, w, v, theta):
    a = (np.eye(3)- rotation_matrix).dot(np.cross(w,v))
    b = w * w.dot(v) * theta
    return (a + b) / (np.linalg.norm(w)**2)

def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a
    joint displacement.

    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement

    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (6,):
        raise TypeError('xi must be a 6-vector')

    #YOUR CODE HERE
    vx = xi[0]
    vy = xi[1]
    vz = xi[2]
    wx = xi[3]
    wy = xi[4]
    wz = xi[5]
    v = np.array([vx,vy,vz]).T
    w = np.array([wx,wy,wz]).T
    expm = rotation_3d(w, theta).A
    RHS = RHScalc(expm, w, v, theta)
    return np.matrix([[expm[0][0], expm[0][1], expm[0][2], RHS[0]],
                         [expm[1][0], expm[1][1], expm[1][2], RHS[1]],
                         [expm[2][0], expm[2][1], expm[2][2], RHS[2]],
                         [0,0,0,1]])

def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6,N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape[0] == 6:
        print(xi.shape[0])
        raise TypeError('xi must be a 6xN')

    prod = np.eye(4)
    i = 0
    while i < len(theta):
        prod = prod.dot(homog_3d(xi[:,i].T, theta[i]))
        i+=1
    return prod

#-----------------------------Testing code--------------------------------------
#-------------(you shouldn't need to modify anything below here)----------------


if __name__ == "__main__":
    def array_func_test(func_name, args, ret_desired):
        ret_value = func_name(*args)
        if not isinstance(ret_value, np.ndarray):
            print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
        elif ret_value.shape != ret_desired.shape:
            print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
        elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
            print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
            print(ret_value)
            print("desired: {}".format(ret_desired))
        else:
            print('[PASS] ' + func_name.__name__ + '() returned the correct value!')
    print('Testing...')

    #Test skew_3d()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)

    #Test rotation_2d()
    arg1 = 2.658
    func_args = (arg1,)
    ret_desired = np.array([[-0.8853, -0.465 ],
                            [ 0.465 , -0.8853]])
    array_func_test(rotation_2d, func_args, ret_desired)

    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    #Test hat_2d()
    arg1 = np.array([2.0, 1, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3.,  0.,  1.],
                            [ 0.,  0.,  0.]])
    array_func_test(hat_2d, func_args, ret_desired)

    #Test hat_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(hat_3d, func_args, ret_desired)

    #Test homog_2d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.3924, -0.9198,  0.1491],
                            [ 0.9198, -0.3924,  1.2348],
                            [ 0.    ,  0.    ,  1.    ]])
    array_func_test(homog_2d, func_args, ret_desired)

    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    #Test prod_exp()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)

    print('Done!')
