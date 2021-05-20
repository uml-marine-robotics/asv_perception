#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import math
import cv2
import numpy as np
import cupy as cp
from pyswarm import pso
from functools import reduce
#from tf.transformations import euler_from_quaternion

# https://computergraphics.stackexchange.com/a/8229
def quaternion_to_rpy(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll,pitch,yaw]

def estimate_pixel_to_metric_size( u, v, M ):
    "Given an image pixel (u,v), estimate the metric size representation of a pixel at that ground point.  M: image to metric matrix"

    # idea:  at image point, measure the magnitude of the difference in radar pixels to the left and right (x) by one pixel
    #   ie, what is the magnitude of the difference in radar pixels at one image pixel difference?
    #       then average the two
    #   it's probably better to calculate one image pixel to the left and right while preserving radius from world origin (ie use normal vector), and measure that
    #       just moving along x axis is a hack but probably good enough for an estimate
    
    pts = np.array([[u-1,v,1],[u,v,1],[u+1,v,1]],dtype=np.float32)  # x, y, 1; left, center, right

    pts_metric = transform_points( pts, M )

    lt_mag = np.linalg.norm( pts_metric[1] - pts_metric[0] )
    rt_mag = np.linalg.norm( pts_metric[1] - pts_metric[2] )
    mag = ( rt_mag + lt_mag ) / 2.0 # compute mean
    return mag


# https://en.wikipedia.org/wiki/Rotation_matrix
def getRotationMatrixManual(rotation_angles):
    
    rotation_angles = list(map(lambda x : np.deg2rad(x), rotation_angles))
    
    
    phi         = rotation_angles[0] # around x
    gamma       = rotation_angles[1] # around y
    theta       = rotation_angles[2] # around z
    
    # X rotation
    Rphi        = np.eye(4,4)
    sp          = np.sin(phi)
    cp          = np.cos(phi)
    Rphi[1,1]   = cp
    Rphi[2,2]   = Rphi[1,1]
    Rphi[1,2]   = -sp
    Rphi[2,1]   = sp
    
    # Y rotation
    Rgamma        = np.eye(4,4)
    sg            = np.sin(gamma)
    cg            = np.cos(gamma)
    Rgamma[0,0]   = cg
    Rgamma[2,2]   = Rgamma[0,0]
    Rgamma[0,2]   = sg
    Rgamma[2,0]   = -sg
    
    # Z rotation (in-image-plane)
    Rtheta      = np.eye(4,4)
    st          = np.sin(theta)
    ct          = np.cos(theta)
    Rtheta[0,0] = ct
    Rtheta[1,1] = Rtheta[0,0]
    Rtheta[0,1] = -st
    Rtheta[1,0] = st
    
    R           = reduce(lambda x,y : np.matmul(x,y), [Rphi, Rgamma, Rtheta]) 
    
    return R

def getPoints_for_PerspectiveTranformEstimation(ptsIn, ptsOut, L ):
    "prepares points for getPerspectiveTransform"

    ptsIn2D      =  ptsIn[0,:]
    ptsOut2D     =  ptsOut[0,:]
    ptsOut2Dlist =  []
    ptsIn2Dlist  =  []
    
    for i in range(0,4):
        ptsOut2Dlist.append([ptsOut2D[i,0], ptsOut2D[i,1]])
        ptsIn2Dlist.append([ptsIn2D[i,0], ptsIn2D[i,1]])
    
    pin  =  np.array(ptsIn2Dlist)   +  [L/2.,L/2.]
    pout = (np.array(ptsOut2Dlist)  +  [1.,1.]) * (0.5*L)
    pin  = pin.astype(np.float32)
    pout = pout.astype(np.float32)
    
    return pin, pout

def get_perspective_transform_points( W, H, yaw, pitch, roll, fV, tx, ty, tz ):
    "Get perspective transform points for camera with image dimensions W & H (px), fov (degrees), extrinsics ypr (degrees), xyz (px)"

    T       = np.eye(4,4)
    T[0,3] = tx
    T[1,3] = ty
    T[2,3] = tz
    
    # Rotation matrices around x,y,z
    R = getRotationMatrixManual([pitch, roll, yaw])
    
    # Perspective Projection Matrix P
    # references:
    #   https://stackoverflow.com/questions/46182845/field-of-view-aspect-ratio-view-matrix-from-projection-matrix-hmd-ost-calib/46195462
    #   https://stackoverflow.com/questions/11277501/how-to-recover-view-space-position-given-view-space-depth-value-and-ndc-xy/46118945#46118945
    P = np.eye(4,4)
    
    #P[0,0] = P[1,1] = 1.0/np.tan(fVhalf)
    # use AR for non-square input img
    aspect = W / float(H)
    fVhalf = np.deg2rad(fV/2.)
    P[0,0] = 1.0/ ( np.tan(fVhalf) * aspect ) 
    P[1,1] = 1.0/ ( np.tan(fVhalf) ) 

    P[3,2] = -1.0

    # pythonic matrix multiplication
    F       = reduce(lambda x,y : np.matmul(x,y), [P, T, R]) 
    
    L = max(W,H)
    Lhalf = L/2.
    # points in square image
    ptsIn = np.array([[
                 [-Lhalf, Lhalf, 0.],[ Lhalf, Lhalf, 0.],[ Lhalf,-Lhalf, 0.],[-Lhalf,-Lhalf, 0.]
                 ]])
    ptsOut  = np.array(np.zeros((ptsIn.shape), dtype=ptsIn.dtype))

    # apply perspective transform to square img points
    ptsOut  = cv2.perspectiveTransform(ptsIn, F)
    
    return getPoints_for_PerspectiveTranformEstimation(ptsIn, ptsOut, L )

def get_perspective_transform_points_( W, H, yaw, pitch, roll, fV, tx, ty, tz ):
    "Get perspective transform points for camera with image dimensions W & H (px), fov (degrees), extrinsics ypr (degrees), xyz (px)"

    T       = np.eye(4,4)
    T[0,3] = tx
    T[1,3] = ty
    T[2,3] = tz
    
    # Rotation matrices around x,y,z
    R = getRotationMatrixManual([pitch, roll, yaw])
    
    # Perspective Projection Matrix P
    # references:
    #   https://stackoverflow.com/questions/46182845/field-of-view-aspect-ratio-view-matrix-from-projection-matrix-hmd-ost-calib/46195462
    #   https://stackoverflow.com/questions/11277501/how-to-recover-view-space-position-given-view-space-depth-value-and-ndc-xy/46118945#46118945
    fVhalf = np.deg2rad(fV/2.)
    P = np.eye(4,4)
    P[0,0] = P[1,1] = 1.0/np.tan(fVhalf)
    P[3,2] = -1.0    

    #need to test w/ non-square img
    #aspect = W / float(H)
    #P[0,0] = P[1,1] = 1.0/ ( np.tan(fVhalf) * aspect ) 

    # pythonic matrix multiplication
    F       = reduce(lambda x,y : np.matmul(x,y), [P, T, R]) 
    
    L = max(W,H)
    Lhalf = L/2.
    # points in square image
    ptsIn = np.array([[
                 [-Lhalf, Lhalf, 0.],[ Lhalf, Lhalf, 0.],[ Lhalf,-Lhalf, 0.],[-Lhalf,-Lhalf, 0.]
                 ]])
    ptsOut  = np.array(np.zeros((ptsIn.shape), dtype=ptsIn.dtype))

    # apply perspective transform to square img points
    ptsOut  = cv2.perspectiveTransform(ptsIn, F)
    
    return getPoints_for_PerspectiveTranformEstimation(ptsIn, ptsOut, L )

def create_camera_to_metric_matrix( W, H, yaw, pitch, roll, fV, tx, ty, tz, depth ):

    dst, src = get_perspective_transform_points( W, H, yaw, pitch, roll, fV, tx, ty, tz )

    # camera to transformed perspective
    M = cv2.getPerspectiveTransform( src, dst )

    # camera -> transformed -> metric
    L = max(W,H)
    to_metric = np.array([
        [0.,-1.,(L/2.)]
        , [-1.,0.,(L/2.)]
        , [0.,0.,(L/(depth*2.))]
        ])

    return np.matmul( to_metric, M )

def create_metric_to_camera_matrix( W, H, yaw, pitch, roll, fV, tx, ty, tz, depth ):

    src, dst = get_perspective_transform_points( W, H, yaw, pitch, roll, fV, tx, ty, tz )

    # transformed perspective to camera
    M = cv2.getPerspectiveTransform( src, dst )

    # metric -> transformed -> camera
    L = max(W,H)
    from_metric = np.linalg.inv( np.array([
        [0.,-1.,(L/2.)]
        , [-1.,0.,(L/2.)]
        , [0.,0.,(L/(depth*2.))]
        ]) )

    return np.matmul( M, from_metric )
    

# given the the transformation matrix from the radar to the image
#  returns the y value at x=img_x which approximates the horizon
#  matrix should be the image-to-metric matrix
def compute_horizon( M, u ):
    
    # vectorized version
    #a = np.arange( w,dtype=np.float32 )
    #a = ( -(a * M[2,0] + M[2,2]) / M[2,1] ).astype(np.int32)

    return -( u * M[2,0] + M[2,2] )/M[2,1]

# constrain an image to bounding box defined by 4 points
# all pixels outside bounding box are set to val
def constrain( img, bb ):

    mask = np.zeros(img.shape, dtype=np.uint8)
    roi_corners = np.array([bb], dtype=np.int32)
    # fill the ROI so it doesn't get wiped out when the mask is applied
    channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
    ignore_mask_color = (255,)*channel_count
    cv2.fillConvexPoly(mask, roi_corners, ignore_mask_color)

    # apply the mask
    masked_image = cv2.bitwise_and(img, mask)

    return masked_image

def extract_max_line_from_img( img ):
    " Returns 1d array of Y indices which has the max value in each X column, or 0 if not found "
    # based on https://stackoverflow.com/a/8768734
    #  this argmax fn doesn't work in cupy?
    h = img.shape[0]

    xp = cp.get_array_module( img )
    linemax =  h - xp.argmax( img[::-1], axis=0 )

    # if no max found in column, replace with zero; note that this assumes the max isn't actually at h
    #linemax[linemax==h]=0

    # if linemax >= some percentage of h, then set to 0
    #   TODO:  parameterize this
    linemax[linemax >= int(h*0.9)]=0

    return linemax

def get_scaling_errors( img ):
    "Assign error weights per pixel"

    xp = cp.get_array_module( img )
    h = img.shape[0]

    gt = extract_max_line_from_img( img )
    gt[gt==0]=h  # map zeros to h

    h_vec = xp.ones_like( gt ) * h  # 1d vector of h
    
    result = h_vec - gt  # subtract height from gt
    result = xp.sqrt( result )  # use sqrt of diff
    return result


def transform_points( pts, M ):
    """ transforms/scales pts (x,y,1) using M; returns transformed/normalized Nx3 matrix with resulting x,y,1 """
    xp = cp.get_array_module(pts)
    ptsi = xp.matmul(M,pts.T).T
    return (ptsi / ptsi[:,2][:,None]) # normalize

def transform_points_to_viewport( pts, M, w, h, truncate_above_horizon = False, dilate = 0 ): # -> np.ndarray:
    """
    Transform points using projection matrix to image indices within a viewport defined by w,h
    Dilate:  number of pixels to dilate each point horizontally
    Returns boolean image
    pts:  (x,y,1) in ENU
    """
    ptsi = transform_points(pts, M)[:,:2] # transform points using M, use first two columns x,y

    # extract points within viewport
    ptsi = ptsi[
        ( ptsi[:,0] < w ) 
        & ( ptsi[:,0] >= 0 ) 
        & ( ptsi[:,1] < h ) 
        & ( ptsi[:,1] >= 0 )
    ]

    # create img with transformed points
    xp = cp.get_array_module(ptsi)
    result = xp.zeros( (h,w), dtype=bool)

    # have points within viewport?
    if ptsi.shape[0] > 0:
        
        # set viewport points from ptsi
        result[ ptsi[:,[1]].astype(xp.int32), ptsi[:,[0]].astype(xp.int32) ] = True

        if truncate_above_horizon:

            Minv = xp.linalg.inv( M )  # need inverse matrix for horizon computation

            # get the line at infinity, remove everything above it
            #   even if we pre-filter the points to prevent above-the-horizon transformations,
            #   this code adds negligible timing overhead, so just leave it
            bb = np.array([ 
                (0,0), (w,0)  # tl, tr
                , ( w, compute_horizon( Minv, w ) )  # br
                , ( 0, compute_horizon( Minv, 0 ) )  # bl
            ], dtype=np.int32 )

            result = cp.asnumpy(result).astype(np.uint8) # cupy bool --> numpy uint8 for cv2 fn
            cv2.fillConvexPoly( result, bb, 0 )

        if dilate > 0:
            result = cp.asnumpy( result ).astype(np.uint8)  # cupy --> numpy uint8 for cv2 fn
            result = cv2.dilate( result, np.ones((1,dilate)))
            result = xp.asarray( result )

    # return uint8 array
    return result.astype(np.uint8)


def extract_max_line_from_pts( pts, M, w, h, dilate=0 ):
    """ transform points using M, constrain to w, h; extract max line """

    # this is similar to overlay creation code; 
    #   create 2d array, transform/filter points, then select the max per image column
    #   dilate=5 worked well for kitti due to pc sparsity
    img = transform_points_to_viewport( pts, M, w, h, dilate )
    
    return extract_max_line_from_img( img )

def create_unified_image( base_img, pts, M, overlay_alpha = 0.5 ):
    """
    Create unified image between base image and points using metric_to_image matrix M
    Overlay is applied with the specified alpha
    """

    # if the base image isn't 3 channel, make it so
    base = base_img
    if ( len(base.shape) < 3 or base.shape[2] < 3 ):
        base = cv2.cvtColor( base,cv2.COLOR_GRAY2RGB )

    h,w = base.shape[:2]

    # transform points, create the overlay
    mask = cp.asnumpy( transform_points_to_viewport( pts, M, w, h, dilate=5 ) )
    overlay = np.zeros_like( base )
    overlay[mask>0]=[0,255,0]

    return cv2.addWeighted( overlay, overlay_alpha, base, 1 - overlay_alpha, 0 )

# lb/ub array index values
ROLL = 0
PITCH = 1
YAW = 2
FOV = 3
TX = 4
TY = 5
TZ = 6
IMU_ROLL = 7
IMU_ROLL_VELOCITY = 8
IMU_PITCH = 9
IMU_PITCH_CUBED = 10
IMU_PITCH_VELOCITY = 11

def adjust_roll_pitch( imu, params ):
    "returns adjusted pitch and roll values from the set of parameters and ROS Imu message"
    
    # in case we're not using imu
    if imu is None:
        return params[ROLL], params[PITCH]

    # imu roll, pitch, yaw; rads
    #imu_rpy = np.degrees( euler_from_quaternion( [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w] ) )
    #imu_rpy = euler_from_quaternion( [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w] )
    imu_rpy = quaternion_to_rpy( imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w )
    
    # imu angular velocity; rads/s
    imu_roll_velocity = imu.angular_velocity.x
    imu_pitch_velocity = imu.angular_velocity.y

    adjusted_roll = params[ROLL] + params[IMU_ROLL]*imu_rpy[0] + params[IMU_ROLL_VELOCITY]*imu_roll_velocity
    adjusted_pitch = params[PITCH] + params[IMU_PITCH]*imu_rpy[1] + params[IMU_PITCH_CUBED]*(imu_rpy[1]**3) + params[IMU_PITCH_VELOCITY]*imu_pitch_velocity
    
    return adjusted_roll, adjusted_pitch

def get_adjusted_params( imu, params ):
    "Returns adjusted params which can be passed directly to matrix creation methods. {y p r fov tx ty tz}"
    r, p = adjust_roll_pitch( imu, params )
    return params[YAW], p, r, params[FOV], params[TX], params[TY], params[TZ]

def pointcloud_reduction( pts, lb, ub, w, h, depth ):
    "Perform pointcloud reduction on the provided points"

    #   the points which create a line will not vary that much as the parameters are adjusted
    #   now that we have constraints, we can project the points which could possibly fit within the window
    #   eg maximum fov, minimum z, min/max yaw etc.  Will need a list of poses from which to extract.
    #       max fov+max yaw+min z + min roll; max fov + min yaw + min z + max roll
    #       how to ensure all points are contained across fovs/yaw?
    #   Then extract the points (just in the bottom line?) after that transformation
    #   Reproject them back to the original frame, should only yield 2K points or so

    # create list of params which will contain all possible point projections
    #   TODO:  need some way to check that all points are contained within cumulative output

    # compute some means
    p = (lb[PITCH]+ub[PITCH])/2.
    tx = (lb[TX]+ub[TX])/2.
    ty = (lb[TY]+ub[TY])/2.

    mtxs = [ 
        # max fov, max yaw, min z, min roll
        create_camera_to_metric_matrix( w, h, 
            ub[YAW], p, lb[ROLL], ub[FOV], 
            tx, ty, lb[TZ], depth 
            ), 

        # max fov, min yaw, min z, max roll
        create_camera_to_metric_matrix( w, h, 
            lb[YAW], p, ub[ROLL], ub[FOV], 
            tx, ty, lb[TZ], depth 
            ), 
    ]

    # convert matrices to cupy as needed
    xp = cp.get_array_module( pts )
    mtxs = [ xp.asarray(m) for m in mtxs ]

    result = None

    for m in mtxs:

        m_inv = xp.asarray( xp.linalg.inv( m ) )

        img = transform_points_to_viewport( pts, m_inv, w, h, True )

        line = extract_max_line_from_img( img )  # y values

        # convert line to columnar x,y,1 matrix
        pts_mtx = xp.hstack( ( xp.arange( w ).reshape(-1,1), line.reshape(-1,1), xp.ones( w ).reshape(-1,1) ) ).astype(xp.float32)

        # remove points at y==0
        pts_mtx = pts_mtx[pts_mtx[:,1] > 0]

        # project back to 3d
        pts_mtx = transform_points( pts_mtx, m )

        # concat with result
        if result is None:
            result = pts_mtx
        else:
            result = xp.vstack( (pts_mtx,result) )

    return result

def to_gpu( a ): # -> cp.ndarray
    "Move array-like/numpy array to gpu"
    return cp.asarray( a )

def to_numpy( a ): # -> np.ndarray
    "Move array-like/cupy array to numpy"
    return cp.asnumpy( a )

def standardize_points( pts_in, min_x = 6 ):  # -> pts
    "Standardize input points for processing"
    pts = pts_in[:,:2] # drop columns beyond x,y

    # drop points below min_x
    pts = pts[pts[:,0] >= min_x]

    return np.hstack( ( pts, np.ones( (pts.shape[0],1) ) ) ).astype(np.float32)  # add column of 1s, convert to float32

class Exemplar(object):
    "Data point; an example of the problem"
    points = None #: Points
    target=None#: 1-D np.ndarray of seg img width.  target line from the segmentation image
    imu=None#: Imu
    error_scaling = None  # 1-D np.ndarray of error scaling coefficients per x pixel
    depth = 0.
    img_w = 0
    img_h = 0

    def __init__( self, pts_in, seg_img, imu, depth, lb, ub, pts_min_x = 1., use_gpu = True ):
        
        #self.old_imu = old_imu_to_params(imu)
        self.imu = imu
        self.depth = depth

        # extract target line from segmentation file
        self.target = extract_max_line_from_img( seg_img )

        # calculate error scaling coefficients for each x
        self.error_scaling = get_scaling_errors( seg_img )

        # get seg img dimensions
        self.img_h, self.img_w = seg_img.shape[0], seg_img.shape[1]

        self.points = standardize_points( pts_in, pts_min_x )

        # cupy conversion if using gpu
        if use_gpu:
            self.points = to_gpu( self.points )
            self.target = to_gpu( self.target )
            self.error_scaling = to_gpu( self.error_scaling )
            
        # further pc reduction (optional)  (down to ~2k pts)
        self.points = pointcloud_reduction( self.points, lb, ub, self.img_w, self.img_h, self.depth )

    def target_np( self ):
        "Return 1d target line as numpy array"
        return cp.asnumpy(self.target)

    #def evaluate( self, r, p, y, fov, tx, ty, tz, r_imu, r_imu_v, p_imu, p_imu_poly, p_imu_v ):
    def evaluate( self, params ):
        "Eval params against exemplar, returns ( error or NaN, metric_to_camera, pts_line )"

        # points to evaluate against
        eval_points = self.points

        # create camera to metric and inverse matrices using parameters
        xp = cp.get_array_module( eval_points )
        
        M_metric_to_camera = xp.asarray( create_metric_to_camera_matrix( self.img_w, self.img_h, *get_adjusted_params( self.imu, params ), depth=self.depth ) )

        # get the line from the projected points
        pts_line = extract_max_line_from_pts( eval_points, M_metric_to_camera, self.img_w, self.img_h )
        
        err = xp.nan

        if xp.any(pts_line):

            # create boolean mask of valid column values as extracted from the points
            valid_indices = ( pts_line != 0 ) & ( self.target != self.img_h ) & ( self.target > 0 )

            # compute error between segmented image line and projected points line

            # get error between prediction and gt
            diff_err = xp.subtract( self.target[valid_indices], pts_line[valid_indices] )

            # square the errors
            diff_err_squared = xp.square( diff_err )

            # scale the squared errors
            diff_err_scaled = xp.multiply( diff_err_squared, self.error_scaling[valid_indices] )

            # median pixel error of scaled errors
            #  cupy doesn't have `median` until v8, so won't work in Py2
            #  err = xp.median( diff_err_scaled )
            err = np.median( to_numpy( diff_err_scaled ) )

        return float(err), M_metric_to_camera, pts_line

def obj( x, *args, **kwargs):
    """
    The objective function to minimize
    Parameters:
        - x:  numpy array of current parameters to evaluate
        - data: list of exemplars
        - params:  Params config
    """
    
    data = kwargs["data"]  # list of Exemplar

    # extract params from x, order must match from lb/ub below
    #r, p, y, fov, tx, ty, tz, r_imu, r_imu_v, p_imu, p_imu_sq, p_imu_v = x

    errors = []

    # evaluate all exemplars; return sum of MSEs
    for ex in data:

        #eval_results = ex.evaluate(*x)
        eval_results = ex.evaluate(x)
        err = eval_results[0]

        if np.isnan( err ):
            return np.inf
        errors.append( err )

    if len(errors)==0:
        return np.inf
    return np.median( errors )

def optimize( lb, ub, data, minstep=1e-1, swarmsize=50 ):# -> Any:
    """
    Optimize params using data, return optimal parameters
    """

    # TODO:  find optimal hyperparams

    # fast
    # opt = pso( obj, lb, ub, minstep=1., swarmsize=10, kwargs={"data":data } )

    # pretty good
    # opt = pso( obj, lb, ub, minstep=1e-3, swarmsize=100, kwargs={"data":data } )

    # mix
    opt = pso( obj, lb, ub, minstep=minstep, swarmsize=swarmsize, kwargs={"data":data } )

    return opt
