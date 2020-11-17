"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

# https://stackoverflow.com/a/20089412
# https://nbviewer.jupyter.org/github/manisoftwartist/perspectiveproj/blob/master/perspective.ipynb

import cv2
import numpy as np
import math
import os
from functools import reduce

def construct_RotationMatrixHomogenous(rotation_angles):
    assert(type(rotation_angles)==list and len(rotation_angles)==3)
    RH = np.eye(4,4)
    cv2.Rodrigues(np.array(rotation_angles), RH[0:3, 0:3])
    return RH

# https://en.wikipedia.org/wiki/Rotation_matrix
def getRotationMatrixManual(rotation_angles):
    
    rotation_angles = map(lambda x : np.deg2rad(x), rotation_angles)
    
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
def getPoints_for_PerspectiveTranformEstimation(ptsIn, ptsOut, W, H, sidelength):
    
    ptsIn2D      =  ptsIn[0,:]
    ptsOut2D     =  ptsOut[0,:]
    ptsOut2Dlist =  []
    ptsIn2Dlist  =  []
    
    for i in xrange(0,4):
        ptsOut2Dlist.append([ptsOut2D[i,0], ptsOut2D[i,1]])
        ptsIn2Dlist.append([ptsIn2D[i,0], ptsIn2D[i,1]])
    
    pin  =  np.array(ptsIn2Dlist)   +  [W/2.,H/2.]
    pout = (np.array(ptsOut2Dlist)  +  [1.,1.]) * (0.5*sidelength)
    pin  = pin.astype(np.float32)
    pout = pout.astype(np.float32)
    
    return pin, pout


def create_warp_matrix(W, H, theta, phi, gamma, scale, fV, tx, ty, tz ):
    
    # M is to be estimated
    M          = np.eye(4, 4)
    
    fVhalf     = np.deg2rad(fV/2.)
    #fHhalf     = np.deg2rad(fH/2.)
    d          = np.sqrt(W*W+H*H)
    sideLengthV = scale*d/np.cos(fVhalf)
    #sideLengthH = scale*d/np.cos(fHhalf)
    h          = d/(2.0*np.sin(fVhalf))

    # n&f are unnecessary
    #n          = h-(d/2.0)  # near plane
    #f          = h+(d/2.0) 

    #constrain img size (to larger of W,H?)
    sideLengthV = max(W,H)

    scale = 1.
    # Translation along Z-axis by -h
    T       = np.eye(4,4)
    #T[2,3]  = -h
    # added translation matrix
    T[0,3] = tx
    T[1,3] = ty
    T[2,3] = tz

    #T[2,3]  = -h
    
    # Rotation matrices around x,y,z
    R = getRotationMatrixManual([phi, gamma, theta])
    
    # Projection Matrix 
    # references:
    #   https://stackoverflow.com/questions/46182845/field-of-view-aspect-ratio-view-matrix-from-projection-matrix-hmd-ost-calib/46195462
    #   https://stackoverflow.com/questions/11277501/how-to-recover-view-space-position-given-view-space-depth-value-and-ndc-xy/46118945#46118945
    P       = np.eye(4,4)
    P[1,1]  = 1.0/np.tan(fVhalf)

    aspect = W / float(H)
    #P[0,0]  = P[1,1]
    P[0,0] = 1.0/ ( np.tan(fVhalf) * aspect )  #need to test w/ non-square img

    #P[2,2]  = -(f+n)/(f-n)
    #P[2,3]  = -(2.0*f*n)/(f-n)
    P[3,2]  = -1.0    
    
    # pythonic matrix multiplication
    F       = reduce(lambda x,y : np.matmul(x,y), [P, T, R]) 
    
    # shape should be 1,4,3 for ptsIn and ptsOut since perspectiveTransform() expects data in this way. 
    # In C++, this can be achieved by Mat ptsIn(1,4,CV_64FC3);
    ptsIn = np.array([[
                 [-W/2., H/2., 0.],[ W/2., H/2., 0.],[ W/2.,-H/2., 0.],[-W/2.,-H/2., 0.]
                 ]])
    ptsOut  = np.array(np.zeros((ptsIn.shape), dtype=ptsIn.dtype))
    ptsOut  = cv2.perspectiveTransform(ptsIn, F)
    
    ptsInPt2f, ptsOutPt2f = getPoints_for_PerspectiveTranformEstimation(ptsIn, ptsOut, W, H, sideLengthV)
    
    # check float32 otherwise OpenCV throws an error
    assert(ptsInPt2f.dtype  == np.float32)
    assert(ptsOutPt2f.dtype == np.float32)
    
    return cv2.getPerspectiveTransform(ptsInPt2f,ptsOutPt2f)

# transforms 2d point using matrix
def transform_pt( mtx, pt ):
    r = np.matmul( mtx, np.array([[pt[0]],[pt[1]],[1.]], np.float) )
    return (r / r[2,0])[:2]

# returns radar to world matrix for provided radar image width and real-world unit measurement
#  converts to x=forward, y=left, z=up IAW rep 103
def get_radar_to_world_matrix( radar_img_side, radar_world_diameter_units ):
    return np.array([
        [0.,-1.,(radar_img_side/2.)]
        , [-1.,0.,(radar_img_side/2.)]
        , [0.,0.,(radar_img_side/radar_world_diameter_units)]
        ])

# given the the transformation matrix from the radar to the image
#  returns the y value at x=img_x which approximates the horizon
def compute_horizon( mtx, img_x ):

    # transform two arbitrary points, compute slope
    p0 = transform_pt( mtx ,(0,0))
    p1 = transform_pt( mtx, (1,0))
    m = (p1[1]-p0[1])/(p1[0]-p0[0]) # m = dy/dx

    # the radar image point (0,0) is probably projected way off to the left of the the image ( ie, x is negative )
    #  we need to calculate the y intercept where the image window starts (x=0)
    #  y=mx+b, solve for b --> b=y-mx
    b = p0[1]- m*p0[0]

    # y = mx + b
    return m*img_x + b

# constrain an image to bounding box defined by 4 points
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

def create_unified_image( base_img, overlay_img, M, overlay_alpha = 0.5 ):
    """
    Create unified image between base and overlay
    Overlay image is warped with the homography matrix M, and everything above the computed horizon is then truncated
    Overlay is applied with the specified alpha
    """
    
    baseH,baseW = base_img.shape[:2]
    H,W    = overlay_img.shape[:2]

    # now actually warp the image, match viewport dims with base image
    overlay = cv2.warpPerspective( overlay_img, M, (baseW,baseH) )

    # constrain overlay to valid region
    #  construct bounding box, clockwise points
    bb = [ 
        ( 0, compute_horizon( M, 0 ) )  # top left
        , ( baseW, compute_horizon( M, baseW ) )  # top right
        , (baseW,baseH) # bottom right
        ,(0,baseH)  # bottom left
        ]
    overlay = constrain( overlay, bb )
    
    return cv2.addWeighted( overlay, overlay_alpha, base_img, 1 - overlay_alpha, 0 )