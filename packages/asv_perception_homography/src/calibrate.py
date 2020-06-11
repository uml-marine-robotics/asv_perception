# https://nbviewer.jupyter.org/github/manisoftwartist/perspectiveproj/blob/master/perspective.ipynb
# https://stackoverflow.com/a/20089412/882436

import cv2
import numpy as np
import math
import os
from functools import reduce
from collections import namedtuple
from PyTunerCv import PyTunerCv as Tuner
from PyTunerCv import PyTunerCvControl as TunerCtl

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

# warp a point from the original plane to the warped plane
def warpPt( M, x, y ):
    r = np.matmul( M, np.array([[x],[y], [1]], np.float) )
    r /= r[2,0]
    return r

# inverse warp a point from the warped plane to the original plane
def invWarpPt( M, x, y ):

    r = np.matmul( np.linalg.inv(M), np.array([[x],[y],[1.]], np.float) )
    r /= r[2,0]
    return r

    # more complicated, inefficient alternative:
    # inverse warp matrix
    #  Given 3x3 warp matrix M and coordinates x, y in warped perspective
    #  Convert warped x, y back to original/unwarped coordinates
    # ( | x' | ) =  inverse( |   M11 - xM31  M12 - xM32    | ) * ( | xM33 - M13 | )
    # ( | y' | )           ( |   M32 - yM31  M22 - yM32    | )   ( | yM33 - M23 | )
    #p = np.array([
    #    [M[0,0] - M[2,0]*x, M[0,1] - M[2,1]*x]
    #    , [M[1,0]-M[2,0]*y, M[1,1] - M[2,1]*y]
    #    ], np.float)
    #q = np.array([
    #    [M[2,2]*x - M[0,2]]
    #    , [M[2,2]*y - M[1,2]]
    #    ], np.float)
    #return np.matmul( np.linalg.inv(p), q )

# optimized version: transforms a point like warpPt/invWarpPt; just provide the correct matrix
def transform_pt( mtx, pt ):
    r = np.matmul( mtx, np.array([[pt[0]],[pt[1]],[1.]], np.float) )
    return (r / r[2,0])[:2]
    
def warpMatrix(W, H, theta, phi, gamma, scale, fV, tx, ty, tz ):
    
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
 
    """
    M33 = cv2.getPerspectiveTransform(ptsInPt2f,ptsOutPt2f)
    #aspect ratio
    #print('ar: %f' % float(P[1,1]/P[0,0]) )
    #fovy
    #print('fovy: %f' % float(2.0*math.atan( 1.0/P[1,1] )))

    # find the bounding box at image coordinates
    # right now just return equation of top line
    tr = ptsOutPt2f[2]
    tl = ptsOutPt2f[3]

    # treat v coord as negative y in cartesian plane, origin at image (0,0)
    # return params for line equation (m=slope, b=offset)
    dy = -tr[1] - (-tl[1])
    dx = tr[0]-tl[0]
    m = dy/dx
    b = -tl[1]-tl[0]*m # calculate y intercept at image y=0

    # bottom coordinates are being flipped/rotated for some reason, so we can't figure out where bottom points are mapped for real
    # this likely explains the mirroring above the horizon
    # as long as the rotation isn't inverted or something, returning the top points alone should work for use as filter

    return M33, m, b
    """

"""
Class which performs conversions to/from Image, Radar, and World (2D) coordinates
World is considered to be centered at the center of the radar image, with east=x, north=y
"""
class ImageRadarTransformer(object):
    
    #Initialize with dictionary containing calbration parameters for rgb to radar:  yaw,pich,roll,fovy,tx,ty,tz,radar_img_origin_x,radar_img_origin_y,radar_range
    def __init__(self, d):

        #radar_side = 1024  # radar img dimension for a side
        # Compute warp matrix and set slope/y-intercepts for horizon (topm,topb)
        # for line equation, treat upper-left corner of picture as 0,0 in cartesian coords
        assert int(d['radar_img_origin_x']) == int(d['radar_img_origin_y'])
        radar_side = int(d['radar_img_origin_x']) * 2  # side length of radar image
        self.M, self.topm, self.topb = warpMatrix( radar_side, radar_side, d['yaw'], d['pitch'], d['roll'], 1., d['fovy'], d['tx'], d['ty'], d['tz'] )
        self.Minv = np.linalg.inv(self.M)
        self.radar_img_origin = ( d['radar_img_origin_x'], d['radar_img_origin_y'] )
        self.radar_range = d['radar_range']

        # radar img units to world units = world units per pixel
        self.radar_world_unit_distance = float(self.radar_range) / float(self.radar_img_origin[0])

    # convert image point to radar img point
    #   if image x,y is above the horizon at the specified point, 
    def image_pt_to_radar( self, pt ):
        #radar_pt = invWarpPt( self.M, pt[0], pt[1])
        #return ( radar_pt[0][0], radar_pt[1][0] )
        return transform_pt( self.Minv, pt )

    # convert radar image point to image point
    def radar_pt_to_image( self, pt ):
        #img_pt = warpPt( self.M, pt[0], pt[1] )
        #return ( img_pt[0][0], img_pt[1][0] )
        return transform_pt( self.M, pt )

    # convert radar image point to world point
    def radar_pt_to_world( self, pt ):
        # translate radar img coords to world, then scale by radar-to-world units
        pts = [ 
            pt[0] - self.radar_img_origin[0]
            , -pt[1] + self.radar_img_origin[1]
            ]
        return tuple(v*self.radar_world_unit_distance for v in pts )

    # convert image point to world point
    def image_pt_to_world( self, pt ):
        return self.radar_pt_to_world( self.image_pt_to_radar( pt ))

    # given the image x coordinate, calculate the corresponding image y coordinate which represents the horizon based on the radar transformation
    def image_horizon_pt( self, x ):
        #y=mx+b, but negate it because image y increases as we move more negative in cartesian plane with upper-left as 0,0
        return -( self.topm * x + self.topb )

    # estimates the world height of an object which measures height_px tall, with base at image point pt
    #   base is assumed to be on the ground
    def estimate_world_height_at_image_pt( self, pt, height_px ):

        # idea:  at image point, measure the magnitude of the difference in radar pixels to the left and right (x) by one pixel
        #   ie, what is the magnitude of the difference in radar pixels at one image pixel difference?
        #       then average the two
        #   it's probably better to calculate one image pixel to the left and right while preserving radius from world origin (ie use normal vector), and measure that
        #       just moving along x axis is a hack but probably good enough for an estimate
        center = np.array( [ self.image_pt_to_radar( pt )])
        rt = np.array([ self.image_pt_to_radar( ( pt[0]+1, pt[1] )) ])
        lt = np.array([ self.image_pt_to_radar( ( pt[0]-1, pt[1] )) ])
        rt_mag = np.linalg.norm( center - rt )
        lt_mag = np.linalg.norm( center - lt )
        mag = ( rt_mag + lt_mag ) / 2.0
        return mag * self.radar_world_unit_distance * height_px

# given the the transformation matrix from the radar to the image
#  returns the y value at x=img_x which approximates the horizon
def compute_horizon( mtx, img_x ):

    # transform two arbitrary points, compute slope
    p0 = transform_pt( mtx ,(0,0))
    p1 = transform_pt( mtx, (1,0))
    m = (p1[1]-p0[1])/(p1[0]-p0[0]) # m = dy/dx

    # the radar image point (0,0) is probably projected way off to the left of the start of the image ( x is negative )
    #  we need to calculate the y intercept where the image window starts (x=0)
    #  y=mx+b, solve for b --> b=y-mx
    b = p0[1]- m*p0[0]

    # y = mx + b
    return m*img_x + b

tuner = None


def onMouse( evt, x, y, flags, userData ):
    # handles mouse event on window. x,y are img coords
    
    if evt == cv2.EVENT_MOUSEMOVE:

        # get matrices stored in userData
        M_ri = userData[2][0]       # radar to img
        M_ir = np.linalg.inv(M_ri)  # img to radar
        M_rw = userData[2][1]       # radar to world
        
        radar_xy = transform_pt( M_ir, (x, y) )  # img to radar pts
        world_xy = transform_pt( M_rw, radar_xy ) # radar to world pts
        world_distance = np.linalg.norm( world_xy )  # real-world distance

        cv2.setWindowTitle(
            tuner.imageWindowName
            , '(Press q to quit)  img_x: %d, img_y: %d ==> radar_x'': %d, radar_y:'': %d ==> world_x: %d, world_y: %d, world_distance: %d' 
                % (x,y, radar_xy[0], radar_xy[1], world_xy[0], world_xy[1], world_distance )
        )

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

def get_radar_to_world_matrix( radar_img_side, radar_world_diameter_units ):
    # returns radar to world matrix for provided radar image width and real-world unit measurement
    return np.array([
        [1.,0.,-(radar_img_side/2.)]
        , [0.,1.,-(radar_img_side/2.)]
        , [0.,0.,(radar_img_side/radar_world_diameter_units)]
        ])

# print a 3x3 matrix
def print_3x3_matrix( M ):
    print('%f \t %f \t %f' % ( M[0,0], M[0,1], M[0,2] ))
    print('%f \t %f \t %f' % ( M[1,0], M[1,1], M[1,2] ))
    print('%f \t %f \t %f' % ( M[2,0], M[2,1], M[2,2] ))

# diameter of the radar img in world units
radar_world_diameter = 220.

# accept a dict of prop-->value and ( base img, overlay img, [array] ).  apply to image, return for display
def tuner_transform_img( d, userData ):

    overlay_img_alpha = 0.5

    # todo (?):  use/remove ImageRadarTransformer
    yaw = d['yaw']
    pitch = d['pitch']
    roll = d['roll']
    fovy = d['fovy']
    tx = d['tx']
    ty = d['ty']
    tz = d['tz']

    # Compute warp matrix
    baseImg = userData[0]
    baseH,baseW = baseImg.shape[:2]
    overlayImg = userData[1]
    H,W    = overlayImg.shape[:2]

    # construct radar to rgb matrix
    M = warpMatrix( W,H, yaw, pitch, roll, 1., fovy, tx, ty, tz )

    # construct RADAR to world matrix
    M_rw = get_radar_to_world_matrix( H, radar_world_diameter )

    # using warp matrix:
    # original pt --> warped pt
    #wp = warpPt( M, 512, 512 )
    # warped pt --> original pt
    #iwp = invWarpPt(M, wp[0,0], wp[1,0])
    # assert( wp == iwp )

    #store matrices for mouse event
    if len(userData[2]) < 2:
        userData[2].append(None)
        userData[2].append(None)

    userData[2][0] = M
    userData[2][1] = M_rw

    result = create_unified_image( baseImg, overlayImg, M, overlay_img_alpha)

    # output parameters/warp matrix?
    if 'output' in d and d['output']:
        print('\n*************** Calibration parameters ***************')
        print('RGB image dimensions: w=%d, h=%d' % ( baseW, baseH ) )
        print('RADAR image dimensions: w=%d, h=%d' % ( W, H ) )

        print('\nyaw: %.3f \npitch: %.3f \nroll: %.3f \nfovy: %.3f \ntx: %.3f \nty: %.3f \ntz: %.3f \nradar_img_w: %d' % ( yaw, pitch, roll, fovy, tx, ty, tz, W ) )
        
        print('\n*************** RADAR to rgb image matrix ***************')
        print_3x3_matrix(M)

        print('\n*************** rgb image to RADAR matrix ***************')
        print_3x3_matrix( np.linalg.inv(M) )

        # construct RADAR to world matrix.  M_rw(3,3) numerator assumes square radar img?
        print('\n*************** RADAR to world matrix ***************')
        M_rw = get_radar_to_world_matrix( W, radar_world_diameter )
        #M_rw = np.array([
        #    [1.,0.,-(W/2.)]
        #    , [0.,1.,-(H/2.)]
        #    , [0.,0.,(W/radar_world_diameter)]
        #    ])
        print_3x3_matrix( M_rw )

        print('\n*To transform a point between planes, given transformation matrix M and point [px py]:')
        print(" [x y w]' = M*[px py 1]'")
        print(" [x y w]' = [x y w]' / w")

        print("\n*To calculate the horizon in the image, see compute_horizon")
        print("(horizon pt @ x=0 )= " + str(compute_horizon(M, 0)) )
        print("(horizon pt @ x=w)= "  + str(compute_horizon(M, W)) )
        
    return result

# crops and then resizes to desired shape without distortion
#  based on https://stackoverflow.com/a/4744625/882436
def resize_crop( img, target_w, target_h ):

    target = target_w/float(target_h)
    h,w = img.shape[:2]
    ar = w/float(h)

    if ar  > target: #crop left&right
        new_w = int(target*h)
        offset = ( w - new_w ) / 2
        resize = (offset, 0, w - offset, h)
    else: # crop top&bottom
        new_h = int(w/target)
        offset = (h-new_h) / 2
        resize = (0,offset,w,h-offset)

    result = img[resize[1]:resize[3], resize[0]:resize[2]]  #crop

    return cv2.resize(result, (target_w,target_h)) #resize


# run this file directly to perform manual calibration on calibration images stored in 'calibrate_imgs' folder
if __name__ == '__main__':
    
    imgs_path = os.path.dirname( os.path.abspath(__file__) ) + '/calibrate_imgs/'

    # radar latency
    base_img = cv2.imread(imgs_path + '1572379335.856818.jpg')
    warp_img = cv2.imread(imgs_path + 'radar_1572379335.812.png')    
    
    # set 5
    base_img = cv2.imread(imgs_path + '1572379326.185618.jpg')
    warp_img = cv2.imread(imgs_path + 'radar_1572379326.3049998.png')

    # set 2
    base_img = cv2.imread(imgs_path + 'base-9238.459751.jpg')
    warp_img = cv2.imread(imgs_path + 'overlay-9238.491.png')

    # set 4 (off pitch)
    base_img = cv2.imread(imgs_path + 'base-9386.641404.jpg')
    warp_img = cv2.imread(imgs_path + 'overlay-9386.602.png')

    # set 1  (dock)
    base_img = cv2.imread(imgs_path + 'base-9420.666.jpg')
    warp_img = cv2.imread(imgs_path + 'overlay-9420.636.png')    

    # set 3 (boat - close)
    base_img = cv2.imread(imgs_path + 'base-9275.98.jpg')
    warp_img = cv2.imread(imgs_path + 'radar_1572379275.77.png')  #set 3 radar before refresh
    warp_img = cv2.imread(imgs_path + 'overlay-9276.02.png')


    # resize to multiple of wasr-accepted format (512x384)
    # base_img = resize_crop(base_img, 1024,768)
    # pretty good values for 1024x768:
    #   roll:   -3.0
    #   pitch:  76.02
    #   yaw:    0.0
    #   fovy:   46.3
    #   tx:     2.5
    #   ty:     4.1
    #   tz:     -4.4
    
    h,w   = warp_img.shape[:2]
    # todo:  add transparency to overlay; black=transparent

    # pretty good values for 1280x1024:
    #   roll:   -2.96
    #   pitch:  80.29
    #   yaw:    8.19
    #   fovy:   46.3
    #   tx:     2.5
    #   ty:     6.2
    #   tz:     -4.4

    #using image dimensions for translation (tx, ty, tz) limits but this is not strictly required

    # set world units of radar diameter
    radar_world_diameter = 220.   # meters

    tuner = Tuner( [
        TunerCtl( 'roll', -2.96, 360, True, -2 ) # set1
        , TunerCtl( 'pitch', 80.29, 360, True, -2 ) # set 3
        , TunerCtl( 'yaw', 8.19, 360, True, -2 ) # set1
        , TunerCtl( 'fovy', 46.3, 180, False, -1 ) #set1
        , TunerCtl( 'tx', 2.5, w, True, -1 )  # set1
        , TunerCtl( 'ty', 6.2, h, True, -1 ) # set 3
        , TunerCtl( 'tz', -4.4, h, True, -1 )
        , TunerCtl( 'output', 0, 1, False, 0 ) #special value for parameters&matrix output to stdout
        ]
        , tuner_transform_img # transform_img
        , onMouse
        , ( base_img, warp_img, [] )  #  transformation matrices will be stored in 3rd element array
    )
    
    tuner.show()
    