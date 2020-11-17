"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import math
import numpy as np
from copy import copy
from filterpy.kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment

def bhattacharyya_distance( mu0, sigma0, mu1, sigma1 ):
    """
    Compute Bhattacharyya distance between two multivariate Gaussians, each defined by mu_X (mean) & sigma_X (covariance)
    https://en.wikipedia.org/wiki/Bhattacharyya_distance#Definition
    mu_X should be a 1xN numpy array
    sigma_X should be NxN numpy array
    returns scalar
    """
    mudiff = mu1-mu0    # difference of means
    mv = (sigma0 + sigma1) / 2. # mean of variances
    t1 = 0.125 * np.dot( np.dot( np.transpose(mudiff), np.linalg.inv( mv ) ), mudiff )
    t2 = 0.5 * np.log( np.linalg.det(mv) / np.sqrt( np.linalg.det(sigma0) * np.linalg.det(sigma1) ) )
    return t1+t2

def bhattacharyya_coeff( bd ):
    """
    Compute Bhattacharyya coefficient from Bhattacharyya distance (scalar)
    https://en.wikipedia.org/wiki/Bhattacharyya_distance#Bhattacharyya_coefficient
    """
    return np.exp(-bd)

def hellinger_distance( bc ):
    """
    Compute Hellinger distance from Bhattacharyya coefficient (scalar)
    """
    return np.sqrt(1-bc)

class BhattacharyyaDistance(object):
    """ Class to calculate 2D Bhattacharyya distance """
    def __call__( self, obj1, obj2 ):
        pm = obj1.position[:2]
        pv = obj1.position_covariance[:2,:2]
        qm = obj2.position[:2]
        qv = obj2.position_covariance[:2,:2]
        return bhattacharyya_distance(pm,pv,qm,qv)

class HellingerDistance(object):
    """ Class to calculate 2D Hellinger distance """
    def __call__( self, obj1, obj2 ):

        # bd --> bc --> hellinger
        bd = BhattacharyyaDistance()( obj1, obj2 )
        bc = bhattacharyya_coeff( bd )
        return hellinger_distance( bc )

class EuclideanDistance(object):
    """ Class to calculate Euclidean distance """
    def __call__( self, obj1, obj2 ):
        a = obj1.position
        b = obj2.position
        return math.sqrt( (a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2 )

class PerspectiveDistance(object):
    """
    Perspective weighted proportional difference
        Get vectors relative to origin.  Origin should be updated at each timestep, prior to __call__
        Proportional component = | vecA - vecB | / | vecA + vecB |
        Weighted theta diff = ( 1 + arccos( (vecA dot vecB)/(|vecA||vecB|) ) ) ^ 2
        result = proportional*weighted
        Returns scalar, [0, (1+pi)^2]
    """

    def __init__( self, origin = [0.,0.,0.] ):
        self.origin = np.array(origin,dtype=np.float)

    def __call__( self, obj1, obj2 ):

        p0 = obj1.position
        p1 = obj2.position

        # if we don't have a position (why?), return max_val
        if p0 is None or p1 is None:
            return (1.+math.pi)**2

        a = p0.ravel() - self.origin
        b = p1.ravel() - self.origin
        p = np.linalg.norm( a - b ) / np.linalg.norm( a + b )#proportional component
        if np.isclose( p, 0. ): return 0.
        w = ( 1 + math.acos( np.dot(a,b)/( np.linalg.norm(a) * np.linalg.norm(b) ) ) ) ** 2  # theta diff weight

        return p*w

def cost_matrix( arr0, arr1, cost_fn ):
    """ compute/return 2d numpy cost matrix """
    arr = np.ndarray((len(arr0),len(arr1)))
    for i, ival in enumerate(arr0):
        for j, jval in enumerate(arr1):
            arr[i,j]= cost_fn( ival, jval )
            #print('-----')
            #o0 = ival
            #o1 = jval
            #print('cost eval:  euclidean=%.4f, bhat=%.4f, hellinger=%.4f'  % ( EuclideanDistance()( o0, o1 ), BhattacharyyaDistance()(o0,o1), HellingerDistance()( o0, o1 ) ) )
            #print(o0)
            #print(o1)

    return arr

def matching( obs0, obs1, cost_fn, min_cost, max_cost ):
    """ match objects using cost fn where min_cost <= cost <= max_cost, return list of index tuple matches """

    m = cost_matrix( obs0, obs1, cost_fn )
    row_idx, col_idx = linear_sum_assignment( m )

    result = []
    for i in range( len(col_idx) ):
        cost = m[ row_idx[i], col_idx[i] ]
        
        if cost >= min_cost and cost <= max_cost:
            result.append( ( row_idx[i], col_idx[i] ) )

            # debug
            #print('======')
            #o0 = obs0[row_idx[i]]
            #o1 = obs1[col_idx[i]]
            #print('match:  euclidean=%.4f, bhat=%.4f, hellinger=%.4f'  % ( EuclideanDistance()( o0, o1 ), BhattacharyyaDistance()(o0,o1), HellingerDistance()( o0, o1 ) ) )
            #print(o0)
            #print(o1)
            
    return result

class TrackedObject(object):

    def __init__( self, measurements, data, **kwargs ):
        """
            TrackedObject represents a tracked object, or candidate for matching
            Default filter: 3d position, size, linear velocity
            measurements:  [position_x, position_y, position_z, length, width, height]
            data:  any object to be associated, for external use
            kwargs:
                measurement_uncertainty:    [float, default=None]   Kalman filter measurement uncertainty
                estimate_velocity:          [bool, default=True]    Kalman filter velocity estimation
        """
        self.id = kwargs.get('id',0)
        self.age = 0
        self.hits = 1
        self.data = [ data ]

        kf = KalmanFilter(dim_x=9, dim_z=6)  #state vector sz, measurement vector sz
        kf.P *= 10.  # covariance matrix

        if kwargs.get( 'estimate_velocity', True ):
            # state transition matrix; set off-diagonals for velocity estimation
            kf.F[0,6]= kf.F[1,7]= kf.F[2,8]= 1.
            kf.P[6:, 6:] *= 10. # high covar for initial velocity estimate
            kf.Q[6:, 6:] *= 0.1 # process uncertainty; low process uncertainty for velocity estimate

        if kwargs.get('measurement_uncertainty'):
            kf.R *= float(kwargs['measurement_uncertainty'])  # measurement uncertainty

        kf.H = np.eye(6,9)

        self.filter = kf
        self.filter.x[:self.filter.dim_z] = np.array(measurements,dtype=np.float).reshape((len(measurements),1))

    def predict( self ):
        self.filter.predict()
        self.age += 1

    def update( self, other ):
        self.filter.update( other.filter.x[:self.filter.dim_z] )  # get measurements from other TrackedObject
        self.data.extend(other.data) # add data from other TrackedObject
        self.hits += 1
        self.age = 0

    @property 
    def position( self ):
        """ returns current xyz estimate, 1x3 numpy array """
        return self.filter.x[:3]

    @property
    def position_covariance( self ):
        """ returns current position covariance, 3x3 numpy array """
        return self.filter.P[:3,:3]

    @property
    def dimensions( self ):
        "returns dimensions (L W H), 1x3 numpy array"
        return self.filter.x[3:6]

    # todo:  rest of properties...

    def __str__( self ):

        d = self.filter.x
        p = self.filter.P

        return "id=%d, age=%d, hits=%d, xyz=(%.2f,%.2f,%.2f), xyz_covar=(%.2f,%.2f,%.2f) lwh=(%.2f,%.2f,%.2f), v=(%.2f,%.2f,%.2f), v_covar=(%.2f,%.2f,%.2f)" \
            % (self.id, self.age, self.hits, d[0],d[1],d[2],p[0][0], p[1][1], p[2][2], d[3],d[4],d[5],d[6],d[7],d[8], p[6][6], p[7][7], p[8][8] )


class SensorTracking(object):
    """ A simple multi-sensor multi-object tracking-fusion algorithm """    

    def __init__( self, **kwargs ):
        """ Constructor
        parameters:
        - max_age:  maximum number of frames a trackedobject can persist without being reassociated.  default=3
        - cost_fn:  function or functor to compute cost between two trackedobject.  default=EuclideanDistance()
        - cost_fn_min:  minimum value for the cost function to consider a match.  default=0.
        - cost_fn_max:  maximum value for the cost function to consider a match.  default=10.
        """
        self.trackers = []
        self._id = 1 # next trackedobject id
        self.max_age = kwargs.get('max_age', 3)
        self.cost_fn = kwargs.get( 'cost_fn', EuclideanDistance() )  # default cost function
        self.cost_fn_min = kwargs.get( 'cost_fn_min', 0. )
        self.cost_fn_max = kwargs.get( 'cost_fn_max', 10. )

    def update( self, detections ):
        """ performs update.  detections is a list of TrackedObject, or TrackedObject-derived.  returns list of trackedobject """

        # perform predictions on existing trackers
        for trk in self.trackers:
            trk.predict()

        # attempt to match detections with existing trackers using cost fn
        match_indices = matching( 
            self.trackers,  # existing trackers, predicted state
            detections,     # sensor detections
            self.cost_fn, self.cost_fn_min, self.cost_fn_max
            )

        # attempt updates of tracked obstacles from matches
        for match in match_indices:
            trk = self.trackers[match[0]]
            det = detections[match[1]]
            trk.update( det ) # do update

        # create list of indices of detections that were matched
        matched_detections = [ match[1] for match in match_indices ]  

        # obstacle birth
        for i, det in enumerate(detections):
            if i not in matched_detections:
                det.id = self._id  # assign id
                self._id += 1
                self.trackers.append( det )
                
        # obstacle death
        trackers_to_remove = [ i for i,trk in enumerate(self.trackers) if trk.age > self.max_age ]
        for i in reversed(trackers_to_remove):
            self.trackers.pop(i)

        return self.trackers