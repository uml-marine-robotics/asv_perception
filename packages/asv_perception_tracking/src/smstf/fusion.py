"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import copy
import numpy as np
from threading import Lock
from tracking import TrackedObject, EuclideanDistance, matching

class TrackedObjectGroup( object ):
    """ 
    Group of TrackedObject
    """

    def __init__( self, id, trk ):
        """
        Constructor
        id:  unique id
        trk:  (sensor_id, TrackedObject)
        """

        # list of (sensor_id, TrackedObject), represents the most recent reading from each sensor
        self.trackers = [trk]
        self.id = id
        self.data = None
        self.lock = Lock()

        # flag if fusion vars need recalculating due to change in source trackers
        self.dirty = False
        
        # fused position and position covariance estimate
        self._position = trk[1].position
        self._position_covariance = trk[1].position_covariance

    def update_from_sensor_trackers( self, sensor, sensor_trackers ):
        """ 
            If this group contains a matching (sensor_id, tracker.id) combination from the input, 
            the tracker data is updated and returns index of sensor_trackers that was used for the update
            Otherwise it removes all sensor-specific trackers from this group and returns None
            TODO:  handle multiple detections from the same sensor
        """
        sensor_id = sensor['id']

        for i,sensor_trk in enumerate(sensor_trackers):
            
            if sensor_trk is None:  # may have previously been utilized/erased
                continue

            for j,trk in enumerate(self.trackers):
                if trk[0] == sensor_id and trk[1].id==sensor_trk.id:

                    # reevaluate sensor tracker using cost metric if have multiple trackers
                    #   if no longer should be associated, pop from trackers, set dirty, and return none
                    if len(self.trackers) > 1 and not self.dirty:
                        cost = sensor['cost_fn']( self, sensor_trk ) * sensor.get('cost_disassociation_factor', 2. )
                        if cost < sensor['cost_fn_min'] or cost > sensor['cost_fn_max']:
                            #print("Disassociating {},{} from {}, cost={}".format( sensor['id'], trk[1].id, self.id, cost ) )
                            self.trackers.pop(j)
                            self.dirty = True
                            return None

                    self.trackers[j]=(sensor_id,sensor_trk)
                    self.dirty = True
                    return i

        # if not found, remove old tracker for this sensor_id
        for i,trk in enumerate(self.trackers):
            if trk[0]==sensor_id:
                self.trackers.pop(i)
                self.dirty = True
                break

        return None

    def _fuse_trackers( self ):
        "performs fusion of tracker data to fusion variables, resets dirty flag"
        
        if not self.dirty: return
        if len( self.trackers ) == 0: 
            self._position=None
            self._position_covariance=None
            return

        if len( self.trackers ) == 1:
            self._position=self.trackers[0][1].position
            self._position_covariance=self.trackers[0][1].position_covariance
        else:   # >1 source, perform fusion

            # https://en.wikipedia.org/wiki/Sensor_fusion#Example_calculations
            # position looks good, but combined variance is halved using this formula            
            positions = [ x[1].position for x in self.trackers ]
            covars = [ x[1].position_covariance for x in self.trackers ]

            # compute element-wise reciprocals of a matrix, where value is not zero
            reciprocals = lambda x : np.where( x != 0.0, x**-1.0, x )

            # create list of covariances
            covars_inv = [ reciprocals( x ) for x in covars ]

            # sum them, get reciprocal of sum
            sums = np.zeros_like(covars_inv[0])
            for c in covars_inv:
                sums = np.add(sums,c)
            covar_coeff = reciprocals( sums )

            # weight positions by reciprocals of their covariances
            weighted_pos = np.zeros_like( positions[0] )
            for z in zip( covars_inv, positions ):
                weighted_pos = np.add( weighted_pos, np.matmul( z[0], z[1] ) )

            # compute position
            self._position = np.matmul( covar_coeff, weighted_pos )

            # combined covariance: https://stats.stackexchange.com/a/56000
            #   assume equal count of observations
            covar_sums = np.zeros_like( covars[0] )
            for c in covars:
                covar_sums = np.add(covar_sums,c)

            # convert positions to diagonal 3x3 so they'll operate nicely with the covars
            positions_diag = [ np.diag( np.reshape(x,-1) ) for x in positions ]
            position_diag = np.diag( np.reshape( self._position,-1) )

            diffs_sq = np.zeros_like( position_diag )    # sum of mean diffs from mean position (computed above), squared
            for pd in positions_diag:
                diffs_sq = np.add( diffs_sq, np.power( position_diag - pd, 2. ) )

            self._position_covariance = np.divide( np.add( covar_sums, diffs_sq ), float(len(covars)) )

        self.dirty=False
    
    # properties for use with distance algorithms

    @property 
    def position( self ):
        "returns current xyz estimate, 1x3 numpy array (or None)"
        self._fuse_trackers()
        return self._position

    @property
    def position_covariance(self):
        "returns current position covariance, 3x3 numpy array (or None)"
        self._fuse_trackers()
        return self._position_covariance

    def __str__( self ):

        p = self._position
        pc = self._position_covariance
        s = "id=%d, xyz=(%.2f,%.2f,%.2f), xyz_covar=(%.2f,%.2f,%.2f)\n\tTrackers:" % ( self.id, p[0], p[1], p[2], pc[0][0], pc[1][1], pc[2][2]  )
        for trk in self.trackers:
            s += "\n\t sensor={}, ".format( trk[0] )
            s += str(trk[1])
        return s


class SensorFusion(object):
    """ Performs fusion of multiple sensors, each which emits a list of TrackedObject """

    def __init__( self, **kwargs ):
        """ Constructor"""

        self.tracked_groups = []  # list of TrackedObjectGroup
        self._id = 1 # next group id
        self.lock = Lock()

    def update( self, sensor, sensor_trackers ):
        """ 
        Performs fusion of [TrackedObject] for {sensor}
        parameters:
            - sensor:  dict containing info about the sensor. 
                keys: id, cost_fn, cost_fn_min, cost_fn_max
                    cost_fn is function or functor to compute cost between two trackedobject, with min/max limits
            - sensor_trackers:  list of [TrackedObject]
        """

        unmatched_groups = copy.copy( self.tracked_groups )  # shallow copy; list of unmatched groups
        unmatched_trackers = copy.copy( sensor_trackers )  # shallow copy; list of unmatched sensor_trackers

        # for each currently tracked group, if that group has a tracker for the current sensor_id, update it
        for i, group in enumerate( unmatched_groups ):
            with group.lock:
                trk_idx = group.update_from_sensor_trackers( sensor, unmatched_trackers )

            if not trk_idx is None:
                unmatched_trackers[trk_idx]=None
                unmatched_groups[i]=None

        # remove None
        unmatched_trackers = filter( None, unmatched_trackers ) 

        # unmatched sensor detections:  attempt to match with unmatched groups, otherwise create new group
        if len(unmatched_trackers) > 0:

            with self.lock:

                unmatched_groups = filter( None, filter( lambda x : x if x and x.trackers else None, unmatched_groups ) )
                
                # match sensor trackers with unmatched groups using sensor-specific cost fn
                match_indices = matching( 
                    unmatched_groups,
                    unmatched_trackers,
                    sensor['cost_fn'], sensor['cost_fn_min'], sensor['cost_fn_max']
                    )
            
                # update groups with sensor trackers
                for match in match_indices:
                    group = unmatched_groups[match[0]]  # get group
                    sensor_tracker = unmatched_trackers[match[1]]  # get sensor tracker object
                    group.trackers.append( (sensor['id'], sensor_tracker) )  # append to trackers
                    group.dirty = True

                    # debug
                    #print("matching:  group id={}, sensor={}, sensor trk id={}, cost={}\n{}\n".format( group.id, sensor['id'], str(sensor_tracker.id), sensor['cost_fn']( group, sensor_tracker ), str(group) ) )

                    unmatched_trackers[match[1]]=None  # remove tracker from unmatched_trackers

            # remaining unmatched trackers:  group birth
            for trk in filter( None, unmatched_trackers ):
                self.tracked_groups.append( TrackedObjectGroup( self._id, ( sensor['id'], trk ) ) )
                self._id += 1

    def cleanup( self ):
        "performs cleanup"
        
        # TODO:  group merge evaluation
        #   update() needs to properly handle one group having multiple detections from the same sensor

        # prune dead tracker groups
        with self.lock:
            dead_groups = [ i for i,group in enumerate(self.tracked_groups) if len(group.trackers) == 0 ]
            for i in reversed(dead_groups):
                self.tracked_groups.pop(i)
