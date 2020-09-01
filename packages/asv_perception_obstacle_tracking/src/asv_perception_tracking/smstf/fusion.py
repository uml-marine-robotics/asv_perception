import copy
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

    def update_from_sensor_trackers( self, sensor_id, sensor_trackers ):
        """ if this group contains a matching (sensor_id, tracker.id) combination from the input, 
            the tracker data is updated and returns index of sensor_trackers that was used for the update
            Otherwise it removes the sensor-specific trackers from this group and returns None
            """
        for i,sensor_trk in enumerate(sensor_trackers):
            
            if sensor_trk is None:  # may have previously been utlized/erased
                continue

            for j,trk in enumerate(self.trackers):
                if trk[0] == sensor_id and trk[1].id==sensor_trk.id:
                    self.trackers[j]=(sensor_id,sensor_trk)
                    return i

        # else, not found.  remove old tracker for this sensor_id
        for i,trk in enumerate(self.trackers):
            if trk[0]==sensor_id:
                self.trackers.pop(i)
                break

        return None

class SensorFusion(object):
    """ Performs fusion of multiple sensors, each which emits a list of TrackedObject """

    def __init__( self, **kwargs ):
        """ Constructor
        parameters:
        - cost_fn:  function or functor to compute cost between two trackedobject.  default=EuclideanDistance()
        - cost_fn_min:  minimum value for the cost function to consider a match.  default=0.
        - cost_fn_max:  maximum value for the cost function to consider a match.  default=10.
        """

        self.tracked_groups = []  # list of TrackedObjectGroup
        self._id = 1 # next group id
        self.cost_fn = kwargs.get( 'cost_fn', EuclideanDistance() )  # default cost function
        self.cost_fn_min = kwargs.get( 'cost_fn_min', 0. )
        self.cost_fn_max = kwargs.get( 'cost_fn_max', 10. )

    def update( self, sensor_id, sensor_trackers ):
        """ performs fusion of sensor_id, [TrackedObject].  returns list of TrackedObjectGroup """

        unmatched_groups = copy.copy(self.tracked_groups)  # shallow copy; list of unmatched groups
        unmatched_trackers = copy.copy( sensor_trackers )  # shallow copy; list of unmatched sensor_trackers

        # for each currently tracked group, if that group has a tracker for the current sensor_id, update it
        for i, group in enumerate( unmatched_groups ):
            trk_idx = group.update_from_sensor_trackers( sensor_id, unmatched_trackers )
            if not trk_idx is None:
                unmatched_trackers[trk_idx]=None
                unmatched_groups[i]=None

        # remove None
        unmatched_groups = filter( None, unmatched_groups )
        unmatched_trackers = filter( None, unmatched_trackers ) 

        # unmatched sensor detections:  attempt to match with unmatched groups, otherwise create new group
        if len(unmatched_trackers) > 0:

            # flatten.  create list of (group, TrackedObject) from unmatched groups.  Each group yields 1+ TrackedObject
            unmatched_group_trackers = []
            for group in unmatched_groups:
                unmatched_group_trackers.extend( [ ( group, trk[1] ) for trk in group.trackers ] )

            # match sensor trackers with existing group detections
            match_indices = matching( 
                [ gt[1] for gt in unmatched_group_trackers ],  # create list of TrackedObject
                unmatched_trackers,
                self.cost_fn, self.cost_fn_min, self.cost_fn_max
                )
        
            # update groups with sensor trackers
            for match in match_indices:
                unmatched_tracker_idx = match[1]  # index in unmatched_tracker
                sensor_tracker = unmatched_trackers[unmatched_tracker_idx]  # get sensor tracker object
                group = unmatched_group_trackers[match[0]][0]  # get group
                group.trackers.append( (sensor_id, sensor_tracker) )  # append to trackers

                # debug
                #group_tracker = unmatched_group_trackers[match[0]][1]
                #print("group id={}, cost={}, matching=\n{}\n{}".format( group.id, self.cost_fn( group_tracker, sensor_tracker ), str(sensor_tracker), str(group_tracker) ) )

                unmatched_trackers[unmatched_tracker_idx]=None  # remove from unmatched_trackers

            # remaining unmatched trackers:  group birth
            for trk in filter( None, unmatched_trackers ):
                self.tracked_groups.append( TrackedObjectGroup( self._id, ( sensor_id, trk ) ) )
                self._id += 1

        # finally, prune dead tracker groups
        dead_groups = [ i for i,group in enumerate(self.tracked_groups) if len(group.trackers) == 0 ]
        for i in reversed(dead_groups):
            self.tracked_groups.pop(i)

        return self.tracked_groups
