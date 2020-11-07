import rospy
import tf2_ros
import tf2_geometry_msgs
import tf2_sensor_msgs
import sensor_msgs.point_cloud2 as pc2
from asv_perception_common.msg import Obstacle
from geometry_msgs.msg import PoseStamped

class FrameTransformer(object):
    """ 
    Class which wraps ros tf2 transforms, obstacle specializations
    Initialize with sufficient time prior to calling a method so that the tf buffer has time to fill
    """

    def __init__(self):
        self.use_most_recent_tf = False  # use most recent tf time
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.buffer )

    def lookup_transform( self, src, dst, stamp ):
        """ looks up a transform from src to dst.  
        handles extrapolation exception and transforms to most recent on failure, setting use_current flag to True
        ultimately throws if lookup unsuccessful
        # returns geometry_msgs/TransformStamped:  http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TransformStamped.html
        """
        
        do_lookup = lambda : self.buffer.lookup_transform( dst, src, rospy.Time( 0 ) if self.use_most_recent_tf else stamp, rospy.Duration( 1.0 ) )

        try:
            return do_lookup()
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn( e.message )

            if self.use_most_recent_tf:
                raise e   # nothing more we can do

            rospy.logwarn( 'Using most recent time instead')
            self.use_most_recent_tf = True
        # other exception, assume unrecoverable.  eg tf2_ros.LookupException, tf2_ros.ConnectivityException

        return do_lookup() # try again, any exception is fatal

    def transform_pose( self, pose, **kwargs ):
        """
        Transform a pose 
        kwargs:
            ts: TransformStamped.  If not specified, then a dst_frame must be specified
            header:  ROS Header.  default={ stamp=rospy.Time.now(), frame_id='base_link' }
            dst_frame:  string, target frame
        """
        # tf2_geometry_msgs requires posestamped
        ps = PoseStamped()
        ps.pose = pose

        ts = kwargs.get('ts')
        if ts is None:
            hdr = kwargs.get('header', rospy.Header( stamp=rospy.Time.now(), frame_id='base_link' ) )
            ts = self.lookup_transform( hdr.frame_id, kwargs['dst_frame'], hdr.stamp )
        
        ps = tf2_geometry_msgs.do_transform_pose( ps, ts )
        return ps.pose

    def transform_cloud( self, cloud, ts ):
        """ pointcloud transform using ts """
        result = tf2_sensor_msgs.do_transform_cloud( cloud, ts )
        result.header.frame_id = ts.child_frame_id
        return result

    def transform_obstacle( self, obs, dst_frame ):
        """ transform obstacle in place using obstacle header time """

        if obs.header.frame_id == dst_frame:  # already done for this obstacle
            return

        ts = self.lookup_transform( obs.header.frame_id, dst_frame, obs.header.stamp )

        # obstacle header
        obs.header.frame_id = dst_frame

        # transform pose.position.  tf2_geometry_msgs uses posestamped
        obs.pose.pose = self.transform_pose( obs.pose.pose, ts=ts )

        # pointcloud transform if desired
        #if transform_points and not obs.points is None and obs.points.width > 0:
        #    obs.points = self.transform_cloud( obs.points, self.lookup_transform( obs.points.header.frame_id, dst_frame, obs.points.header.stamp ) )
