from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

class FeedforwardImuController( object ):
    """ 
        A simple feed forward controller for angular IMU data (RPY and velocity)
        Set RPY coefficients for x (alpha), velocity (beta), acceleration (gamma)
        TODO:  computed accel values can occasionally be janky, a filter of some sort would be nice
    """

    def __init__( self ):
        
        self.yaw = self.yaw_alpha = self.yaw_beta = self.yaw_gamma = 0.

        self.pitch = self.pitch_beta = self.pitch_gamma = 0.
        self.pitch_alpha = 1.

        self.roll = self.roll_beta = self.roll_gamma = 0.
        self.roll_alpha = 1.

        self._last_msg = None

    def update( self, msg ):
        """ Update controller with imu message; x=roll, y=pitch, z=yaw
            computes yaw, pitch, roll member vars, combining data with coefficients _alpha, _beta, _gamma
        """

        if self._last_msg is None:
            self._last_msg = msg

        # imu.orientation is a normalized quaternion.  euler_from_quaternion returns radians
        rpy = euler_from_quaternion( [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w] )

        velocity = msg.angular_velocity  # should be rad/s

        accel = [0.]*3

        # compute delta-t
        dt = ( msg.header.stamp - self._last_msg.header.stamp ).to_sec()
        
        if dt > 0.:
            last_velocity = self._last_msg.angular_velocity
            accel[0]= ( velocity.x - last_velocity.x ) / dt
            accel[1]= ( velocity.y - last_velocity.y ) / dt
            accel[2]= ( velocity.z - last_velocity.z ) / dt

        self._last_msg = msg

        self.roll = rpy[0]*self.roll_alpha + velocity.x*self.roll_beta + accel[0]*self.roll_gamma
        self.pitch = rpy[1]*self.pitch_alpha + velocity.y*self.pitch_beta + accel[1]*self.pitch_gamma
        self.yaw = rpy[2]*self.yaw_alpha + velocity.z*self.yaw_beta + accel[2]*self.yaw_gamma
