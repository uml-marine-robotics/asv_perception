# UI tool for online calibration of rgb and radar
import rospy
import yaml
import io
from std_msgs.msg import Empty
from PyTunerTk import Tuner, show_info_messagebox, show_yesno_messagebox

pub_refresh = (None,None)  # (topic name str, rospy Publisher)
yaml_path_prefix = '../launch/calib'

# defaults for 1280x1024, 1024x1024 radar image @ r=110m
def get_default_parameters():
    return {
    'topic': '/left_camera/homography'
    , 'roll': 0.0
    , 'pitch': 80.6
    , 'yaw': 4.0
    , 'fovy': 46.3
    , 'tx': 5.0
    , 'ty': 6.2
    , 'tz': -4.4
    }

def topic_to_fname( topic ):
    return yaml_path_prefix + str(topic).replace( '/', '_' ) + ".yaml"

def create_param_path( topic, param ):
    result = str(topic)
    if not result.endswith('/'):
        result += '/'
    return result + param

def cb_tuner_change( d ):
    
    assert( 'topic' in d and len(d['topic']) > 0 )
    topic = d['topic']

    for (k,v) in d.items():
        if k == 'topic':
            continue
        rospy.set_param( create_param_path( topic, k ), v )

    global pub_refresh
    if pub_refresh[0] != topic:
        pub_refresh = (topic, rospy.Publisher( create_param_path( topic, "refresh" ), Empty, queue_size=1))
    pub_refresh[1].publish( Empty() )

# writes the dictionary to the file system, returns file name
def write_yaml( d ):
    fname = topic_to_fname( d['topic'] )
    with open( fname, 'w' ) as f:
        yaml.safe_dump( d, f )
    return fname

def cb_tuner_save( d ):
    if show_yesno_messagebox("Are you sure you want to save these parameters?"):
        show_info_messagebox( "Parameters saved to %s" % write_yaml(d) )

def cb_tuner_reset( d ):
    if show_yesno_messagebox("Are you sure you want to reset these parameters to default?"):
        show_info_messagebox( "Parameters reset to default in %s.  Please restart the application to view changes" % write_yaml( get_default_parameters() ) )
    
if __name__ == "__main__":

    rospy.init_node( "calibrate" )

    # default parameters
    params = get_default_parameters()

    # attempt to load params
    fname = topic_to_fname( params['topic'])
    try:
        with open( fname ) as f:
            params = yaml.safe_load( f )
        print("Successfully loaded data from %s" % fname )
    except IOError:
        print("File not found: %s" % fname )

    tuner = Tuner( [
        ( 'entry', 'topic', params['topic'], None )
        , ( 'slider', 'roll', params['roll'], -180., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'pitch', params['pitch'], -180., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'yaw', params['yaw'], -180., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'fovy', params['fovy'], 0., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'tx', params['tx'], -512., 512., 0.01, cb_tuner_change )
        , ( 'slider', 'ty', params['ty'], -512., 512., 0.01, cb_tuner_change )
        , ( 'slider', 'tz', params['tz'], -512., 512., 0.01, cb_tuner_change )
        , ( 'button', 'Save', cb_tuner_save )
        , ( 'button', 'Defaults', cb_tuner_reset )
        ]
        , 'Calibration'
    )
    
    tuner.show()
    