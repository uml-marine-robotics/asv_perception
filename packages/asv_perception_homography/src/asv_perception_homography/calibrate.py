"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

# UI tool for online calibration of rgb and radar
import rospy
import yaml
import io, os
from std_msgs.msg import Empty
from PyTunerTk import Tuner, show_info_messagebox, show_yesno_messagebox

pub_refresh = (None,None)  # (topic name str, rospy Publisher)
tuner = None
yaml_path_prefix = os.environ['ASV_PERCEPTION_CONFIG'] + 'calib' # save/load location, relative to this file

# defaults for 1280x1024, 1024x1024 radar image @ r=110m
def get_default_parameters():
    return {
    'topic': '/camera0/homography'
    , 'roll': 0.0
    , 'pitch': 80.6
    , 'imu_pitch_alpha': 1.4
    , 'imu_pitch_beta': 0.2
    , 'imu_pitch_gamma': 0.0
    , 'imu_roll_alpha': 0.0
    , 'imu_roll_beta': 0.0
    , 'imu_roll_gamma': 0.0
    , 'yaw': 4.0
    , 'fovy': 46.3
    , 'tx': 5.0
    , 'ty': 6.2
    , 'tz': -4.4
    }

def topic_to_fname( topic ):
    "return full filename path for saving/loading"
    return yaml_path_prefix + str(topic).replace( '/', '_' ) + ".yaml"

def create_param_path( topic, param ):
    result = str(topic)
    if not result.endswith('/'):
        result += '/'
    return result + param

def cb_tuner_topic_change( d ):

    assert( 'topic' in d and len(d['topic']) > 0 )
    topic = d['topic']

    global pub_refresh
    if pub_refresh[0] == topic: # no change needed
        return

    show_tuner(d)

def cb_tuner_change( d ):
    
    assert( 'topic' in d and len(d['topic']) > 0 )
    topic = d['topic']

    for (k,v) in d.items():
        if k == 'topic':
            continue
        rospy.set_param( create_param_path( topic, k ), v )

    global pub_refresh
    #if pub_refresh[0] != topic or pub_refresh[1] is None:
    #    pub_refresh = (topic, rospy.Publisher( create_param_path( topic, "refresh" ), Empty, queue_size=1))
    assert pub_refresh[1]
    pub_refresh[1].publish( Empty() )

# writes the dictionary to the file system, returns file name
def write_yaml( d ):
    fname = topic_to_fname( d['topic'] )
    with open( fname, 'w' ) as f:
        yaml.safe_dump( d, f )
    return fname

def cb_tuner_save( d ):
    try:
        if show_yesno_messagebox("Are you sure you want to save these parameters?"):
            show_info_messagebox( "Parameters saved to %s" % write_yaml(d) )
    except Exception as e:
        show_info_messagebox(str(e))

def cb_tuner_reset( d ):
    try:
        if show_yesno_messagebox("Are you sure you want to reset these parameters?"):
            show_tuner( d )
    except Exception as e:
        show_info_messagebox(str(e))

def cb_tuner_defaults( d ):
    try:
        if show_yesno_messagebox("Are you sure you want to reset these parameters to default?"):
            show_info_messagebox( "Parameters reset to default in %s.  Please restart the application to view changes" % write_yaml( get_default_parameters() ) )
    except Exception as e:
        show_info_messagebox(str(e))

def show_tuner( params ):

    global tuner

    if tuner:
        tuner.close()

    # attempt to load params
    fname = topic_to_fname( params['topic'])
    try:
        with open( fname ) as f:
            params = yaml.safe_load( f )
        print("Successfully loaded data from %s" % fname )
    except IOError:
        print("File not found: %s" % fname )

    # default parameters
    if params is None or len(params) == 0:
        params = get_default_parameters()

    # set up pub_refresh state
    assert( 'topic' in params and len( params['topic']) > 0 )
    topic = params['topic']
    global pub_refresh
    pub_refresh = (topic, rospy.Publisher( create_param_path( topic, "refresh" ), Empty, queue_size=1))

    #  ('entry', label, default, command )
    #  ('slider', label, default, min, max, resolution, command )
    #  ('button', label, command )

    tuner = Tuner( [
        ( 'entry', 'topic', params['topic'], cb_tuner_topic_change )
        , ( 'slider', 'roll', params['roll'], -180., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'pitch', params['pitch'], -180., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'imu_pitch_alpha', params['imu_pitch_alpha'], -10., 10., 0.1, cb_tuner_change )
        , ( 'slider', 'imu_pitch_beta', params['imu_pitch_beta'], -2., 2., 0.01, cb_tuner_change )
        , ( 'slider', 'imu_roll_alpha', params.get( 'imu_roll_alpha', 0.0 ), -10., 10., 0.1, cb_tuner_change )
        , ( 'slider', 'imu_roll_beta', params.get( 'imu_roll_beta', 0.0 ), -2., 2., 0.01, cb_tuner_change )
        , ( 'slider', 'yaw', params['yaw'], -180., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'fovy', params['fovy'], 0., 180., 0.01, cb_tuner_change )
        , ( 'slider', 'tx', params['tx'], -512., 512., 0.01, cb_tuner_change )
        , ( 'slider', 'ty', params['ty'], -512., 512., 0.01, cb_tuner_change )
        , ( 'slider', 'tz', params['tz'], -512., 512., 0.01, cb_tuner_change )
        , ( 'button', 'Save', cb_tuner_save )
        , ( 'button', 'Reset', cb_tuner_reset )
        , ( 'button', 'Defaults', cb_tuner_defaults )
        ]
        , 'Calibration'
    )
    
    tuner.show()
    
if __name__ == "__main__":

    rospy.init_node( "calibrate" )

    params = get_default_parameters()

    show_tuner(params)
    