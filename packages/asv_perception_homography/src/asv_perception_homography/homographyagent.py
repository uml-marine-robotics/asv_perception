"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import numpy as np

from .homography_utils import (
    optimize, get_adjusted_params, 
    create_camera_to_metric_matrix, create_metric_to_camera_matrix
)

from .maxentropylist import MaxEntropyList

class HomographyAgent(object):
    "Handles exemplar evaluation and parameter selection"

    def __init__( self, lb, ub, points_range, max_frames = 5, max_error=1000. ):

        self.max_frames = max_frames
        self.max_error = max_error
        self.calibration_lb = lb
        self.calibration_ub = ub
        self.points_range = points_range

        self.params = []  # current param set
        self.best_params = self.params

        self.img_w = 0 # segmented image width; should match rgb
        self.img_h = 0 # segmented image height; should match rgb

        self.temporal_max_entropy_frames = MaxEntropyList(self.max_frames)
        self.temporal_max_entropy_params = []
        
        self.global_max_entropy_frames = MaxEntropyList(self.max_frames*2)  # TODO separate param?
        self.global_max_entropy_params = []

        self.temporal_needs_training = False
        self.global_needs_training = False

    def create_camera_to_metric_matrix( self, imu ):
        return create_camera_to_metric_matrix( self.img_w, self.img_h, *get_adjusted_params( imu, self.best_params ), depth=self.points_range )

    def create_metric_to_camera_matrix( self, imu ):
        return create_metric_to_camera_matrix( self.img_w, self.img_h, *get_adjusted_params( imu, self.best_params ), depth=self.points_range )

    def evaluate( self, current_frame ):
        "Evaluate the provided frame (Exemplar); sets best_params and XXX_needs_training flags"

        # update img_w, img_h from segmentation image; needed for publishing camera -> metric matrix
        self.img_h, self.img_w = current_frame.img_h, current_frame.img_w
        
        current_frame_error_temporal = np.inf
        current_frame_error_global = np.inf
        
        # evaluate current frame error if we have an existing set of params
        if len(self.temporal_max_entropy_params) > 0:
            current_frame_error_temporal, _, _ = current_frame.evaluate( self.temporal_max_entropy_params )            
            if np.isnan(current_frame_error_temporal):
                current_frame_error_temporal=np.inf

        if len(self.global_max_entropy_params) > 0:
            current_frame_error_global, _, _ = current_frame.evaluate( self.global_max_entropy_params )
            if np.isnan(current_frame_error_global):
                current_frame_error_global=np.inf

        # TODO
        #else:
        #    self.publish_debug_visualization( seg, current_frame.points, current_frame_transform )
        print("Error: t={}, g={}".format( current_frame_error_temporal, current_frame_error_global ) )

        # decide which params to use
        if current_frame_error_temporal < current_frame_error_global:
            self.best_params = self.temporal_max_entropy_params
        else:
            self.best_params = self.global_max_entropy_params

        # if not currently optimizing, can change frames
        # use Exemplar.target_np() for information gain calc
        if not self.temporal_max_entropy_frames.lock.locked():
            with self.temporal_max_entropy_frames.lock:
                self.temporal_max_entropy_frames.append( current_frame, lambda x : x.target_np() )

                if current_frame_error_temporal > self.max_error and self.temporal_max_entropy_frames.is_full():
                    self.temporal_needs_training=True

        # only add current frame for global consideration if current frame error is reasonable
        if not self.global_max_entropy_frames.lock.locked() and min(current_frame_error_temporal, current_frame_error_global) < self.max_error:
        #if not self.global_max_entropy_frames.lock.locked():
            with self.global_max_entropy_frames.lock:
                self.global_max_entropy_frames.append( current_frame, lambda x : x.target_np() )

                if current_frame_error_global > self.max_error and self.global_max_entropy_frames.is_full():
                    self.global_needs_training=True

    def train( self, pso_minstep=1e-1, pso_swarmsize=50 ):
        "Evaluate need for training; perform training as needed.  Minstep and swarmsize are PSO parameters"

        if self.temporal_needs_training:
            with self.temporal_max_entropy_frames.lock:
                print("Starting temporal optimization; len=", len(self.temporal_max_entropy_frames.items) )
                
                # temporal best params
                self.temporal_max_entropy_params, err = optimize( self.calibration_lb, self.calibration_ub, self.temporal_max_entropy_frames.items, minstep=pso_minstep, swarmsize=pso_swarmsize )
                print("Temporal error=", err )
                self.temporal_max_entropy_frames.items = []  # clear temporal list
                self.temporal_needs_training=False

        
        if self.global_needs_training:
            print("Starting global optimization; len=", len(self.global_max_entropy_frames.items) )
            with self.global_max_entropy_frames.lock:
                self.global_max_entropy_params, err = optimize( self.calibration_lb, self.calibration_ub, self.global_max_entropy_frames.items, minstep=pso_minstep, swarmsize=pso_swarmsize )
                print("Global error=", err )
                self.global_needs_training=False