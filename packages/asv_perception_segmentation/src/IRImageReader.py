#!/usr/bin/env python
import sys
import os
import argparse
from os import walk
from os import listdir
#import wasr_models
import numpy as np
import kaffe
from kaffe import tensorflow as tf

helpStr = '''
          The IRImageReader reads IR image and it associated mask
          '''

class IRImageReader(object):
    ''' IR image reader which reads gray-scale images
    '''
    def __init__(self, data_dir, file_data_list, input_size):
        ''' The init function initialzes the class by checking validity of data_dir
        file_data_list etc.
        data_dir : The directory containing training images and their mask images
        file_data_list : File containing names of images & their mask images in following format -
        /path/to/image /path/to/mask_image
        '''
         #irReader = ImageReader(data_dir, data_list, input_size, false, false, false)
        self.data_dir = data_dir
        self.file_data_list = file_data_list

        cwd = os.getcwd()
        print("Current dir '%s'" % cwd)

        if not os.path.isfile(file_data_list):
            print("File '%s' does not exist." % file_data_list)

        if not os.path.isdir(data_dir):
            print("Directory '%s' does not exist." % data_dir)

        fileH_IRImageList = open(file_data_list, 'r')
        linesInFile = fileH_IRImageList.readlines()

        self.irimage = []
        self.irimageMask = []
        for line in linesInFile:
            imagefile, maskfile = line.strip("\r\n").split(' ')
            self.irimage.append(data_dir + imagefile)
            self.irimageMask.append(data_dir + maskfile)

    def readImages(self):
        self.images = tf.convert_to_tensor(self.irimage, dtype=tf.string)
        self.labels = tf.convert_to_tensor(self.irimageMask, dtype=tf.string)
        

        pass

# process command line
if len(sys.argv) < 2:
    print ("Not enough arguments!")
    print ("Usage: IRImageReader <IRImageDir> <FileContainingIRImageAndMask>")
    exit()

if "-h" in sys.argv:
    print(helpStr)

if __name__ == '__main__':
    main()  