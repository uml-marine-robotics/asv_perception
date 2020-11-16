"""
# License

Each contributor holds copyright over their contributions to Caffe-Tensorflow. In particular:

- Any included network model is provided under its original license.

- Any portion derived from Caffe is provided under its original license.

- Caffe-tensorflow is provided under the MIT license, as specified below.

# The MIT License (MIT)

Copyright (c) 2016 Saumitro Dasgupta

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import sys

SHARED_CAFFE_RESOLVER = None

class CaffeResolver(object):
    def __init__(self):
        self.import_caffe()

    def import_caffe(self):
        self.caffe = None
        try:
            # Try to import PyCaffe first
            import caffe
            self.caffe = caffe
        except ImportError:
            # Fall back to the protobuf implementation
            from . import caffepb
            self.caffepb = caffepb
            show_fallback_warning()
        if self.caffe:
            # Use the protobuf code from the imported distribution.
            # This way, Caffe variants with custom layers will work.
            self.caffepb = self.caffe.proto.caffe_pb2
        self.NetParameter = self.caffepb.NetParameter

    def has_pycaffe(self):
        return self.caffe is not None

def get_caffe_resolver():
    global SHARED_CAFFE_RESOLVER
    if SHARED_CAFFE_RESOLVER is None:
        SHARED_CAFFE_RESOLVER = CaffeResolver()
    return SHARED_CAFFE_RESOLVER

def has_pycaffe():
    return get_caffe_resolver().has_pycaffe()

def show_fallback_warning():
    msg = '''
------------------------------------------------------------
    WARNING: PyCaffe not found!
    Falling back to a pure protocol buffer implementation.
    * Conversions will be drastically slower.
    * This backend is UNTESTED!
------------------------------------------------------------

'''
    sys.stderr.write(msg)
