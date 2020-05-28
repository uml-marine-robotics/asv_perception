import cv2
import math
from collections import namedtuple

"""
control def for Tuner
    name, initial_value (int), max_value (int), centered (bool), precision (int)
   precision:  allow values at specified precision, values are scaled by 10^precision; eg for precision to two decimals, specify -2.  
               this is needed to work around highgui only allowing int values
"""
PyTunerCvControl = namedtuple('Control', ['id', 'initial', 'max', 'centered', 'precision'])

class PyTunerCv(object):
    """
    Python tuner for opencv
    A class for tuning settings using OpenCV 3+ and Py 2.7+
    """

    #: Window to show tuner in
    tunerWindowName = "PyTunerCvTuner"

    #: Window to show results in.  if you want them to be in the same window, set the display_window_name = window_name
    imageWindowName = "PyTunerCvImage"

    #: The keycode for the user to quit the application.  Default: 113 (q)
    quitKey = 113

    #: User-defined data to store and pass along with events
    userData = None

    #: Refresh rate for output updating in ms
    refreshRate = 250

    # props should be an array Control named tuple
    # cbUpdate accepts the input image and a map of prop and value, should return an image
    def __init__(self,  controls, cbRefresh, cbMouse = None, userData = None, sameWindow = False ):
        """Initialization

        Keyword arguments:
        controls -- array of PyTunerCvControl
        cbRefresh -- function with the parameters ( dictionary, userData ) to accept dictionary of PyTunerCvControl.id and value along any specified userData
        cbMouse -- See OpenCV setmousecallback
        userData -- user data to be supplied along with callbacks
        sameWindow -- If True, shows the image and the tuner controls in a single window.  If False, shows the image and tuner controls in separate windows
        """
        self.controls = controls
        self.cbRefresh = cbRefresh
        self.cbMouse = cbMouse
        self.userData = userData

        self._dirty = True

        if sameWindow:
            self.imageWindowName = self.tunerWindowName

    def _refresh(self, v = None ):
        """Refresh the image via client callback"""

        if self._dirty is False:
            return

        d = dict()
        for ctl in self.controls:
            val = cv2.getTrackbarPos(ctl.id, self.tunerWindowName)

            #offset for scale
            val *= math.pow(10, ctl.precision)

            if ctl.centered:  #offset for center
                val -= ctl.max/2.

            d[ctl.id]=val
            
        # call the update callback fn from the client, get the new img
        self.output = self.cbRefresh( d, self.userData )
        self._dirty = False
        """
        try:
            self.output = self.cbUpdate( d )
        except RuntimeError as e:  # warning:  this crashes when changing the sliders a lot (and/or quickly?)
            # todo:  solve 'maximum recursion depth exceeded while calling a Python object' error
            cv2.destroyAllWindows()
        """

    # event sink for trackbar refresh
    def _makeDirty(self, p = None):
        self._dirty = True

    def show(self):

        # construct window(s)
        cv2.namedWindow(self.imageWindowName, cv2.WINDOW_AUTOSIZE)
        if self.tunerWindowName != self.imageWindowName:
            cv2.namedWindow(self.tunerWindowName, cv2.WINDOW_GUI_NORMAL)
        
        if self.cbMouse is not None:
            cv2.setMouseCallback(self.imageWindowName, self.cbMouse, self.userData )

        for ctl in self.controls:
            
            prec = math.pow(10, ctl.precision * -1.)

            val = ctl.initial
            maxval = ctl.max
            if ctl.centered:
                val += ctl.max / 2.

            # scale to precision
            val *= prec
            maxval *= prec

            cv2.createTrackbar( 
                ctl.id
                , self.tunerWindowName
                , int(val)
                , int(maxval)
                , self._makeDirty
            )

        # begin refresh loop
        while True:
            self._refresh()
            cv2.imshow(self.imageWindowName, self.output )
            if cv2.waitKey(self.refreshRate) == self.quitKey:
                break
        
        # cleanup
        cv2.destroyWindow(self.tunerWindowName)
        cv2.destroyWindow(self.imageWindowName)
