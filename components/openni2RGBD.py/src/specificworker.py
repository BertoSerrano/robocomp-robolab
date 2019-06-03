#
# Copyright (C) 2019 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp. If not, see <http://www.gnu.org/licenses/>.
#
import os, sys
import numpy as np
from genericworker import *
import traceback
from primesense import openni2
from primesense import _openni2 as c_api


# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

        self.pcl = []
        self.openni2 = openni2

        OPENNI2_LIB = ''

        self.rgb_stream = None
        self.depth_stream = None
        self.device = None
        try:
            OPENNI2_LIB = os.environ['OPENNI2_LIB']
        except:
            pass
        finally:
            if len(OPENNI2_LIB) < 1:
                # print 'specificworker.py: OPENNI2_LIB environment variable not set! Exiting.'
                # sys.exit()
                print 'OPENNI2_LIB environment not set'
                OPENNI2_LIB = '/usr/local'
        self.openni2.initialize(OPENNI2_LIB)
        if (self.openni2.is_initialized()):
            print('OpenNI2 is initialized . . .')
        else:
            print('OpenNI2 is NOT INITIALIZED')
            exit(0)

    def __del__(self):
        print 'SpecificWorker destructor'
        self.depth_stream.stop()
        self.openni2.unload()
        self.rgb_stream.stop()

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        print 'SpecificWorker.compute...'

        # computeCODE
        try:
            self.device = self.openni2.Device.open_any()
            print self.device

            self.rgb_stream = self.device.create_color_stream()
            self.depth_stream = self.device.create_depth_stream()
            self.depth_stream.set_video_mode(
                c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM,
                                   resolutionX=320, resolutionY=240, fps=30))
            self.depth_stream.set_mirroring_enabled(False)
            self.rgb_stream.set_mirroring_enabled(False)

            #  Start the streams
            self.rgb_stream.start()
            self.depth_stream.start()

            while True:
                pass

        except Ice.Exception, e:
            traceback.print_exc()
            print e

        except Exception as e:
            traceback.print_exc()
            print e
            self.__del__()
            exit(0)

        # The API of python-innermodel is not exactly the same as the C++ version
        # self.innermodel.updateTransformValues("head_rot_tilt_pose", 0, 0, 0, 1.3, 0, 0)
        # z = librobocomp_qmat.QVec(3,0)
        # r = self.innermodel.transform("rgbd", z, "laser")
        # r.printvector("d")
        # print r[0], r[1], r[2]

        return True

    #
    # getRegistration
    #
    def getRegistration(self):
        ret = Registration()
        #
        # implementCODE
        #
        return ret

    #
    # getData
    #
    def getData(self):
        #
        # implementCODE
        #
        rgbMatrix = imgType()
        distanceMatrix = depthType()
        hState = RoboCompJointMotor.MotorStateMap()
        bState = RoboCompGenericBase.TBaseState()
        return [rgbMatrix, distanceMatrix, hState, bState]

    #
    # getXYZ
    #
    def getXYZ(self):
        #
        # implementCODE
        #
        points = PointSeq()
        hState = RoboCompJointMotor.MotorStateMap()
        bState = RoboCompGenericBase.TBaseState()
        return [points, hState, bState]

    #
    # getRGB
    #
    def getRGB(self):
        #
        # implementCODE
        #
        bgr = np.fromstring(self.rgb_stream.read_frame().get_buffer_as_uint8(), dtype=np.uint8).reshape(240 * 320, 3)
        # import ctypes
        # bgr = np.fromstring(self.rgb_stream.read_frame().get_buffer_as(ctypes.c_byte), dtype=np.byte).reshape(240 * 320, 3)
        # bgr = self.rgb_stream.read_frame().get_buffer_as(ctypes.c_byte).reshape(240 * 320, 3)
        # bgr = np.fromstring(self.rgb_stream.read_frame().get_buffer_as_uint8(), dtype=np.uint8).reshape(240 * 320, 3)
        # rgb = np.interp(bgr, [0, 255], [0.0, 1.0])

        color = []

        print(" BGR type:  ", type(bgr), " BGR LEN", len(bgr))

        pix = bgr[1200]
        print("TYPE PIX ((pix = bgr[3])) : ", type(pix))
        print("TYPE LEN ((pix = bgr[3])) : ", len(pix))
        print (" TYPE PIX[0] : ", type(pix[0]))

        othertry = ColorRGB()
        print othertry
        othertry.red = chr(pix[2])
        othertry.green = chr(pix[1])
        othertry.blue = chr(pix[0])
        print(" show an example of Color RGB : ", othertry, "\n and its type: ", type(othertry))

        othertry = ColorRGB(red=pix[2], green=pix[1], blue=pix[0])
        print "NOW AFTER USING CHR"
        print(" show an example of Color RGB : ", othertry, "\n and its type: ", type(othertry))

        othertry = ColorRGB(red=int(pix[2]), green=int(pix[1]), blue=int(pix[0]))
        print "JUST INTEGERS"
        print(" show an example of Color RGB : ", othertry, "\n and its type: ", type(othertry))

        for pixel in bgr:
            c = ColorRGB(red=chr(pixel[2]), green=chr(pixel[1]), blue=chr(pixel[0]))
            color.append(c)

        # color = [ColorRGB(bytes(r), bytes(g), bytes(b)) for b, g, r in bgr]

        hState = 0
        bState = 0
        return [color, hState, bState]

    #
    # getRGBDParams
    #
    def getRGBDParams(self):
        ret = TRGBDParams()
        #
        # implementCODE
        #
        return ret

    #
    # getDepth
    #
    def getDepth(self):
        #
        # implementCODE
        #
        depth = DepthSeq()
        hState = RoboCompJointMotor.MotorStateMap()
        bState = RoboCompGenericBase.TBaseState()
        return [depth, hState, bState]

    #
    # setRegistration
    #
    def setRegistration(self, value):
        #
        # implementCODE
        #
        pass

    #
    # getXYZByteStream
    #
    def getXYZByteStream(self):
        #
        # implementCODE
        #
        pointStream = imgType()
        hState = RoboCompJointMotor.MotorStateMap()
        bState = RoboCompGenericBase.TBaseState()
        return [pointStream, hState, bState]

    #
    # getImage
    #
    def getImage(self):
        #
        # implementCODE
        #
        color = ColorSeq()
        depth = DepthSeq()
        points = PointSeq()
        hState = RoboCompJointMotor.MotorStateMap()
        bState = RoboCompGenericBase.TBaseState()
        return [color, depth, points, hState, bState]

    #
    # getDepthInIR
    #
    def getDepthInIR(self):
        #
        # implementCODE
        #
        distanceMatrix = depthType()
        hState = RoboCompJointMotor.MotorStateMap()
        bState = RoboCompGenericBase.TBaseState()
        return [distanceMatrix, hState, bState]
