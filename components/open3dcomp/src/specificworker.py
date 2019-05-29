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
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
from math import isnan

from genericworker import *

# If RoboComp was compiled with Python bindings you can use InnerModel in Python
# sys.path.append('/opt/robocomp/lib')
# import librobocomp_qmat
# import librobocomp_osgviewer
# import librobocomp_innermodel
import open3d as o3
import cv2
import numpy as np
import traceback


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.timer.timeout.connect(self.compute)
        self.Period = 2000
        self.timer.start(self.Period)

    def __del__(self):
        print 'SpecificWorker destructor'

    def setParams(self, params):
        # try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        # except:
        #	traceback.print_exc()
        #	print "Error reading config params"
        return True

    @QtCore.Slot()
    def compute(self):
        try:
            # color, depth, _, _ = self.rgbd_proxy.getData()
            nube, _, _ = self.rgbd_proxy.getXYZ()
            colors, _, _ = self.rgbd_proxy.getRGB()
            print len(nube)
            self.pintar(nube, colors)
        # image = np.frombuffer(color, dtype=np.uint8)
        # print image.size
        # image = np.reshape(image, (480, 640, 3))
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # cv2.imshow("color", image)
        # print (min(depth), max(depth))
        # depth = np.interp(depth, [min(depth), max(depth)], [0, 255])
        # print(min(depth), max(depth))
        # depth = np.reshape(depth, (480, 640, 1))
        # cv2.imshow("depth", depth.astype(np.uint8))
        # cv2.waitKey(5)
        # print len(color)
        except Ice.Exception, e:
            traceback.print_exc()
            print e

        return True

    def pintar(self, nube, colors):
        #
        # points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0],
        #           [0, 0, 1], [1, 0, 1], [0, 1, 1], [1, 1, 1]]
        # lines = [[0, 1], [0, 2], [1, 3], [2, 3],
        #          [4, 5], [4, 6], [5, 7], [6, 7],
        #          [0, 4], [1, 5], [2, 6], [3, 7]]
        # colors = [[1, 0, 0] for i in range(len(lines))]
        # line_set = o3.LineSet()
        # line_set.points = o3.Vector3dVector(points)
        # line_set.lines = o3.Vector2iVector(lines)
        # line_set.colors = o3.Vector3dVector(colors)
        # o3.draw_geometries([line_set])

        xyz, rgb = [], []
        print nube[10000:10100]
        # print colors[10000:10100]
        print type(nube)
        whites = 0
        nan_count = 0
        for p, c in zip(nube, colors):
            if not (isnan(p.x) or isnan(p.y) or isnan(p.z)):
                xyz.append([p.x, p.y, p.z])
                if not (isnan(c.red) or isnan(c.green) or isnan(c.blue)):
                    rgb.append([c.red, c.green, c.blue])
                else:
                    rgb.append([0, 0, 255])
                if c.red==255 and c.green==255 and c.blue==255:
                    whites+=1
            else:
                nan_count+=1
        print "Ostia que blanco " + str(whites)
        print "Ostia que NAN " + str(nan_count)

        # colors = [[1, 4, 0] for i in range(len(xyz))]
        # Pass xyz to Open3D.PointCloud and visualize
        pcd = o3.PointCloud()

        # pcd = o3.LineSet()
        pcd.points = o3.Vector3dVector(xyz)
        pcd.colors = o3.Vector3dVector(rgb)
        # downpcd = o3.voxel_down_sample(pcd, voxel_size=0.1)
        o3.draw_geometries([pcd])

        # print(np.asarray(pcd.points))
        # o3.draw_geometries([pcd])

    # print("Downsample the point cloud with a voxel of 0.05")
