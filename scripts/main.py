#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan as ls
from sensor_msgs.msg import PointCloud2 as pc2
from laser_geometry import LaserProjection
import math
import ros_numpy
import numpy as np
from icp2 import Align2D
import matplotlib.pyplot as plt
from copy import deepcopy


def plot_data(data_1, label_1, markersize_1=8):
    fig = plt.figure(figsize=(10, 6))
    
    ax = fig.add_subplot(111)
    ax.axis([-10, 10, -10, 10])

    if data_1 is not None:
        x_p, y_p = data_1
        ax.plot(x_p, y_p, color='#336699', markersize=markersize_1, marker='o', linestyle=":", label=label_1)
    ax.legend()
    return ax

class Laser2PC(object):
    sweep_cnt = 0


    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/laserPointCloud", pc2, queue_size=1)

        self.convertpcPub = rospy.Publisher("/convertPointCloud", pc2, queue_size=1)

        self.laserSub = rospy.Subscriber("/scan", ls, self.laserCallback)
        self.cur_data = None
        self.prev_data = None
        self.move_data = [[],[]]

        self.rot_degree = 0
        self.move_coor = [0, 0]

    def laserCallback(self, data): 
        cloud_out = self.laserProj.projectLaser(data)
        

        if(self.cur_data is not None):
            self.prev_data = np.copy(self.cur_data)

        self.cur_data = ros_numpy.numpify(cloud_out)
        self.cur_data = np.array(map(list, self.cur_data))

        self.cur_data = np.delete(self.cur_data, np.s_[2:], axis=1)
        self.cur_data = np.c_[self.cur_data, np.ones(len(self.cur_data))]

        if self.prev_data is not None:

            aligner = Align2D(self.cur_data, self.prev_data)
            matched_trg, matched_src, indices = aligner.FindCorrespondences(self.cur_data)
            T = aligner.transform

            if T[0,0] >= 1.0:
                T[0,0] = 1.0
            elif T[0,0] <= -1.0:
                T[0,0] = -1.0
            if T[1,0] >= 1.0:
                T[1,0] = 1.0
            elif T[1,0] <= -1.0:
                T[1,0] = -1.0

            acos_val = round((180/math.pi) * math.acos(T[0,0]), 7)
            asin_val = round((180/math.pi) * math.asin(T[1,0]), 7)

            rotation_degree = 0

            if 0 <= acos_val <= 90 and 0 < asin_val <= 90:
                rotation_degree = acos_val
            elif 90 < acos_val <= 180 and 0 < asin_val <=90:
                rotation_degree = acos_val
            elif 0 <= acos_val <= 90 and -90 <= asin_val <= 0:
                rotation_degree = -acos_val
            elif 90 < acos_val <= 180 and -90 <= asin_val <= 0:
                rotation_degree = -acos_val

            self.rot_degree += round(rotation_degree, 1)
            print(T)

            print("rotation degree1 : {}".format(rotation_degree))

            print("rotation degree2 : {}".format(self.rot_degree))

            self.move_coor[0] += round(T[0,2], 4); self.move_coor[1] += round(T[1,2], 4)

            print('11X diff : {}, Y diff : {}'.format(T[0,2], T[1,2]))

            print('22X diff : {}, Y diff : {}'.format(self.move_coor[0], self.move_coor[1]))
            self.move_data[0].append(self.move_coor[0]); self.move_data[1].append(self.move_coor[1])

        if Laser2PC.sweep_cnt == 200:
            plot_data(self.move_data, "M : moving path")
            plt.show()

        self.pcPub.publish(cloud_out)


        print(Laser2PC.sweep_cnt)
        Laser2PC.sweep_cnt += 1



if __name__ == '__main__':
    rospy.init_node("laser2PointCloud")
    l2pc = Laser2PC()
    rospy.spin()

