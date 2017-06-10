#!/usr/bin/env python

from matplotlib import pyplot as plt
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from smooth_map.srv import smoothmap_srv

class Map(object):
    def __init__(self, origin_x=-9.95, origin_y=-9.95, resolution=.05,width=2000, height=2000):
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width))

    def to_message(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),Quaternion(0, 0, 0, 1))

        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

class Mapper(object):
    def __init__(self,temp_map,origin_x,origin_y):
        self._map = Map(origin_x,origin_y)
        self._map_pub = rospy.Publisher('map', OccupancyGrid, latch=True, queue_size=10)
        self._map_data_pub = rospy.Publisher('map_metadata', MapMetaData, latch=True, queue_size=10)
        self.scan_callback(temp_map)


    def scan_callback(self, temp_map):
        image = np.zeros((2000,2000))
        for i in range(0,len(temp_map[0])):
                self._map.grid[temp_map[0][i], temp_map[1][i]] = 1.0
                image[temp_map[0][i], temp_map[1][i]] = 1.0

        self.publish_map()


    def publish_map(self):
        grid_msg = self._map.to_message()
        self._map_data_pub.publish(grid_msg.info)
        self._map_pub.publish(grid_msg)


class IOI():
    def __init__(self):
        rospy.init_node('smooth_map_ioi')
        self.x = np.zeros(0)
        self.y = np.zeros(0)
        self.x_origin = 0
        self.y_origin = 0
        self.smooth_map = rospy.Service('/Smooth_Map', smoothmap_srv, self.SmoothMap_Service)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, self.callback)

        rospy.loginfo("IOI Smooth map server is on")
        rospy.spin()

    def SmoothMap_Service(self,req):
        try:
            self.processmap()
            temp_map = self.map_demo()

            m = Mapper(temp_map,self.x_origin,self.y_origin)
            rospy.loginfo("Published IOI Smooth map interpolation")
            return True
        except:
            rospy.loginfo("Failed to update global map")
            return False

    def processmap(self):
        rel_x = np.around((self.x - self.cur_x) / self.res)
        rel_y = np.around((self.y - self.cur_y) / self.res)

        fix_px_x = round(self.cur_x/self.res)
        fix_px_y = round(self.cur_y/self.res)

        fix_px_x = int(200 - fix_px_x)
        fix_px_y = int(200 - fix_px_y)

        map_data = np.asarray(self.data)

        plt.figure(1,figsize=(6, 6))
        map_data = map_data.reshape((self.data_info_height, self.data_info_width))

        for i in range(0,len(rel_x)):
            temp_rel_x = int(rel_x[i])
            temp_rel_y = int(rel_y[i])
            temp_x = 200 + temp_rel_y
            temp_y = 200 + temp_rel_x

            map_data[temp_x-2:temp_x+2, temp_y-2:temp_y+2] = 70

        map_data[198:202, 198:202] = 50
        plt.imshow(map_data, plt.cm.gist_heat)
        np.save('map',map_data)
        plt.savefig('1.jpg')
        plt.clf()
        rospy.loginfo("BAT MAP Interpolation - Processing Local Map.")

    def callback(self,data):
        print "Got Local_Map"
        self.cur_x = data.info.origin.position.x + 9.95
        self.cur_y = data.info.origin.position.y + 9.95
        self.x_origin = data.info.origin.position.x
        self.y_origin = data.info.origin.position.y
        self.res = data.info.resolution
        self.x = np.append(self.cur_x, self.x)
        self.y = np.append(self.cur_y, self.y)
        self.data = data.data

        self.data_info_height = data.info.height
        self.data_info_width = data.info.width

    def map_demo(self):
        gray = np.load("map.npy")
        gray = np.array(gray, dtype = np.uint8)

        ## 99 - Obstacle, 70 - Path, 50 - Current Position

        inflation_radius = 1
        iter = 3

        for j in range(0,iter):
            obstacle_location = np.where( gray == 99 )
            for i in range(0,len(obstacle_location[0])):
                rec = gray[obstacle_location[0][i]-inflation_radius:obstacle_location[0][i]+inflation_radius,obstacle_location[1][i]-inflation_radius:obstacle_location[1][i]+inflation_radius]

                if len(np.where(rec==70)[0])==0:
                    gray[obstacle_location[0][i]-5:obstacle_location[0][i]+5,obstacle_location[1][i]-5:obstacle_location[1][i]+5] = 99
                else:
                    print "Cannot inflate obstacle"


            path_location = np.where( gray == 70 )


        return obstacle_location

if __name__ == "__main__":
    IOI()
