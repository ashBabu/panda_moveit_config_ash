#!/usr/bin/env python
# -*- coding: UTF-8 -*-



import tf
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import MarkerArray


class OctomapCollisionGenerator(object):
    def __init__(self):
        super(OctomapCollisionGenerator, self).__init__()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.sub_occ = rospy.Subscriber('/occupied_cells_vis_array', MarkerArray, self.occupied_cb)
        # self.subl_free = rospy.Subscriber('/free_cells_vis_array', MarkerArray, self.free_cb)
        # self.subl_free = rospy.Subscriber('/free_cells_vis_array', MarkerArray, self.free_cb)
        self.last_size = -1

    def occupied_cb(self, marker_array):
        ps = PoseStamped()
        for m in marker_array.markers:
            ps.header = m.header
            ps.pose = m.pose
            total_loop_count = max(self.last_size, len(m.points))
            self.last_size = len(m.points)

            for i in range(600):
                try:
                    ps.pose.position = m.points[i]
                    self.scene.add_box(str(i), ps, (0.05, 0.05, 0.05))
                except IndexError:
                    pass
                    # self.scene.remove_world_object(str(i))

    # def free_cb(self, marker_array):
    #     for m in marker_array.markers:
    #         self.scene.remove_world_object(m.id)

def main():
    rospy.init_node('collision_generator')
    ocg = OctomapCollisionGenerator()
    rospy.spin()

if __name__ == '__main__':
    main()