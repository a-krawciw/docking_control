#!/usr/bin/env python
import rospy
from mavros_utils import RCManager, BoatConfig

def main():
    rospy.init_node('position_hold')

    rc_controller = RCManager()
    rc_controller.set_raw_override()
    boat_config = BoatConfig(0.165, -0.0123/4, -80)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():

        rc_controller.setForceTorque(0.0, 0.0, boat_config=boat_config)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass