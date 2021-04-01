#python脚本启动launch文件
import roslaunch
import rospy
import time

#rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(
    uuid, ["/home/x805/catkin_ws/src/indoor_navigation/launch/savemap.launch"])#启动保存地图的launch文件
launch.start()

rospy.sleep(3)
#try:
  #launch.spin()
#finally:
  # After Ctrl+C, stop all nodes from running
launch.shutdown()

