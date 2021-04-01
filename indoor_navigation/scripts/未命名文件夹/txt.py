
# coding=utf-8
import re
import json
import io
from geometry_msgs.msg import PoseStamped 
from rospy_message_converter import json_message_converter
from nav_msgs.msg import Path
if __name__ == '__main__':
    f = open("guiji/guiji1.txt","r") 
    a=f.readlines()
    str=''
    y5={}
    i=0
    ite = Path()
    for line in a[0:]:
        seq = re.compile(" ")
        lst = seq.split(line.strip())
        p= PoseStamped()
        ite.poses.append(p)
        ite.poses[i].pose.position.x = float(lst[0])
        ite.poses[i].pose.position.y = float(lst[1])
        i=i+1
    path_json_ = json_message_converter.convert_ros_message_to_json(ite)
    print path_json_
   # print(type(str))
    #print(a[2:])
    f.close()
    #text=json.loads(str)
    #text
