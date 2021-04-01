#!/usr/bin/env python
# BEGIN ALL
# BEGIN SHEBANG
#!/usr/bin/env python
# END SHEBANG
 
# BEGIN IMPORT
import os  
import yaml
import rospy
import re
import json
import io
# END IMPORT
 
# BEGIN STD_MSGS
#from indoor_navigation.msg import ctrmode
from indoor_navigation.msg import status
from indoor_navigation.msg import panangle
from indoor_navigation.msg import panctrmode
# END STD_MSGS
 
 
rospy.init_node('topic_publisher')
 
# BEGIN PUB
#pub1 = rospy.Publisher('ctrmode', ctrmode, queue_size=10)
pub2 = rospy.Publisher('status', status, queue_size=10)
pub3 = rospy.Publisher('panangle', panangle, queue_size=10)
pub4 = rospy.Publisher('panctrmode', panctrmode, queue_size=10)
# END PUB
 
# BEGIN LOOP
rate = rospy.Rate(10)
#a=ctrmode()
b=status()
d=panangle()
c=panctrmode()
#a.ctrmodeback=1
b.status=1
c.panctrmodeback=1
d.yaw=1.03
d.pitch=2.02
d.roll=3.01
i=0
while not rospy.is_shutdown():
        b.status=i%5
        #a.ctrmodeback=i%8+1
        c.panctrmodeback=i%3+1
	#pub1.publish(a)
	pub2.publish(b)
	pub3.publish(d)
	pub4.publish(c)
        i=i+1
        rate.sleep()
# END LOOP
# END ALL
