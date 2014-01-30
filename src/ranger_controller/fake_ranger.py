import logging
from monitor import Monitor
from ranger import sm
import smach

import rospy
import smach_ros
rospy.init_node("fake_ranger_controller")

import threading

logging.getLogger("rosout").setLevel(logging.ERROR)
logging.getLogger("rospy").setLevel(logging.ERROR)

try:
    with Monitor() as monitor:

        t = threading.Thread(target=monitor.run)
        t.start()

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        outcome = sm.execute()

        rospy.spin()
        sis.stop()

except smach.exceptions.InvalidUserCodeError as iuce:

    import traceback
    traceback.print_exc()
    print(iuce.message)

