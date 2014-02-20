from smach import State, StateMachine

from ranger_aseba import get_robot, ID

ranger = get_robot()

# import the navigation state machine
from ranger_navigation import sm_nav

class ChargingIdle(State):
    def __init__(self, outcomes = ['lolette_removed']):

        State.__init__(self, outcomes=outcomes)

    def execute(self, userdata):
        while True:
            if not ranger.lolette:
                return 'lolette_removed'

class ChargingActiveWait(State):
    """ Waits until the beacon emits, ie, the child has put the
    lolette on the beacon.
    """
    def __init__(self):
        State.__init__(self, 
                        outcomes = ['succeeded', 'aborted'],
                        output_keys = ["destination"])

    def execute(self, userdata):
        while True:
            
            if ranger.lolette:
                return 'aborted'

            if ID["BEACON"] in ranger.beacons and \
               not ranger.beacons[ID["BEACON"]].obsolete():

                userdata.destination = ID["BEACON"]
                return 'succeeded'


class WaitInteractionEnd(State):
    def __init__(self):
        State.__init__(self, 
                       outcomes = ['succeeded'],
                       output_keys = ["destination"])

    def execute(self, userdata):
        while True:
            if ranger.lolette:
                userdata.destination = ID["MYSTATION"]
                return 'succeeded'



sm = StateMachine(outcomes = ['failure'])

with sm:
    StateMachine.add('Charging Idle', ChargingIdle(),
                     transitions = {'lolette_removed': 'Charging Active Wait'})

    StateMachine.add('Charging Active Wait', ChargingActiveWait(),
                     transitions = {'succeeded': 'Nav to beacon',
                                    'aborted': 'Charging Idle'},
                     remapping = {"destination": "destination"})

    StateMachine.add('Nav to beacon', sm_nav,
                     transitions = {'succeeded': 'Wait Interaction End',
                                    'aborted': 'Nav to station'},
                     remapping = {"target": "destination"})

    StateMachine.add('Wait Interaction End', WaitInteractionEnd(),
                     transitions = {'succeeded': 'Nav to station'})

    StateMachine.add('Nav to station', sm_nav,
                     transitions = {'succeeded': 'Charging Idle',
                                    'aborted': 'Nav to station'},
                     remapping = {"target": "destination"})


if __name__ == "__main__":

    import rospy
    import smach_ros
    rospy.init_node("ranger_controller")


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()

    rospy.spin()
    sis.stop()

    robot.close()
