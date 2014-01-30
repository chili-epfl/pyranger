from ranger_aseba import get_robot, ID
from smach import State, StateMachine

ranger = get_robot()

class DetectTarget(State):
    def __init__(self):
        State.__init__(self, outcomes = ['detected', 'failed', 'aborted'],
                             input_keys = ['target_in'])

    def execute(self, userdata):

        if userdata.target_in not in ID:
            return 'failed'
        
        id = ID[userdata.target_in]

        while True:
            if ranger.lolette:
                return 'aborted'

            if id in ranger.beacons and \
               not ranger.beacons[id].obsolete():
            
                   return 'detected'

            #TODO: code behaviour to find target


class Move(State):
    def __init__(self):
        State.__init__(self, outcomes = ['reached', 'aborted'],
                            input_keys = ['target_in'],
                            output_keys = ['target_out'])

    def execute(self, userdata):
        while True:
            if ranger.lolette:
                userdata.target_out = ID["MYSTATION"]
                return 'aborted'

class Dock(State):
    def __init__(self):
        State.__init__(self, outcomes = ['succeeded', 'aborted'],
                       input_keys = ['target_in'],
                       output_keys = ['target_out'])


    def execute(self, userdata):
        while True:
            if ranger.lolette:
                userdata.target_out = ID["MYSTATION"]
                return 'aborted'



sm_nav = StateMachine(outcomes = ["succeeded", "aborted"],
                      input_keys = ["target"],
                      output_keys = ["nexttarget"])

with sm_nav:
    StateMachine.add("Detect Target", DetectTarget(),
                     transitions = {'detected': 'Move to Target',
                                    'failed': 'Detect Target',
                                    'aborted': 'aborted'},
                     remapping = {"target_in": "target"})

    StateMachine.add("Move to Target", Move(),
                     transitions = {'reached':'Dock',
                                    'aborted': 'aborted'},
                     remapping = {"target_in": "target",
                                  "target_out": "nexttarget"})

    StateMachine.add("Dock", Dock(),
                     transitions = {'succeeded':'succeeded',
                                    'aborted': 'aborted'},
                     remapping = {"target_in": "target",
                                  "target_out": "nexttarget"})



