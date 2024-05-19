# state/set_repeat_last.py
import smach

class SetRepeatLast(smach.State):
    def __init__(self, goto_gnss_state):
        smach.State.__init__(self, outcomes=['done'])
        self.goto_gnss_state = goto_gnss_state

    def execute(self, userdata):
        self.goto_gnss_state.use_last_coordinate = True
        return 'done'

