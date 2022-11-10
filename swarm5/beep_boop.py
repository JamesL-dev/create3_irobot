from rclpy.action.client import ActionClient
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from irobot_create_msgs.action import AudioNoteSequence
from builtin_interfaces.msg import Duration
from action_handler import ActionHandler

class BoopDriver(ActionHandler):
    def __init__(self, robot_name):
        super().__init__(f"{robot_name}_audio_note_sequence")
        self._client = ActionClient(self, AudioNoteSequence, f'/{robot_name}/audio_note_sequence')

    def _start(self):
        frequencies = [900]
        notes = [AudioNote() for x in frequencies]
        for i in range(len(notes)):
            notes[i].frequency = frequencies[i]
            notes[i].max_runtime = Duration()
            notes[i].max_runtime.sec = 1
        note_sequence = AudioNoteVector()
        note_sequence.notes = notes

        goal_msg = AudioNoteSequence.Goal()
        goal_msg.iterations = 1
        goal_msg.note_sequence = note_sequence
        return goal_msg
