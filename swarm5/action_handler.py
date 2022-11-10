import rclpy

class ActionHandler(rclpy.node.Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.done, self.accepted = False, False
        self.goal_msg = None
        self.goal_handle = None
    def _response_callback(self, future):
        self.goal_handle = future.result()
        self.accepted = self.goal_handle.accepted
        newFuture = self.goal_handle.get_result_async()

        newFuture.add_done_callback(self._result_callback)
    
    def _result_callback(self, future):
        self.goal_handle = None
        self.done = True

    def start(self, *args, **kwargs):
        # must define self._start() yourself, which returns a goal_msg
        # must also define self._client which is a client for that action type
        self.goal_msg = self._start(*args, **kwargs)

        self.done, self.accepted = False, False
        self._client.wait_for_server()
        future = self._client.send_goal_async(self.goal_msg)
        future.add_done_callback(self._response_callback)

    def startBlocking(self, *args, **kwargs):
        self.goal_msg = self._start(*args, **kwargs)

        self.done, self.accepted = False, False
        self._client.wait_for_server()
        future = self._client.send_goal_async(self.goal_msg)
        future.add_done_callback(self._response_callback)

        while not self.done:
            rclpy.spin_once(self)

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0)
        return (self.done, self.accepted)

    def reset(self, *args, **kwargs):
        self.stop()
        self.start(*args, **kwargs)

    def stop(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

        self.goal_handle = None
        self.done, self.accepted = False, False
        self.goal_msg = None