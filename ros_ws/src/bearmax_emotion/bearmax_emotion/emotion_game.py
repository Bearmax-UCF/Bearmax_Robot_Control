import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from bearmax_msgs.msg import Emotion, EmotionGameState, EmotionScores, StackRequest
from bearmax_msgs.action import Task
from std_msgs.msg import String
from typing import Callable

from bearmax_emotion.emotion_lib.src.gameNode import EmotionGame
from bearmax_emotion.utils import state_to_msg, new_req
from datetime import datetime, timedelta
import json


# TODO: Handle pause/resume/start/end as service server
class GameNode(Node):

    def __init__(self):
        super().__init__(
            "emotion_game",
            automatically_declare_parameters_from_overrides=True
        )

        self._last_emotion_cb_time = None
        self._last_emotion = None
        self.round_in_progress = False

        self.emotion_in_sub = self.create_subscription(
            Emotion,
            "/emotion_in",
            self.emotionCallback,
            1
        )

        self.stack_sub = self.create_subscription(
            String,
            "/stack_out",
            self.stackCallback,
            1
        )

        # We publish the game state to make debugging easier
        self.state_out_pub = self.create_publisher(
            EmotionGameState,
            "/state_out",
            1
        )

        self.to_stack_pub = self.create_publisher(
            StackRequest,
            "/stack_in",
            1
        )

        self.task_client = ActionClient(self, Task, "task")
        self._target_emotion = None

        timer_period = 0.5  # seconds
        self._timer = self.create_timer(timer_period, self.publish_state)

        self._game = EmotionGame(self.logger, self.send_to_stack)
        self._game.registerCallback("new_round", self.handle_new_round)
        self._game.registerCallback("on_win", self.handle_on_win)
        self._game.registerCallback("on_lose", self.handle_on_lose)

    @property
    def emotion_threshold(self) -> int:
        """
        The amount of time (milliseconds) needed for an emotion
        to be submitted to the emotion game instance.
        """
        return self.get_parameter(
            "emotion_threshold"
        ).get_parameter_value().integer_value

    @property
    def logger(self): return self.get_logger()

    def send_task(self, e: str, on_fin: Callable):
        self.logger.info(f"Running Emotion task: {e}")

        self.task_client.wait_for_server()

        goal_msg = Task.Goal()
        goal_msg.task_name = e

        self.logger.info("Sending task request...")

        self._send_task_future = self.task_client.send_goal_async(goal_msg)

        self._send_task_future.add_done_callback(
            self.task_goal_response_callback)

        self._task_finished_cb = on_fin

    def task_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._task_finished_cb()
            return

        self._goal_handle = goal_handle

        if self._target_emotion == "happy" or self._target_emotion is None:
            self._get_task_result_future = goal_handle.get_result_async()
            self._get_task_result_future.add_done_callback(
                self.task_result_callback)
        else:
            self._timer = self.create_timer(3.0, self.timer_callback)

    def task_result_callback(self, future):
        status = future.result().status

        self.logger.info(f"Task {status}")

        self._task_finished_cb()

    def timer_callback(self):
        future = self._goal_handle.cancel_goal_async()

        future.add_done_callback(self._task_finished_cb)

        self._timer.cancel()

    # TODO: Is this dead code?
    def handle_new_round(self, new_emotion: str, round_cb: Callable):
        self._last_emotion = None
        self._correct_emotion_count = 0
        self._last_emotion_cb_time = None
        self._target_emotion = new_emotion
        self.logger.info(f"Received new round emotion: {new_emotion}")

        def cb(*_, **__):
            """Runs when we ack new emotion"""
            def on_fin(*_, **__):
                """Runs when task is finished"""
                self.logger.info(
                    "Finished task, watching for emotion")
                self.round_in_progress = True
                round_cb()

            self.send_task(new_emotion, on_fin)

        self.send_to_stack(
            "speak",
            f"Let's pretend to be {new_emotion}", cb)


    def handle_on_win(self, cb: Callable):
        def win_cb(*_, **__):
            self.round_in_progress = False
            self.logger.warn("Win task not yet implemented!")
            cb()
        self.send_to_stack("speak", "Great job!", win_cb)

    def handle_on_lose(self, detected_emotion: str, target_emotion: str, cb: Callable):
        def lose_cb(*_, **__):
            self.round_in_progress = False
            self.logger.warn("Lose task not yet implemented!")
            cb()
        self.send_to_stack("speak", "That's OK! Let's try another.", lose_cb)

    def publish_state(self):
        # self.state_out_pub.publish(state_to_msg(self._game.state))
        pass  # TODO: Fix error "The 'target_emotion' field must be of type 'str'"

    def emotionCallback(self, data):
        if not self.round_in_progress:
            return

        # how many correct frames are needed to succeed
        correct_frames_needed = 5

        if data.emotion in ["happy", "sad", "angry"]:
            self._last_emotion = data.emotion

        if data.emotion == self._target_emotion:
            self._correct_emotion_count += 1

        # If we got enough correct frames, proceed.
        if self._correct_emotion_count >= correct_frames_needed:
            self._last_emotion_cb_time = datetime.now()
            self._game.handleEmotionChange(self._target_emotion)

        if self._last_emotion_cb_time is None:
            self._last_emotion_cb_time = datetime.now()

        dur = datetime.now() - self._last_emotion_cb_time
        threshold_dur = timedelta(
            seconds=(self.emotion_threshold / 1000))

        # If we go overtime without meeting the
        # correct frame amount, send the game the last valid emotion
        if dur >= threshold_dur:
            self._last_emotion_cb_time = datetime.now()
            self._game.handleEmotionChange(self._last_emotion)

    def stackCallback(self, data):
        action = data.data
        self.logger.info(f"Received from stack: {action}")
        if action == "emotionStart":
            def cb(*_, **__):
                self._game.start()
            self.send_to_stack("speak", "Let's play a game!", cb)
        elif action == "emotionStop":
            final_score_str = self._game.end().to_json_str()
            self.send_to_stack("emotionGameStats", final_score_str)
            self.logger.info(
                f"Game ended successfully! Final score: {final_score_str}")
            self.send_to_stack("speak", "Thanks for playing!")
        elif action[:3] == "ACK":
            if self._ack_callback is not None:
                _cb = self._ack_callback
                self._ack_callback = None
                _cb()
        elif action == "recalibrate":
            def on_fin(*_, **__):
                self.logger.info("RESETTING HEAD")
            self.send_task("reset", on_fin)


    def on_shutdown(self):
        if self._game.state.started:
            final_score_str = self._game.end().to_json_str()
            self.send_to_stack("emotionGameStats", final_score_str)
            self.logger.info(
                f"Game ended successfully! Final score: {final_score_str}")

    def send_to_stack(self, event, data, cb: Callable = None):
        self._ack_callback = cb
        self.to_stack_pub.publish(
            new_req(event, str(data)))


def main(args=None):
    rclpy.init(args=args)

    emotion_game = GameNode()

    # try:
    #     rclpy.spin(emotion_game)
    # except:
    #     # The context is already gone at this point
    #     emotion_game.on_shutdown()
    # rclpy.try_shutdown()

    while rclpy.ok():
        rclpy.spin_once(emotion_game)

    emotion_game.destroy_node()
    rclpy.shutdown()
