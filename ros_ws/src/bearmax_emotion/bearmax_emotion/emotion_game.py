import rclpy
from rclpy.node import Node
from bearmax_msgs.msg import Emotion, EmotionGameState, EmotionScores
from bearmax_emotion.emotion_lib.src.gameNode import EmotionGame
from bearmax_emotion.utils import state_to_msg
from datetime import datetime, timedelta


# TODO: Handle pause/resume/start/end as service server
class GameNode(Node):

    def __init__(self):
        super().__init__(
            "emotion_game",
            automatically_declare_parameters_from_overrides=True
        )

        self._last_emotion_cb_time = None
        self._last_emotion = None
        self.round_in_progress = True

        self.emotion_in_sub = self.create_subscription(
            Emotion,
            "/emotion_in",
            self.callback,
            1
        )

        # We publish the game state to make debugging easier
        self.state_out_pub = self.create_publisher(
            EmotionGameState,
            "/state_out",
            1
        )
        timer_period = 0.5  # seconds
        self._timer = self.create_timer(timer_period, self.publish_state)

        self._game = EmotionGame()
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

    def handle_new_round(self):
        self.logger.warn("New Round task not implemented!")
        self._last_emotion = None
        self._last_emotion_cb_time = None
        self.round_in_progress = True

    def handle_on_win(self):
        self.round_in_progress = False
        self.logger.warn("Win task not implemented!")

    def handle_on_lose(self, detected_emotion: str, target_emotion: str):
        self.round_in_progress = False
        self.logger.warn("Lose task not implemented!")

    def publish_state(self):
        self.state_out_pub.publish(state_to_msg(self._game.state))

    def callback(self, data):
        if not self.round_in_progress: return

        if self._last_emotion_cb_time is None:
            self._last_emotion_cb_time = datetime.now()
            self._last_emotion = data.emotion
            return

        dur = datetime.now() - self._last_emotion_cb_time
        threshold_dur = timedelta(
            seconds=(self.emotion_threshold / 1000))

        if dur >= threshold_dur and self._last_emotion == data.emotion:
            self._last_emotion_cb_time = datetime.now()
            self._game.handleEmotionChange(data.emotion)
        elif self._last_emotion != data.emotion:
            self._last_emotion_cb_time = datetime.now()
            self._last_emotion = data.emotion

    def on_shutdown(self):
        if self._game.state.started:
            final_score = self._game.end()
            self.logger.info(
                f"Game ended with Final Score: {final_score}")


def main(args=None):
    rclpy.init(args=args)

    emotion_game = GameNode()

    try:
        rclpy.spin(emotion_game)
    except:
        # The context is already gone at this point
        emotion_game.on_shutdown()

    rclpy.try_shutdown()
