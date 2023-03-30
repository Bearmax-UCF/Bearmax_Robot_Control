from bearmax_emotion.emotion_lib.src.gameNode import Scores, State
from bearmax_msgs.msg import EmotionGameState, EmotionScores, StackRequest
from dataclasses import asdict


def scores_to_msg(s: Scores) -> EmotionScores:
    """Converts Scores dataclass to ros msg"""
    msg = EmotionScores()
    for k, v in asdict(s).items():
        setattr(msg, k, v)
    return msg


def state_to_msg(s: State) -> EmotionGameState:
    """Converts State dataclass to ros msg"""
    msg = EmotionGameState()
    for k, v in vars(s).items():
        new_v = v
        if isinstance(v, Scores):
            new_v = scores_to_msg(v)
        setattr(msg, k, new_v)
    return msg


def new_req(event: str, data: str) -> StackRequest:
    req = StackRequest()
    req.event = event
    req.data = data
    return req
