predictions = {
    "can_shoot": None,
    "closest_enemy": None,
    "own_goal": None,
    "goal": None,
    "ball_struct": None,
    "teammates_from_goal": []
}


def set_prediction(item, value):
    predictions[item] = value


def get_predictions():
    return predictions
