"""Actual timestamp references for an independent offline experiment.

Unused by the ROS entry point. Selected tuples retain their original frame,
pose and event mask; missing references are not interpolated or substituted.
Ordinary measured-center tracking is outside this helper.
"""


def select_timed_history(stamp, history):
    """Select actual 0.4/0.2-second-old frames within 0.050001 seconds.

    Return available references in target-age order. Equal age errors choose
    the newer actual timestamp, regardless of the input history order.
    """
    selected = []
    for target_age in (.4, .2):
        options = [frame for frame in history
                   if 0 < stamp-frame[0] and abs((stamp-frame[0])-target_age) <= .050001]
        if options:
            selected.append(min(options, key=lambda frame: (abs((stamp-frame[0])-target_age), -frame[0])))
    return selected
