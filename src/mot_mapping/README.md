# FAPP perception integration

Source: https://github.com/arclab-hku/FAPP
Revision: dbd8173bd1518419b5e33faa7eecb75ed9d9cdcd (MIT).

Only perception and original EKF tracking are integrated. Input clouds must
already be registered in the odometry frame. No planner consumes these outputs.
ROS topics, frame IDs and timestamps are adapted for UAV1 FAST-LIO. Tracking
uses cloud timestamp intervals instead of the upstream fixed 0.02 seconds.

Launch: `roslaunch mot_mapping uav1_detection.launch`.

The six/seven Terminator entrypoints default to FAPP; `--detector ldop` selects
the existing LDOP detector. RViz groups are configured in the existing UAV1
profiles. Static point output and upstream planning are not enabled here.

Smoke test (after sourcing the workspace):
`ROS_MASTER_URI=http://127.0.0.1:11431 ROS_IP=127.0.0.1 python3 src/mot_mapping/test/perception_smoke.py`
The test starts and stops its own local ROS master and perception launch.
