uint8 BEHAVIOR_BASELINE=0
uint8 BEHAVIOR_LONGITUDINAL=1
uint8 BEHAVIOR_VERTICAL=2
uint8 BEHAVIOR_TURNING=3
uint8 BEHAVIOR_LATERAL=4

# Behavior family represented by this branch. BEHAVIOR_BASELINE is only
# for the first-stage model-based baseline extrapolation.
uint8 behavior_type

# Normalized signed behavior strength in [-1,1]. Baseline uses 0.
float64 behavior_value

# Branch probability. First-stage baseline uses 1.0.
float64 probability

DynamicObjectPredictionPoint[] points
