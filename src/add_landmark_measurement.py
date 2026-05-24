import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_landmark_measurement(graph, initial_estimate, result):
    # Determine the correct rotation (bearing) and distance from X(4) to L(2) 
    relVec = np.array([4 - 5.41421356, 2 - 1.41421356])
    dirVec = np.array([0, 1])

    rotation = np.arccos(np.dot(relVec, dirVec)/(np.linalg.norm(relVec)*np.linalg.norm(dirVec)))
    distance = np.linalg.norm(relVec)
    print(rotation, np.degrees(rotation))
    graph.add(gtsam.BearingRangeFactor2D(X(4), L(2), gtsam.Rot2.fromDegrees(np.degrees(rotation)), distance, MEASUREMENT_NOISE))
    return graph