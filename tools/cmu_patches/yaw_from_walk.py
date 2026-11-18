"""Measure how far the lidar cloud is yawed from the IMU body frame.

Record both odometries while you walk the robot straight FORWARD (remote,
no sideways stick) a few metres:
    python3 dual_odometry_record.py          # on the robot, Foxy shell, ~150 s
then run this on /tmp/dual_rec.json (columns: t, SLAM x y z yaw, leg x y z yaw).

It prints the direction of travel in each robot's own body frame. The leg
odometry (/utlidar/robot_odom) is forward by definition, so "offset" is the
yaw to add to the cloud (cloud_yaw_deg in ~/Desktop/level_correction.yaml).
Healthy after the correction: forward offset within a few degrees of 0.
"""
import json, math, sys
import numpy as np

r = np.array(json.load(open(sys.argv[1] if len(sys.argv) > 1 else '/tmp/dual_rec.json')))
S, L = r[:, 1:5], r[:, 5:9]


def body(P, i, j):
    dx, dy, y = P[j, 0] - P[i, 0], P[j, 1] - P[i, 1], P[i, 3]
    return dx * math.cos(y) + dy * math.sin(y), -dx * math.sin(y) + dy * math.cos(y)


rows = []
for i in range(0, len(r) - 5):
    a, b = body(L, i, i + 5), body(S, i, i + 5)
    if math.hypot(*a) > 0.12 and math.hypot(*b) > 0.05:
        rows.append((math.degrees(math.atan2(a[1], a[0])), math.degrees(math.atan2(b[1], b[0]))))
R = np.array(rows)
fwd = R[np.abs(R[:, 0]) < 25]
if len(fwd) < 3:
    sys.exit('not enough forward motion in the recording')
d = (fwd[:, 1] - fwd[:, 0] + 180) % 360 - 180
z = np.exp(1j * np.radians(d)).mean()
print('forward windows %d: SLAM direction minus leg direction = %+.1f deg (circular std %.1f)' % (
    len(fwd), math.degrees(np.angle(z)), math.degrees(math.sqrt(max(0, -2 * math.log(abs(z)))))))
print('if this is far from 0, add %+.1f to cloud_yaw_deg' % (-math.degrees(np.angle(z))))
