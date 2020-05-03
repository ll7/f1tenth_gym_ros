from unrealcv.util import read_png, read_npy
from unrealcv import client
import cv2
import pdb
import time
import matplotlib.pyplot as plt
# plt.ion()

# unreal_origin = [1345.0, 3110.0, 132.0]
unreal_origin = [1345.0, 3110.0, 132.0 + 50]
client.connect()
print(client.request('vget /unrealcv/version'))
print(client.request('vget /unrealcv/status'))

#1) Get Pose
pose_str = client.request('vget /camera/0/pose')
x, y, z, pitch, yaw, roll = list(map(float, pose_str.split()))

#2) Set Pose (we only care about yaw)
client.request(f"vset /camera/0/pose {unreal_origin[0]} {unreal_origin[1]} {unreal_origin[2]} {0} {180} {0}")
time.sleep(0.01)

#3) Get result
img = cv2.cvtColor(read_npy(client.request('vget /camera/0/lit npy'))
, cv2.COLOR_RGB2BGR)   

cv2.imshow("unrealcam", img)
cv2.waitKey(0)