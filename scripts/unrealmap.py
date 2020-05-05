"""
Interactive script that converts your unreal map screenshot to an actual map
"""
import cv2
import numpy as np
from scipy import stats
import pdb

#SET DEBUGMODE = True to see cv2 output
DEBUGMODE = True

#FILL THIS IN
origin_world = [1343.0, 3104.0, 131.0]

filepath = "../maps/unreal_map.png"
input_map = cv2.imread(filepath)
bg_rgb = np.array([35, 35, 35])
bg = cv2.inRange(input_map, bg_rgb, bg_rgb)

# Get length of line
#TODO: Get line_crop coordinates from mouseclick
line_crop_bw = bg[1000:1040, 70:120]
line_crop_rgb = input_map[1000:1040, 70:120]
fg_idx = np.nonzero(line_crop_bw == 0.)
y, length = stats.mode(fg_idx[0])
if DEBUGMODE:
    y_idxs = np.where(fg_idx[0] == y)[0]
    start_idx, end_idx = y_idxs[0], y_idxs[-1]
    line_idxs = (fg_idx[0][start_idx:end_idx], fg_idx[1][start_idx:end_idx])
    line_crop_rgb[line_idxs] = np.array([255.0, 0., 0.])
    cv2.imshow("DEBUG line_crop", line_crop_rgb)
    print("Press window to continue")
    cv2.waitKey(0)

# Get start position coordinates (center of red_square)
#TODO: Get map_crop coordinates from mouseclick
map_crop_bw = bg[:900, :]
map_crop_rgb = input_map[:900, :]
start_marker_rgb = np.array([0., 0., 166.])
red_square_idxs = np.where(map_crop_rgb == start_marker_rgb)
ry_start, ry_end = red_square_idxs[0][0], red_square_idxs[0][-1]
rx_start, rx_end = red_square_idxs[1][0], red_square_idxs[1][-1]
center_y = (ry_start + (ry_end - ry_start) / 2.)
center_x = (rx_start + (rx_end - rx_start) / 2.)
if DEBUGMODE:
    map_crop_rgb[center_y, center_x] = np.array([255., 255., 255.])
    origin_unreal = [center_y, center_x, 0.0]
    print("Click window to continue")
    cv2.imshow("DEBUG start location", map_crop_rgb)
    cv2.waitKey(0)
origin_x = -(center_x * 1./length[0])
origin_y = (center_y - map_crop_rgb.shape[0]) * 1./length[0]
origin_unreal = [origin_x, origin_y, 0.0]

#Convert map to bitmap representation (use map_crop)
bitmap_map = map_crop_bw
bitmap_map[ry_start:ry_end+1, rx_start:rx_end+1] = 255

cv2.imwrite("../maps/unreal.png", bitmap_map)

#Write other data to file
image = "unreal.png"
resolution = 1./length[0]
origin = origin_unreal
negate = 0
occupied_thresh = 0.65 #doesn't matter
free_thresh = 0.196 #doesn't matter
unreal_origin = origin_world
params = f"image: {image}\nresolution: {resolution}\norigin: {origin}\nnegate: {negate}\noccupied_thresh: {occupied_thresh}\nfree_thresh: {free_thresh}\nunreal_origin: {unreal_origin}"

f = open("../maps/unreal.yaml", "w")
f.write(params)
f.close()