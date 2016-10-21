#!/bin/bash


#for i in ../igus_op/*.stl; do
#	fn=$(basename "$i")
#	meshlabserver -i "$i" -o "$fn" -s convex_hull.mlx -om vn
#done

for i in eye_mask_link.stl head_full.stl left_foot_link.stl left_shank_link.stl left_shoulder_pitch_link.stl right_hip_yaw_link.stl right_shank_link.stl; do
	meshlabserver -i "$i" -o "$i" -s invert_faces.mlx -om vn
done
