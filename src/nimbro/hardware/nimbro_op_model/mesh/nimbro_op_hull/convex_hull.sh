#!/bin/bash


for i in ../nimbro_op/*.STL; do
	fn=$(basename "$i")
	meshlabserver -i "$i" -o "$fn" -s convex_hull.mlx -om vn
done

for i in trunk_link.STL right_shank_link.STL left_shank_link.STL right_ankle_link.STL right_lower_arm_link.STL left_hip_roll_link.STL left_shoulder_pitch_link.STL; do
	meshlabserver -i "$i" -o "$i" -s invert_faces.mlx -om vn
done
