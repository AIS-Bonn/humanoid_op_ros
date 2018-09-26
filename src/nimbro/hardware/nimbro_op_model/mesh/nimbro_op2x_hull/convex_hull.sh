#!/bin/bash


#for i in ../nimbro_op2x_hull/*.STL; do
#	fn=$(basename "$i")
#	meshlabserver -i "$i" -o "$fn" -s convex_hull.mlx -om vn
#done

for i in hip_yaw_link.STL left_ankle_link.STL left_hip_link.STL left_lower_arm_link.STL left_shoulder_link.STL left_thigh_link.STL left_upper_arm_link.STL right_ankle_link.STL right_foot_link.STL right_hip_link.STL right_lower_arm_link.STL right_shoulder_link.STL right_thigh_link.STL right_upper_arm_link.STL shank_front_link.STL thigh_front_link.STL trunk_link.STL; do
	meshlabserver -i "$i" -o "$i" -s invert_faces.mlx -om vn
done
