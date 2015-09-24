#!/usr/bin/python
# -*- coding: utf-8 -*-
__author__ = 'sebastian'

import sys
import yaml


def mirror_data(motionfile):
    mf = file(motionfile, 'r')
    data = yaml.load(mf)
    
    # Adjust the motion file header
    header = data['header']
    header['name'] = header['name'] + '_mirrored'
    header['playState'] = header['playState'] + '_mirrored'
    
    # Adjust the motion keyframes
    motion = data['motion']
    for frame in motion:
        joints = frame["joints"]
        
        relb_pitch = joints['right_elbow_pitch']['position']
        rsho_pitch = joints['right_shoulder_pitch']['position']
        rhip_pitch = joints['right_hip_pitch']['position']
        rank_pitch = joints['right_ankle_pitch']['position']
        rkne_pitch = joints['right_knee_pitch']['position']
        rhip_roll = joints['right_hip_roll']['position']
        rsho_roll = joints['right_shoulder_roll']['position']
        rank_roll = joints['right_ankle_roll']['position']
        rhip_yaw = joints['right_hip_yaw']['position']

        joints['right_elbow_pitch']['position'] = joints['left_elbow_pitch']['position']
        joints['right_shoulder_pitch']['position'] = joints['left_shoulder_pitch']['position']
        joints['right_hip_pitch']['position'] = joints['left_hip_pitch']['position']
        joints['right_ankle_pitch']['position'] = joints['left_ankle_pitch']['position']
        joints['right_knee_pitch']['position'] = joints['left_knee_pitch']['position']
        joints['right_hip_roll']['position'] = -joints['left_hip_roll']['position']
        joints['right_shoulder_roll']['position'] = -joints['left_shoulder_roll']['position']
        joints['right_ankle_roll']['position'] = -joints['left_ankle_roll']['position']
        joints['right_hip_yaw']['position'] = -joints['left_hip_yaw']['position']

        joints['left_elbow_pitch']['position'] = relb_pitch
        joints['left_shoulder_pitch']['position'] = rsho_pitch
        joints['left_hip_pitch']['position'] = rhip_pitch
        joints['left_ankle_pitch']['position'] = rank_pitch
        joints['left_knee_pitch']['position'] = rkne_pitch
        joints['left_hip_roll']['position'] = -rhip_roll
        joints['left_shoulder_roll']['position'] = -rsho_roll
        joints['left_ankle_roll']['position'] = -rank_roll
        joints['left_hip_yaw']['position'] = -rhip_yaw

        joints['neck_yaw']['position'] = -joints['neck_yaw']['position']

    return data


def main(argv):
    for mf in argv:
        print 'Mirroring Motion %s' % mf
        out = mirror_data(mf)
        ofile = file(mf[:-5] + '_mirrored' + mf[-5:], 'w')
        yaml.dump(out, ofile, default_flow_style=False)


if __name__ == '__main__':
    main(sys.argv[1:])