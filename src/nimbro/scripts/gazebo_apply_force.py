#!/usr/bin/env python

# Script to quickly apply forces to the nimbro_op model in Gazebo
# Author: Sebastian Schueller

import optparse
from math import sin, cos

import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import Point, Wrench

def polar2D_to_cartesian3D((r, theta)):
	return (r*cos(theta), r*sin(theta), 0.0)


def call_service(force, torque, model, ref_model, link, ref, duration):
	rospy.wait_for_service('/gazebo/apply_body_wrench')
	try:
		apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
		apply_body_wrench(
			model + '::' + link,
			ref_model + '::' + ref,
			Point(0.0,0.0,0.0),
			Wrench(
				Point(*force),
				Point(*torque)
				),
			rospy.Time(0,0),
			rospy.Duration.from_sec(duration))
		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e


if __name__ == "__main__":
	parser = optparse.OptionParser()
	parser.add_option("-f", "--force", action="store", dest="force", type="float", nargs=2, default=(0.0,0.0), help="2D force vector in the form: 'impulse theta'")
	parser.add_option("-t", "--torque", action="store", dest="torque", type="float", nargs=2, default=(0.0,0.0), help="2D torque vector in the form: 'impulse theta'")
	parser.add_option("-p", "--pure", action="store_true", dest="use_force", default=False, help="If set, the force and torque commands will assume force instead of impulse")
	parser.add_option("-d", "--duration", action="store", dest="duration", type="float", default=0.001, help="Duration of the applied force in seconds (float)")
	ref_group = optparse.OptionGroup(parser, "Reference Options", "Options used to specify the point where the force is applied")
	ref_group.add_option("-m", "--model", action="store", dest="model", default="nimbro_op", help="The model to apply the force/torque to. Defaults to 'nimbro_op'")
	ref_group.add_option("-M", "--ref_model", action="store", dest="ref_model", help="The model used as the reference coordinate system. Defaults to the model the force/torque is applied to")
	ref_group.add_option("-l", "--link", action="store", dest="link", default="trunk_link", help="The link of the model to apply force/torque to. Defaults to 'trunk_link'")
	ref_group.add_option("-r", "--ref", action="store", dest="ref", help="The link used as the reference coordinate system. Defaults to the link the force/torque is applied to")
	parser.add_option_group(ref_group)
	(opt, arg) = parser.parse_args()
	
	if opt.force == (0,0) and opt.torque == (0,0) :
		parser.error("Neither the force nor torque to apply have been set! Try --help.")
		
	#Set defaults for the coordinate reference system
	if not opt.ref:
		opt.ref = opt.link
	if not opt.ref_model:
		opt.ref_model = opt.model
	
	#Calculate force from impulse if --pure is not set
	if not opt.use_force:
		opt.force = (opt.force[0] / opt.duration, opt.force[1])
		opt.torque = (opt.torque[0] / opt.duration, opt.torque[1])
	
	call_service(
		polar2D_to_cartesian3D(opt.force),
		polar2D_to_cartesian3D(opt.torque),
		opt.model,
		opt.ref_model,
		opt.link,
		opt.ref,
		opt.duration)
