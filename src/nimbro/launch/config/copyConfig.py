#!/usr/bin/python
# Author: Hafez Farazi <farazi@ais.uni-bonn.de>
#         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# Imports
import sys
import yaml
import optparse
import os.path
import shutil
from os.path import expanduser
from termcolor import colored

# Function to write an element of a hierarchical dictionary of unknown depth
def writePart(data, components, value):
	if len(components) == 1:
		data[components[0]] = value
		return
	writePart(data[components[0]], components[1:], value)

# Main function
def main():
	
	# Parse the arguments
	parser = optparse.OptionParser()
	parser.add_option('-f', '--from', type="string", dest="fromR", help="From", default="")
	parser.add_option('-t', '--to', type="string", dest="toR", help="To", default="")
	parser.add_option('-p', '--part', type="string", dest="part", help="Part like vision", default="all")
	parser.add_option('-b', '--baseP', type="string", dest="baseP", help="Base Path", default="HOME")
	options, args = parser.parse_args()
	
	# Process the arguments
	if(options.baseP == "HOME"):
		options.baseP = expanduser("~") + "/NimbRo-OP/src/nimbro/launch/config/"
	fromS = options.baseP + "config_" + options.fromR + ".yaml"
	toS = options.baseP + "config_" + options.toR + ".yaml"

	# Handle case where entire config file should be copied
	if(options.part == "all" and os.path.isfile(fromS)):
		shutil.copyfile(fromS, toS)
		print colored("All configs copied from " + options.fromR + " to " + options.toR, "green")
		sys.exit()

	# Error handling if files not found
	if(not os.path.isfile(fromS) or not os.path.isfile(toS)):
		print colored("Please specify correct 'from' and 'to' robot names in the command line arguments...", "red")
		sys.exit()
    
	# Split the config path into its components
	configvar = options.part
	components = configvar.strip('/').split("/")

	# Load the source yaml file
	sfd = file(fromS, 'r')
	sData = yaml.load(sfd)

	# Get the required part of the source file
	sTagData = sData
	for comp in components:
		if (not type(sTagData) is dict) or (not comp in sTagData):
			print colored('Config parameter path does not exist in the source file!', 'red')
			sys.exit()
		sTagData = sTagData[comp]

	# Load the destination yaml file
	dfd = file(toS, 'r')
	dData = yaml.load(dfd)

	# Get the required part of the destination file
	dTagData = dData
	for comp in components:
		if (not type(dTagData) is dict) or (not comp in dTagData):
			print colored('Config parameter path does not exist in the destination file!', 'red')
			sys.exit()
		dTagData = dTagData[comp]

	# Write the source value into the destination data
	writePart(dData, components, sTagData)

	# Save the output
	ofile = file(toS, 'w')
	yaml.dump(dData, ofile, default_flow_style=False, width=5000)
	print colored("'" + options.part + "' configs copied from " + options.fromR + " to " + options.toR, "green")

# Run the main
if __name__ == '__main__':
	main()
# EOF