#!/usr/bin/python
# Author: Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

# Imports
import re
import sys
import yaml
from termcolor import colored


# Get a node from a dictionary tree by path (Note: Check the return value by explicit comparison to None)
def getNodeByPath(node, path):

	# Retrieve the required node
	return getNodeByComponents(node, pathToComponents(path))


# Get a node from a dictionary tree by components (Note: Check the return value by explicit comparison to None)
def getNodeByComponents(node, components):

	# Retrieve the required node
	for comp in components:
		if (not type(node) is dict) or (comp not in node):
			return None
		node = node[comp]
	return node


# Add or update the value of a node in a dictionary tree by path
def writeNodeByPath(node, path, value):

	# Add/update the required node
	return writeNodeByComponents(node, pathToComponents(path), value)


# Add or update the value of a node in a dictionary tree by components
def writeNodeByComponents(node, components, value):

	# We cannot write the root node
	if not components:
		return False

	# Add/update the required node
	for i, comp in enumerate(components):
		if not type(node) is dict:
			return False
		if i + 1 == len(components):
			node[comp] = value
		else:
			if comp not in node:
				node[comp] = dict()
			node = node[comp]

	# Return that the write was successful
	return True


# Remove a node from a dictionary tree by path
def removeNodeByPath(node, path, prune = True):

	# Remove the required node
	return removeNodeByComponents(node, pathToComponents(path), prune)


# Remove a node from a dictionary tree by components
def removeNodeByComponents(node, components, prune = True):

	# We cannot remove the root node
	if not components:
		return False

	# Remove the required node
	pruneNode = node
	pruneComp = components[0]
	for i, comp in enumerate(components):
		if (not type(node) is dict) or (not comp in node):
			return False
		if len(node) > 1:
			pruneNode = node
			pruneComp = comp
		if i + 1 == len(components):
			if prune:
				del pruneNode[pruneComp]
			else:
				del node[comp]
		else:
			node = node[comp]

	# Return that the deletion was successful
	return True


# Convert a path to its components
def pathToComponents(path):

	# Convert the path to its components
	return filter(None, path.split('/'))


# Convert components to their equivalent path
def componentsToPath(components):

	# Convert the components to their equivalent path
	return "/".join(components)


# Read in a YAML file
def readYAML(filepath):

	# Open and read in the file as required
	f = file(filepath, 'r')
	data = yaml.load(f)
	f.close()
	return data


# Write YAML data out to a file
def writeYAML(filepath, data):

	# Write the data to file as required
	f = file(filepath, 'w')
	yaml.dump(data, f, default_flow_style = False, width = 5000)
	f.close()

	# Clean up after the YAML dump function
	cleanYAMLDump(filepath)


# Clean up a YAML file that was produced by the YAML dump function
def cleanYAMLDump(filepath):

	# Read in the yaml file in raw form
	f = open(filepath, 'r')
	contents = f.read()
	f.close()

	# Apply regular expressions to do some manual cleaning of the yaml file
	contents = re.compile(r"^(.*\S) +$", re.MULTILINE).sub(r"\1", contents)
	contents = re.compile(r"^( *[^:]+:) +'(.*)' *$", re.MULTILINE).sub(r'\1 "\2"', contents)
	contents = re.compile(r"^( *[^:]+:) +(-?[0-9]+\.[0-9]*[1-9])0+ *$", re.MULTILINE).sub(r"\1 \2", contents)
	contents = re.compile(r"^( *[^:]+:) +(-?[0-9]+)\.0+ *$", re.MULTILINE).sub(r"\1 \2", contents)

	# Write the cleaned output back to the yaml file
	f = open(filepath, 'w')
	f.write(contents)
	f.close()


# Convert a value to string
def toString(value):

	# Convert the value to string as required
	return "'" + value + "'" if type(value) is str else str(value)


# Error function
def error(message, exitAfter = False):

	# Print the required error message and exit
	print colored("Error: " + message, "red")
	if exitAfter:
		sys.exit()


# Warning function
def warning(message):

	# Print the required warning message
	print colored("Warning: " + message, "yellow")

# EOF