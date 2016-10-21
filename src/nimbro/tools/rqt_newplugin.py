import os
import zipfile
import sys
import fileinput

numargs = len(sys.argv)
method = 0

if(numargs<2 or numargs > 4):
	print('Not enough or too many arguments.')
	print('Usage: '+sys.argv[0]+' [Package Name] (Module Name) (Plugin Name)')
	sys.exit()
elif(numargs>2):
	method = 2
	modulename = sys.argv[2]
	pluginname = sys.argv[3]
else:
	method = 1

path = './'
#origin = os.getcwd()

defaultzip = zipfile.ZipFile('./defaultPlugin.zip', 'r')

packagename = sys.argv[1]
os.chdir(path)
os.system('catkin_create_pkg '+packagename+' rospy rqt_gui rqt_gui_py') # Probably useless if there is not record of created plugins taken
os.system('rm '+packagename+' -rf')
os.system('mkdir '+packagename)

srcdir = path+'/'+packagename+'/'
defaultzip.extractall(srcdir)

print('Extracted dummy Plugin.');

for line in fileinput.input(srcdir+'plugin.xml', inplace = 1):
	if 'samplePackage' in line:
		line = line.replace('samplePackage', packagename)
	if method == 2 and 'sampleModule' in line:
		line = line.replace('sampleModule', modulename)
	if method == 2 and 'SamplePlugin' in line:
		line = line.replace('SamplePlugin', pluginname)
	sys.stdout.write(line)

print('plugin.xml updated.')

for line in fileinput.input(srcdir+'CMakeLists.txt', inplace = 1):
	if 'samplePackage' in line:
		line = line.replace('samplePackage', packagename)
	if method == 2 and 'sampleModule' in line:
		line = line.replace('sampleModule', modulename)
	if method == 2 and 'SamplePlugin' in line:
		line = line.replace('SamplePlugin', pluginname)
	sys.stdout.write(line)

print('CMakeLists.txt updated.')

for line in fileinput.input(srcdir+'package.xml', inplace = 1):
	if 'samplePackage' in line:
		line = line.replace('samplePackage', packagename)
	if method == 2 and 'sampleModule' in line:
		line = line.replace('msampleModule', modulename)
	if method == 2 and 'SamplePlugin' in line:
		line = line.replace('SamplePlugin', pluginname)
	sys.stdout.write(line)

print('package.xml updated.')
os.rename(srcdir+'src/samplePackage', srcdir+'src/'+packagename)
print('Renamed package directory.')
os.rename(srcdir+'src/'+packagename+'/sampleModule.py',srcdir+'src/'+packagename+'/'+modulename+'.py')
print('Renamed module script.')

for line in fileinput.input(srcdir+'src/'+packagename+'/'+modulename+'.py', inplace = 1):
	if 'samplePackage' in line:
		line = line.replace('samplePackage', packagename)
	if method == 2 and 'sampleModule' in line:
		line = line.replace('sampleModule', modulename)
	if method == 2 and 'SamplePlugin' in line:
		line = line.replace('SamplePlugin', pluginname)
	sys.stdout.write(line)

print('--Default plugin created.--')
defaultzip.close()
