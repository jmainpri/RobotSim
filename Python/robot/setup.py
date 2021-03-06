#!/usr/bin/env python

from distutils.core import setup,Extension
import distutils.util
import os
import glob

#these probably do not need to be changed
robotSimDir = '../..'
krisLibraryDir = '../../Library/KrisLibrary'
odedir = '../../Library/ode-0.11.1'
gluidir = '../../Library/glui-2.36'
tinyxmlDir = '../../Library/tinyxml'
#optional
assimpDir='Library/assimp--3.0.1270-sdk'

includeDirs = [robotSimDir,krisLibraryDir,tinyxmlDir,odedir+'/include','/usr/include','.']

tinyxmlLibDir = tinyxmlDir
odelibdir = odedir+'/ode/src/.libs'
gluilibdir = gluidir+'/src/lib'
assimpLibDir = assimpDir+'/lib'

on_cygwin = distutils.util.get_platform().startswith('cygwin')
on_windows = distutils.util.get_platform().startswith('win')
win32debug = False
print distutils.util.get_platform()
if on_windows:
	if win32debug:
		libdirs = [robotSimDir+'/msvc/Debug',robotSimDir+'/Library']
	else:
		libdirs = [robotSimDir+'/msvc/Release',robotSimDir+'/Library']
else:
	libdirs = [robotSimDir+'/lib',krisLibraryDir+'/lib',tinyxmlLibDir,odelibdir,gluilibdir,assimpLibDir,'/usr/lib']

commonfiles = ['pyerr.cpp']

rssourcefiles = commonfiles + ['robotsim.cpp','robotik.cpp','robotsim_wrap.cxx']
mpsourcefiles = commonfiles + ['motionplanning.cpp','motionplanning_wrap.cxx']
cosourcefiles = commonfiles + ['collide.cpp','collide_wrap.cxx']
rfsourcefiles = commonfiles + ['rootfind.cpp','pyvectorfield.cpp','rootfind_wrap.cxx']

#needed for KrisLibrary to link
kllibs = ['KrisLibrary','tinyxml','glpk','glui','GL']
if on_cygwin:
    kllibs[-1] = 'opengl32'
    kllibs.append('glut32')
if on_windows:
	if win32debug:
		kllibs = ['KrisLibraryd','tinyxmld_STL','glui32d','glut32','opengl32','winmm','user32']
	else:
		kllibs = ['KrisLibrary','tinyxml_STL','glui32','glut32','opengl32','winmm','user32']

#needed for RobotSim to link
libs = ['RobotSim']+kllibs+['ode']
#switch to this if assimp support is desired
#libs = ['RobotSim']+kllibs+['ode','assimp']
if on_windows:
	if win32debug:
		libs[-1]='ode_doubled'
	else:
		libs[-1]='ode_double'
		
defines = [('TIXML_USE_STL',None),('dDOUBLE',None)]
if on_windows:
	defines += [('WIN32',None)]

setup(name='RobotSim',
      version='0.2',
      description='RobotSim extension module',
      author='Kris Hauser',
      author_email='hauserk@indiana.edu',
      url='https://github.com/krishauser/RobotSim',
      ext_modules=[Extension('robot._robotsim',
                             [os.path.join('src',f) for f in rssourcefiles],
                             include_dirs=includeDirs,
                             define_macros=defines,
                             library_dirs=libdirs,
                             libraries=libs,
                             language='c++'),
                   Extension('robot._motionplanning',
                             [os.path.join('src',f) for f in mpsourcefiles],
                             include_dirs=includeDirs,
                             define_macros=defines,
							 library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++'),
                   Extension('robot._collide',
                             [os.path.join('src',f) for f in cosourcefiles],
                             include_dirs=includeDirs,
							 define_macros=defines,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++'),
                   Extension('robot._rootfind',
                             [os.path.join('src',f) for f in rfsourcefiles],
                             include_dirs=includeDirs,
							 define_macros=defines,
                             library_dirs=libdirs,
                             libraries=kllibs,
                             language='c++')],
	  packages=['robot'],
	  package_dir={'robot':'.'}
     )

