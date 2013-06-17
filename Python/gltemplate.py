#!/usr/bin/python

import sys
from robot import *
from robot.glprogram import *

class MyGLViewer(GLRealtimeProgram):
    def __init__(self,world):
        GLRealtimeProgram.__init__(self,"My GL program")
        self.world = world
        #Put your initialization code here
        #the current example creates a collision class, simulator, 
        #simulation flag, and screenshot flags
        self.collider = robotcollide.WorldCollider(world)
        self.sim = Simulator(world)
        self.simulate = False

        self.saveScreenshots = False
        self.nextScreenshotTime = 0
        self.screenshotCount = 0

    def display(self):
        #Put your display handler here
        #the current example draws the simulated world
        self.sim.updateWorld()
        self.world.drawGL()

    def control_loop(self):
        #Put your control handler here
        pass

    def idle(self):
        #Put your idle loop handler here
        #the current example simulates with the current time step self.dt
        if self.simulate and self.saveScreenshots:
            if self.ttotal >= self.nextScreenshotTime:
                self.save_screen("image%04d.ppm"%(self.screenshotCount,))
            self.screenshotCount += 1
            self.nextScreenshotTime += 1.0/30.0;

        if self.simulate:
            self.control_loop()
            self.sim.simulate(self.dt)
            glutPostRedisplay()

    def mousefunc(self,button,state,x,y):
        #Put your mouse handler here
        #the current example prints out the list of objects clicked whenever
        #you right click
        print "mouse",button,state,x,y
        if button==2 and state==0:
            print [o.getName() for o in self.click_world(x,y)]
            return
        GLRealtimeProgram.mousefunc(self,button,state,x,y)

    def specialfunc(self,c,x,y):
        #Put your keyboard special character handler here
        print c,"pressed"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        print c,"pressed"
        if c == 's':
            self.simulate = not self.simulate
            print "Simulating:",self.simulate
        elif c == 'm':
            self.saveScreenshots = not self.saveScreenshots
            print "Movie mode:",self.saveScreenshots
        glutPostRedisplay()

    def click_world(self,x,y):
        """Helper: returns a list of world objects sorted in order of
        increasing distance."""
        #get the viewport ray
        (s,d) = self.click_ray(x,y)
        
        #run the collision tests
        self.collider.updateFrames()
        collided = []
        for g in self.collider.geomList:
            (hit,pt) = collide.rayCast(g[1],s,d)
            if hit:
                dist = vectorops.dot(vectorops.sub(pt,s),d)
                collided.append((dist,g[0]))
        return [g[1] for g in sorted(collided)]


if __name__ == "__main__":
    print "gltemplate.py: This example demonstrates how to simulate a world and read user input"
    if len(sys.argv)<=1:
        print "USAGE: gltemplate.py [world_file]"
        exit()
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    viewer.run()
