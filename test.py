#!/usr/bin/env python
import random
import time
import math
from collections import defaultdict

from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject
from panda3d.core import Vec3, Vec4, Point3
from panda3d.core import AmbientLight, Spotlight
from panda3d.core import Texture, TextureStage
from panda3d.core import TransformState
from panda3d.core import GeoMipTerrain
from panda3d.core import PNMImage, Filename
from panda3d.core import BitMask32
from pandac.PandaModules import ClockObject
from pandac.PandaModules import WindowProperties
from panda3d.bullet import BulletWorld
from panda3d.bullet import (
        BulletPlaneShape, BulletCylinderShape,
        BulletBoxShape, BulletHeightfieldShape)
from panda3d.bullet import BulletRigidBodyNode, BulletDebugNode
from panda3d.bullet import BulletVehicle
from panda3d.bullet import XUp, YUp, ZUp


class Vehicle(object):
    def __init__(self, position, render, world):
        # Chassis uses a simple box shape
        # Note that these are half-extents:
        shape = BulletBoxShape(Vec3(0.6, 1.4, 0.5))
        ts = TransformState.makePos(Point3(0, 0, 0.5))

        self.rigidNode = BulletRigidBodyNode("vehicle")
        self.rigidNode.addShape(shape, ts)
        self.rigidNode.setMass(800.0)
        self.rigidNode.setDeactivationEnabled(False)

        self.np = render.attachNewNode(self.rigidNode)
        self.np.setPos(position)
        world.attachRigidBody(self.rigidNode)

        # Vehicle
        self.vehicle = BulletVehicle(world, self.rigidNode)
        self.vehicle.setCoordinateSystem(ZUp)
        world.attachVehicle(self.vehicle)

        self.yugoNP = loader.loadModel("models/yugo/yugo.egg")
        self.yugoNP.reparentTo(self.np)

        # Create wheels
        for fb, y in (("F", 1.05), ("B", -1.05)):
            for side, x in (("R", 0.7), ("L", -0.7)):
                np = loader.loadModel("models/yugo/yugotire%s.egg" % side)
                np.reparentTo(render)
                isFront = fb == "F"
                self.addWheel(Point3(x, y, 0.3), isFront, np)

    def addWheel(self, position, isFront, np):
        wheel = self.vehicle.createWheel()

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(position)
        wheel.setFrontWheel(isFront)

        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(0.25)
        wheel.setMaxSuspensionTravelCm(40.0)

        wheel.setSuspensionStiffness(40.0)
        wheel.setWheelsDampingRelaxation(2.3)
        wheel.setWheelsDampingCompression(4.4)
        wheel.setFrictionSlip(100.0)
        wheel.setRollInfluence(0.1)


class PlayerControl(DirectObject):
    """
    Controls a vehicle from input
    """

    def __init__(self, vehicle, camera, controlSet):
        self.vehicle = vehicle
        self.camera = camera
        self.inputState = defaultdict(bool)
        
        controls = {
                    "keyboard": {
                        "forward": "arrow_up",
                        "brake": "arrow_down",
                        "left": "arrow_left",
                        "right": "arrow_right",
                        }
                    }
        #added for multiplayer
        if controlSet == 2:
            controls = {
                    "keyboard": {
                        "forward": "w",
                        "brake": "s",
                        "left": "a",
                        "right": "d",
                        }
                    }

        for action, bind in controls["keyboard"].iteritems():
            self.accept(bind, self.inputState.update, [{action: True}])
            self.accept(bind + "-up", self.inputState.update, [{action: False}])

        # Initialise steering
        self.steering = 0.0  # in degrees
        self.steeringLock = 45.0  # in degrees
        self.steeringIncrement = 50.0  # in degrees/second
        self.centeringRate = 50.0  # in degrees/second
        self.reversing = False

        # Initialise camera
        self.updateCamera(initial=True)
        self.camera.node().getLens().setFov(60)

    def updatePlayer(self, dt):
        velocity = self.vehicle.rigidNode.getLinearVelocity()
        speed = math.sqrt(sum(v ** 2 for v in velocity))
        self.processInput(dt, speed=speed)
        self.updateCamera(speed=speed)

    def processInput(self, dt, speed=0.0):
        """Control the players car"""

        engineForce = 0.0
        brakeForce = 0.0

        if self.inputState["forward"]:
            engineForce = 2000.0
        if self.inputState["brake"]:
            if speed < 0.5 or self.reversing:
                # If we're stopped, then start reversing
                engineForce = -1000.0
                self.reversing = True
            else:
                brakeForce = 100.0
        else:
            self.reversing = False

        if self.inputState["left"]:
            self.steering += dt * self.steeringIncrement
            self.steering = min(self.steering, self.steeringLock)
        if self.inputState["right"]:
            self.steering -= dt * self.steeringIncrement
            self.steering = max(self.steering, -self.steeringLock)
        elif not(self.inputState["left"]):
            # gradually re-center the steering
            if self.steering > 0.0:
                self.steering -= dt * self.centeringRate
                if self.steering < 0.0:
                    self.steering = 0.0
            elif self.steering < 0.0:
                self.steering += dt * self.centeringRate
                if self.steering > 0.0:
                    self.steering = 0.0

        vehicle = self.vehicle.vehicle
        # Apply steering to front wheels
        vehicle.setSteeringValue(self.steering, 0);
        vehicle.setSteeringValue(self.steering, 1);

        # Apply engine and brake to rear wheels
        vehicle.applyEngineForce(engineForce, 2);
        vehicle.applyEngineForce(engineForce, 3);
        vehicle.setBrake(brakeForce, 2);
        vehicle.setBrake(brakeForce, 3);

    def updateCamera(self, speed=0.0, initial=False):
        """Reposition camera depending on the vehicle speed"""

        minDistance = 8.0
        maxDistance = 13.0
        minHeight = 1.5
        maxHeight = 3.0
        maxSpeed = 30.0  # m/s

        if initial:
            distance = minDistance
            height = minHeight
        else:
            distance = (minDistance +
                    (maxDistance - minDistance) * speed / maxSpeed)
            distance = min(maxDistance, distance)
            height = minHeight + (maxHeight - minHeight) * speed / maxSpeed
            height = min(maxHeight, height)

        vPos = self.vehicle.np.getPos()
        headingRad = self.vehicle.np.getH() * math.pi / 180.0

        targetPos = vPos + Vec3(
                distance * math.sin(headingRad),
                -distance * math.cos(headingRad),
                height)
        cameraPos = self.camera.getPos()

        if initial:
            self.camera.setPos(targetPos)
        else:
            self.camera.setPos(cameraPos + (targetPos - cameraPos) * 0.1)
        # Look slightly ahead of the car
        self.camera.lookAt(*vPos)
        self.camera.setP(self.camera.getP() + 7)


class BulletApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.windowEventSetup()

        self.setFrameRateMeter(True)
        self.globalClock = ClockObject.getGlobalClock()
        self.globalClock.setMode(ClockObject.MLimited)
        self.globalClock.setFrameRate(60)
        self.disableMouse()

        self.createLights()
        self.render.setShaderAuto()

        self.initialisePhysics(debug=False)

        self.createGround()

        # create a pyramid of barrels
        width = 0.8
        height = 1.0
        numRows = 5
        for row in range(numRows):
            for column in range(numRows - row):
                rowStart = 0.5 * width * (row - numRows + 1)
                position = [
                        rowStart + column * width,
                        0,
                        row * height]
                self.createBarrel(position, height)

        self.taskMgr.add(self.gameLoop, 'update')
        
        #disable default cam so that we can have multiplayer
        base.camNode.setActive(False)
        
        #moved player setup out to its own method
        self.setupPlayers()
    
    
    
    def setupPlayers(self):
        """
        Method to set up player, camera and skybox
        Sets up two player splitscreen
        TODO: Needs reorganising, removal of hardcoding
        """
        #set up first player camera
        self.camera1=self.createCamera((0.,1,0.5,1),(0,-8,3)) 
        self.vehicle = Vehicle((0.0, -15.0, 0.5), self.render, self.world)
        self.controller = PlayerControl(self.vehicle, self.camera1, 1)
        self.createSkybox(self.camera1,1)
        
        self.camera2=self.createCamera((0.,1,0,0.5),(0,-8,3))  
        self.vehicle2 = Vehicle((0.0, -30., 0.5), self.render, self.world)
        self.controller2 = PlayerControl(self.vehicle2, self.camera2, 2)
        self.createSkybox(self.camera2,2)




    def windowEventSetup( self ): 
        """
        Method to bind window events (resizing etc) to windowEventHandler method
        """
        self.accept('window-event', self.windowEventHandler) 

    def windowEventHandler( self, window=None ): 
        """ 
        Called when the panda window is modified to update FOV and aspect ratio
        TODO fix hardcoding for camera names
        """ 
        wp = window.getProperties() 
        windowWidth = wp.getXSize() 
        windowHeight = wp.getYSize() 
        self.camera1.node().getLens().setAspectRatio(windowWidth/(windowHeight/2)) 
        self.camera1.node().getLens().setFov(60)
        self.camera2.node().getLens().setAspectRatio(windowWidth/(windowHeight/2)) 
        self.camera2.node().getLens().setFov(60)



    def createCamera(self,dispRegion,pos): 
        """
        Method to create a camera. Takes displayRegion and a position tuple.
        """
        camera=base.makeCamera(base.win,displayRegion=dispRegion) 
        windowWidth = base.win.getXSize()
        windowHeight = base.win.getYSize()
        camera.node().getLens().setAspectRatio(windowWidth/(windowHeight/2)) 
        camera.node().getLens().setFov(60)
        camera.setPos(pos)
        return camera



    def initialisePhysics(self, debug=False):
        """
        Create Bullet world for physics objects
        """

        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -9.81))
        if debug:
            self.debugNP = self.render.attachNewNode(BulletDebugNode('Debug'))
            self.debugNP.show()
            self.debugNP.node().showWireframe(True)
            self.debugNP.node().showConstraints(True)
            self.debugNP.node().showBoundingBoxes(False)
            self.debugNP.node().showNormals(True)
            self.world.setDebugNode(self.debugNP.node())

    def createBarrel(self, position, height=1.0):
        """
        Create a barrel with the centre of the base at the given position
        """

        # Get barrel textures
        colour = random.choice(("black", "blue", "green", "red", "yellow"))
        texture = self.loader.loadTexture("models/barrel/diffus_" + colour + ".tga")
        normalMap = self.loader.loadTexture("models/barrel/normal_hard_bumps.tga")
        specularMap = self.loader.loadTexture("models/barrel/specular_rust.tga")
        nmTs = TextureStage('normal')
        nmTs.setMode(TextureStage.MNormal)
        sTs = TextureStage('specular')
        sTs.setMode(TextureStage.MGloss)

        # Create graphical model for barrel
        barrel = self.loader.loadModel("models/barrel/metal_barrel.egg")
        barrel.setTexture(texture)
        barrel.setTexture(nmTs, normalMap)
        barrel.setTexture(sTs, specularMap)
        barrelMin, barrelMax = barrel.getTightBounds()

        # Y is the cylinder axis here
        scale = height  / (barrelMax[2] - barrelMin[2])
        radius = (barrelMax[0] - barrelMin[0]) * scale * 0.5

        # Create a barrel for physics
        barrelShape = BulletCylinderShape(radius, height, ZUp)
        barrelNode = BulletRigidBodyNode('barrel')
        barrelNode.setMass(8.0)
        barrelNode.setFriction(100.0)
        barrelNode.addShape(barrelShape)

        np = self.render.attachNewNode(barrelNode)
        position[2] = position[2] + 0.5 * height
        np.setPos(*position)
        np.setHpr(0, 0, 0)
        self.world.attachRigidBody(barrelNode)

        barrel.setScale(scale, scale, scale)
        barrel.setPos(0.0, 0.0, -0.5)
        barrel.setHpr(0, 0, 0)
        barrel.reparentTo(np)

    def createGround(self):
        """Create ground using a heightmap"""

        # Create heightfield for physics
        maxHeight = 10.0

        # Image needs to have dimensions that are a power of 2 + 1
        heightMap = PNMImage('models/floor/elevation.png')
        xdim = heightMap.getXSize()
        ydim = heightMap.getYSize()
        shape = BulletHeightfieldShape(heightMap, maxHeight, ZUp)
        shape.setUseDiamondSubdivision(True)

        np = self.render.attachNewNode(BulletRigidBodyNode('terrain'))
        np.node().addShape(shape)
        np.setPos(0, 0, 0)
        self.world.attachRigidBody(np.node())

        # Create graphical terrain from same height map
        self.terrain = GeoMipTerrain('terrain')
        self.terrain.setHeightfield(heightMap)

        self.terrain.setBlockSize(32)
        self.terrain.setBruteforce(True)
        rootNP = self.terrain.getRoot()
        rootNP.reparentTo(self.render)
        rootNP.setSz(maxHeight)

        offset = xdim / 2.0 - 0.5
        rootNP.setPos(-offset, -offset, -maxHeight / 2.0)
        self.terrain.generate()

        # Apply texture
        diffuse = self.loader.loadTexture(Filename("models/floor/dirt.png"))
        diffuse.setWrapU(Texture.WMRepeat)
        diffuse.setWrapV(Texture.WMRepeat)
        rootNP.setTexture(diffuse)
        textureSize = 6.0
        ts = TextureStage.getDefault()
        rootNP.setTexScale(ts, xdim / textureSize, ydim / textureSize)

        # Create planes around area to prevent player flying off the edge
        sides = (
                (Vec3(1, 0, 0), -xdim / 2.0),
                (Vec3(-1, 0, 0), -xdim / 2.0),
                (Vec3(0, 1, 0), -ydim / 2.0),
                (Vec3(0, -1, 0), -ydim / 2.0),
                )
        for sideNum, side in enumerate(sides):
            normal, offset = side
            sideShape = BulletPlaneShape(normal, offset)
            sideNode = BulletRigidBodyNode('side%d' % sideNum)
            sideNode.addShape(sideShape)
            self.world.attachRigidBody(sideNode)

    def createSkybox(self, currentCamera, bitmaskCode):
        """
        Create a skybox linked to the current camera setup. 
        Skybox will only be shown to camera with this bitmaskCode
        TODO: remove bitmaskCode or make it more modular to reduce coupling
        """

        sky = self.loader.loadModel("models/sky/cube.egg")
        diffuse = self.loader.loadTexture("models/sky/skymap.png")
        sky.setTexture(diffuse)
        sky.setScale(270)
        # Get it to follow the camera so it feels as if it's infinitely
        # far away, but call setCompass so it rotates with the world
        sky.reparentTo(currentCamera)
        sky.setCompass()
        sky.hide(BitMask32.allOn())
        currentCamera.node().setCameraMask(BitMask32.bit(bitmaskCode))
        sky.show(BitMask32.bit(bitmaskCode))
        

    def createLights(self):
        """Create an ambient light and a spotlight for shadows"""
        self.render.clearLight()

        alight = AmbientLight('ambientLight')
        alight.setColor(Vec4(0.7, 0.7, 0.7, 1))
        alightNP = self.render.attachNewNode(alight)
        self.render.setLight(alightNP)

        # Create a spotlight for shadows
        # Could also use a directional light with the automatic shadows
        spotlight = Spotlight('light')
        spotlight.setColor(Vec4(0.8, 0.8, 0.8, 1))
        spotlight.setShadowCaster(True, 2048, 2048)
        spotlight.getLens().setFov(40)
        spotlight.getLens().setNearFar(2, 50)
        spotlightNP = self.render.attachNewNode(spotlight)
        spotlightNP.setPos(-20, -20, 35)
        spotlightNP.lookAt(0, 0, 0)
        self.render.setLight(spotlightNP)

    def gameLoop(self, task):
        dt = self.globalClock.getDt()
        self.controller.updatePlayer(dt)
        self.controller2.updatePlayer(dt)
        self.world.doPhysics(dt)
        return task.cont


app = BulletApp()
app.run()
