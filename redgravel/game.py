import random
import time
import math
from collections import defaultdict
import json
import sys

from direct.showbase.DirectObject import DirectObject
from direct.showbase import Audio3DManager
from direct.filter.CommonFilters import CommonFilters
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
    def __init__(self, position, render, world, base):
        self.base = base
        loader = base.loader

        vehicleType = "yugo"
        self.vehicleDir = "data/vehicles/" + vehicleType + "/"
        # Load vehicle description and specs
        with open(self.vehicleDir + "vehicle.json") as vehicleData:
            data = json.load(vehicleData)
            self.specs = data["specs"]

        # Chassis for collisions and mass uses a simple box shape
        halfExtents = (0.5 * dim for dim in self.specs["dimensions"])
        shape = BulletBoxShape(Vec3(*halfExtents))
        ts = TransformState.makePos(Point3(0, 0, 0.5))

        self.rigidNode = BulletRigidBodyNode("vehicle")
        self.rigidNode.addShape(shape, ts)
        self.rigidNode.setMass(self.specs["mass"])
        self.rigidNode.setDeactivationEnabled(False)

        self.np = render.attachNewNode(self.rigidNode)
        self.np.setPos(position)
        world.attachRigidBody(self.rigidNode)

        # Vehicle physics model
        self.vehicle = BulletVehicle(world, self.rigidNode)
        self.vehicle.setCoordinateSystem(ZUp)
        world.attachVehicle(self.vehicle)

        # Vehicle graphical model
        self.vehicleNP = loader.loadModel(self.vehicleDir + "car.egg")
        self.vehicleNP.reparentTo(self.np)

        # Create wheels
        wheelPos = self.specs["wheelPositions"]
        for fb, y in (("F", wheelPos[1]), ("B", -wheelPos[1])):
            for side, x in (("R", wheelPos[0]), ("L", -wheelPos[0])):
                np = loader.loadModel(self.vehicleDir + "tire%s.egg" % side)
                np.reparentTo(render)
                isFront = fb == "F"
                self.addWheel(Point3(x, y, wheelPos[2]), isFront, np)

    def addWheel(self, position, isFront, np):
        wheel = self.vehicle.createWheel()
        wheelSpecs = self.specs["wheels"]

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(position)
        wheel.setFrontWheel(isFront)

        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(wheelSpecs["radius"])
        wheel.setMaxSuspensionTravelCm(wheelSpecs["suspensionTravel"] * 100.0)
        wheel.setSuspensionStiffness(wheelSpecs["suspensionStiffness"])
        wheel.setWheelsDampingRelaxation(wheelSpecs["dampingRelaxation"])
        wheel.setWheelsDampingCompression(wheelSpecs["dampingCompression"])
        wheel.setFrictionSlip(wheelSpecs["frictionSlip"])
        wheel.setRollInfluence(wheelSpecs["rollInfluence"])

    def initialiseSound(self, audioManager):
        """Start the engine sound"""

        self.engineSound = audioManager.loadSfx(self.vehicleDir + "engine.wav")
        audioManager.attachSoundToObject(self.engineSound, self.np)
        self.engineSound.setLoop(True)
        self.engineSound.setPlayRate(0.6)
        self.engineSound.play()

        self.gearSpacing = (self.specs["sound"]["maxExpectedRotationRate"] /
                self.specs["sound"]["numberOfGears"])
        self.reversing = False

    def updateSound(self, dt):
        """Use vehicle speed to update sound pitch"""

        soundSpecs = self.specs["sound"]
        # Use rear wheel rotation speed as some measure of engine revs
        wheels = (self.vehicle.getWheel(idx) for idx in (2, 3))
        # wheelRate is in degrees per second
        wheelRate = 0.5 * abs(sum(w.getDeltaRotation() / dt for w in wheels))

        # Calculate which gear we're in, and what the normalised revs are
        if self.reversing:
            numberOfGears = 1
        else:
            numberOfGears = self.specs["sound"]["numberOfGears"]
        gear = min(int(wheelRate / self.gearSpacing),
                numberOfGears - 1)
        posInGear = (wheelRate - gear * self.gearSpacing) / self.gearSpacing

        targetPlayRate = 0.6 + posInGear * (1.5 - 0.6)
        currentRate = self.engineSound.getPlayRate()
        self.engineSound.setPlayRate(0.8 * currentRate + 0.2 * targetPlayRate)

    def updateControl(self, controlState, dt):
        """Updates acceleration, braking and steering

        These are all passed in through a controlState dictionary
        """

        # Update acceleration and braking
        wheelForce = controlState["throttle"] * self.specs["maxWheelForce"]
        self.reversing = controlState["reverse"]
        if self.reversing:
            # Make reversing a bit slower than moving forward
            wheelForce *= -0.5

        brakeForce = controlState["brake"] * self.specs["brakeForce"]

        # Update steering
        # Steering control state is from -1 to 1
        steering = controlState["steering"] * self.specs["steeringLock"]

        # Apply steering to front wheels
        self.vehicle.setSteeringValue(steering, 0);
        self.vehicle.setSteeringValue(steering, 1);
        # Apply engine and brake to rear wheels
        self.vehicle.applyEngineForce(wheelForce, 2);
        self.vehicle.applyEngineForce(wheelForce, 3);
        self.vehicle.setBrake(brakeForce, 2);
        self.vehicle.setBrake(brakeForce, 3);


class PlayerControl(DirectObject):
    """
    Controls a vehicle from input
    """

    def __init__(self, vehicle, camera, controlSet):
        self.vehicle = vehicle
        self.camera = camera
        self.inputState = defaultdict(bool)
        self.vehicleControlState = {}
        self.vehicleControlState["reverse"] = False
        self.vehicleControlState["steering"] = 0.0

        controls = {
                    "keyboard": {
                        "forward": "arrow_up",
                        "brake": "arrow_down",
                        "left": "arrow_left",
                        "right": "arrow_right",
                        }
                    }
        #added for multiplayer
        if controlSet == 1:
            controls = {
                    "keyboard": {
                        "forward": "w",
                        "brake": "s",
                        "left": "a",
                        "right": "d",
                        }
                    }
        elif controlSet == 2:
            controls = {
                    "keyboard": {
                        "forward": "t",
                        "brake": "g",
                        "left": "f",
                        "right": "h",
                        }
                    }
        elif controlSet == 3:
            controls = {
                    "keyboard": {
                        "forward": "o",
                        "brake": "l",
                        "left": "k",
                        "right": ";",
                        }
                    }

        for action, bind in controls["keyboard"].iteritems():
            self.accept(bind, self.inputState.update, [{action: True}])
            self.accept(bind + "-up", self.inputState.update, [{action: False}])

        # Initialise camera
        self.updateCamera(initial=True)
        self.camera.node().getLens().setFov(60)

        # Steering change per second, normalised to steering lock
        # Eg. 45 degrees lock and 1.0 rate means 45 degrees per second
        self.steeringRate = 0.8
        self.centreingRate = 1.2

    def updatePlayer(self, dt):
        velocity = self.vehicle.rigidNode.getLinearVelocity()
        speed = math.sqrt(sum(v ** 2 for v in velocity))
        self.processInput(dt, speed=speed)
        self.updateCamera(speed=speed)
        self.vehicle.updateSound(dt)

    def processInput(self, dt, speed=0.0):
        """Use controls to update the player's car"""

        # For keyboard throttle and brake are either 0 or 1
        if self.inputState["forward"]:
            self.vehicleControlState["throttle"] = 1.0
        else:
            self.vehicleControlState["throttle"] = 0.0

        # Update braking and reversing
        if self.inputState["brake"]:
            if speed < 0.5 or self.vehicleControlState["reverse"]:
                # If we're stopped, then start reversing
                # Also keep reversing if we already were
                self.vehicleControlState["reverse"] = True
                self.vehicleControlState["throttle"] = 1.0
                self.vehicleControlState["brake"] = 0.0
            else:
                self.vehicleControlState["reverse"] = False
                self.vehicleControlState["brake"] = 1.0
        else:
            self.vehicleControlState["reverse"] = False
            self.vehicleControlState["brake"] = 0.0

        # steering is normalised from -1 to 1, corresponding
        # to the steering lock right and left
        steering = self.vehicleControlState["steering"]
        if self.inputState["left"]:
            steering += dt * self.steeringRate
            steering = min(steering, 1.0)
        elif self.inputState["right"]:
            steering -= dt * self.steeringRate
            steering = max(steering, -1.0)
        else:
            # gradually re-center the steering
            if steering > 0.0:
                steering -= dt * self.centreingRate
                if steering < 0.0:
                    steering = 0.0
            elif steering < 0.0:
                steering += dt * self.centreingRate
                if steering > 0.0:
                    steering = 0.0
        self.vehicleControlState["steering"] = steering

        self.vehicle.updateControl(self.vehicleControlState, dt)

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


class World(object):
    def __init__(self, base, numberOfPlayers):
        self.base = base
        self.render = base.render
        self.taskMgr = base.taskMgr
        self.loader = base.loader

        self.windowEventSetup()

        # Create separate nodes so that shaders apply to
        # stuff in the worldRender but not the outsideWorldRender
        self.worldRender = render.attachNewNode('world')
        self.outsideWorldRender = render.attachNewNode('world')

        self.base.setFrameRateMeter(True)
        self.globalClock = ClockObject.getGlobalClock()
        self.globalClock.setMode(ClockObject.MLimited)
        self.globalClock.setFrameRate(60)
        self.base.disableMouse()

        if not 0 < numberOfPlayers < 5:
            raise ValueError("Number of players must be from 1 to 4")
        self.numberOfPlayers = numberOfPlayers

        self.createLights()

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

        #disable default cam so that we can have multiplayer
        base.camNode.setActive(False)

        #moved player setup out to its own method
        self.setupPlayers(self.numberOfPlayers)

        # Set up shaders for shadows and cartoon outline
        self.setupShaders()

        # Create an audio manager. This is attached to the camera for
        # player 1, so sounds close to other players might not be very
        # loud
        self.audioManager = Audio3DManager.Audio3DManager(
                self.base.sfxManagerList[0], self.playerCameras[0])
        # Distance should be in m, not feet
        self.audioManager.setDistanceFactor(3.28084)

        # Now initialise audio for all vehicles that were
        # previously set up. We can't just create the audio manager
        # first because we need the player 1 camera
        for vehicle in self.playerVehicles:
            vehicle.initialiseSound(self.audioManager)

    def run(self):
        """
        Start running the game loop by adding it
        to the task manager
        """

        self.taskMgr.add(self.gameLoop, 'update')

    def setupPlayers(self, numberOfPlayers):
        """
        Method to set up player, camera and skybox
        Sets up two player splitscreen
        TODO: Needs reorganising, removal of hardcoding
        """
        self.playerCameras = []
        self.playerVehicles = []
        self.playerControllers = []
        self.playerInputKeys = range(numberOfPlayers)
        #default camera position bound to each car
        cameraPosition = (0,-8,3)
        #default vehicle position
        vehiclePosition = (0.0, -15.0, 0.5)
        
        #single player display setup - default
        displayXStart = 0.0
        displayXEnd = 1.0
        displayYStart = 0.0
        displayYEnd = 1.0
        
        
        #I am not proud of myself for this... There has to be a better way using maths
        # But it's nearly 2am and I'll figure it out tomorrow
        for i in xrange(numberOfPlayers):            
            if numberOfPlayers == 2:
                if i == 0:#player 1, indexes from 0
                    displayXStart = 0.0#don't need this but it's safer to have it'
                    displayXEnd = 1.0
                    displayYStart = 0.5
                    displayYEnd = 1.0
                else:
                    displayXStart = 0.0#don't need this but it's safer to have it'
                    displayXEnd = 1.0
                    displayYStart = 0.0
                    displayYEnd = 0.5
                    vehiclePosition = (0.0, -25.0, 0.5)
            elif numberOfPlayers == 3:
                if i == 0: #top of screen
                    displayXStart = 0.0#don't need this but it's safer to have it'
                    displayXEnd = 1.0
                    displayYStart = 0.5
                    displayYEnd = 1.0
                elif i == 1: #p2, bottom left
                    displayXStart = 0.0 
                    displayXEnd = 0.5
                    displayYStart = 0.0
                    displayYEnd = 0.5
                    vehiclePosition = (0.0, -25.0, 0.5)
                else: #bottom right
                    displayXStart = 0.5
                    displayXEnd = 1.0
                    displayYStart = 0.0
                    displayYEnd = 0.5
                    vehiclePosition = (0.0, 15.0, 0.5)
            elif numberOfPlayers == 4:
                if i == 0: #p1, top left
                    displayXStart = 0.0#don't need this but it's safer to have it'
                    displayXEnd = 0.5
                    displayYStart = 0.5
                    displayYEnd = 1.0
                elif i == 1: #p2, top right
                    displayXStart = 0.5 
                    displayXEnd = 1.0
                    displayYStart = 0.5
                    displayYEnd = 1.0
                    vehiclePosition = (0.0, -25.0, 0.5)
                elif i == 2: #p3, bottom left
                    displayXStart = 0.0
                    displayXEnd = 0.5
                    displayYStart = 0.0
                    displayYEnd = 0.5
                    vehiclePosition = (0.0, 15.0, 0.5)
                else: #else p4, bottom right
                    displayXStart = 0.5
                    displayXEnd = 1.0
                    displayYStart = 0.0
                    displayYEnd = 0.5
                    vehiclePosition = (0.0, 25.0, 0.5)
                
            #setup display area for this player's camera
            displayBox = (displayXStart, displayXEnd, displayYStart, displayYEnd)
            #set up camera with display area above and default camera position
            camera = self.createCamera(displayBox, cameraPosition)
            self.playerCameras.append(camera)
            #set up player car
            vehicle = Vehicle(vehiclePosition, self.worldRender, self.world, self.base)
            self.playerVehicles.append(vehicle)
            #set up player controller with car, camera and keyset
            playerController = PlayerControl(self.playerVehicles[i], self.playerCameras[i], self.playerInputKeys[i])
            self.playerControllers.append(playerController)
            #set up skybox for this particular camera set and hide from other cameras
            self.createSkybox(self.playerCameras[i], self.playerInputKeys[i]) #can use inputkey code for bitmaskCode as it will be unique

    def windowEventSetup( self ):
        """
        Method to bind window events (resizing etc) to windowEventHandler method
        """
        self.base.accept('window-event', self.windowEventHandler)

    def windowEventHandler(self, window = None): 
        """ 
        Called when the panda window is modified to update FOV and aspect ratio
        TODO fix hardcoding for camera names
        """ 
        if window.isClosed():
            sys.exit()
        
        wp = window.getProperties() 
        windowWidth = wp.getXSize() 
        windowHeight = wp.getYSize() 
        for i in self.playerCameras:  #added to allow for changing player number
            #since window size has changed we need to update the aspect ratio and FOV
            i.node().getLens().setAspectRatio(windowWidth/(windowHeight/2)) 
            i.node().getLens().setFov(60)


    def createCamera(self,dispRegion,pos): 
        """
        Method to create a camera. Takes displayRegion and a position tuple.
        Sets aspect ratio based on window properties and also sets FOV
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
            self.debugNP = self.outsideWorldRender.attachNewNode(BulletDebugNode('Debug'))
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
        texture = self.loader.loadTexture("data/models/barrel/diffus_" + colour + ".tga")
        normalMap = self.loader.loadTexture("data/models/barrel/normal_hard_bumps.tga")
        specularMap = self.loader.loadTexture("data/models/barrel/specular_rust.tga")
        nmTs = TextureStage('normal')
        nmTs.setMode(TextureStage.MNormal)
        sTs = TextureStage('specular')
        sTs.setMode(TextureStage.MGloss)

        # Create graphical model for barrel
        barrel = self.loader.loadModel("data/models/barrel/metal_barrel.egg")
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

        np = self.worldRender.attachNewNode(barrelNode)
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
        heightMap = PNMImage('data/models/floor/elevation.png')
        xdim = heightMap.getXSize()
        ydim = heightMap.getYSize()
        shape = BulletHeightfieldShape(heightMap, maxHeight, ZUp)
        shape.setUseDiamondSubdivision(True)

        np = self.outsideWorldRender.attachNewNode(
            BulletRigidBodyNode('terrain'))
        np.node().addShape(shape)
        np.setPos(0, 0, 0)
        self.world.attachRigidBody(np.node())

        # Create graphical terrain from same height map
        self.terrain = GeoMipTerrain('terrain')
        self.terrain.setHeightfield(heightMap)

        self.terrain.setBlockSize(32)
        self.terrain.setBruteforce(True)
        rootNP = self.terrain.getRoot()
        rootNP.reparentTo(self.worldRender)
        rootNP.setSz(maxHeight)

        offset = xdim / 2.0 - 0.5
        rootNP.setPos(-offset, -offset, -maxHeight / 2.0)
        self.terrain.generate()

        # Apply texture
        diffuse = self.loader.loadTexture(Filename("data/models/floor/dirt.png"))
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

        sky = self.loader.loadModel("data/models/sky/cube.egg")
        diffuse = self.loader.loadTexture("data/models/sky/skymap.png")
        sky.setTexture(diffuse)
        sky.setScale(270)
        # Get it to follow the camera so it feels as if it's infinitely
        # far away, but call setCompass so it rotates with the world
        sky.reparentTo(currentCamera)
        sky.setCompass()
        # hide skybox from other players
        sky.hide(BitMask32.allOn())
        #show only to this player's camera
        currentCamera.node().setCameraMask(BitMask32.bit(bitmaskCode))
        sky.show(BitMask32.bit(bitmaskCode))
        

    def createLights(self):
        """Create an ambient light and a spotlight for shadows"""

        self.render.clearLight()

        alight = AmbientLight('ambientLight')
        alight.setColor(Vec4(0.7, 0.7, 0.7, 1))
        alightNP = self.worldRender.attachNewNode(alight)
        self.worldRender.setLight(alightNP)

        # Create a spotlight for shadows
        # Could also use a directional light with the automatic shadows
        spotlight = Spotlight('light')
        spotlight.setColor(Vec4(0.8, 0.8, 0.8, 1))
        spotlight.setShadowCaster(True, 2048, 2048)
        spotlight.getLens().setFov(40)
        spotlight.getLens().setNearFar(2, 50)
        spotlightNP = self.worldRender.attachNewNode(spotlight)
        spotlightNP.setPos(-20, -20, 35)
        spotlightNP.lookAt(0, 0, 0)
        self.worldRender.setLight(spotlightNP)

    def setupShaders(self):
        """
        Creates shaders for cartoon outline and enables
        shaders for shadows
        """

        if (self.base.win.getGsg().getSupportsBasicShaders() == 0):
            return

        thickness = 1.0
        for camera in self.playerCameras:
            self.filters = CommonFilters(self.base.win, camera)
            filterEnabled = self.filters.setCartoonInk(separation=thickness)
            if (filterEnabled == False):
                # Couldn't enable filter, video card probably
                # doesn't support filter
                return

        self.worldRender.setShaderAuto()

    def gameLoop(self, task):
        dt = self.globalClock.getDt()
        for i in self.playerControllers: #added to allow for changing player number
            i.updatePlayer(dt)
        self.world.doPhysics(dt)
        return task.cont
