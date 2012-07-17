import random
import time

from direct.showbase.ShowBase import ShowBase
from panda3d.core import Vec3, Vec4, Point3
from panda3d.core import AmbientLight, Spotlight
from pandac.PandaModules import ClockObject
from panda3d.core import Texture, TextureStage
from panda3d.core import TransformState
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape, BulletCylinderShape, BulletBoxShape
from panda3d.bullet import BulletRigidBodyNode, BulletDebugNode
from panda3d.bullet import BulletVehicle
from panda3d.bullet import XUp, YUp, ZUp


class Vehicle(object):
    def __init__(self, position, render, world):
        # Chassis uses a simple box shape
        # Note that these are half-extents:
        shape = BulletBoxShape(Vec3(0.6, 1.4, 0.5))
        ts = TransformState.makePos(Point3(0, 0, 0.5))

        self.node = BulletRigidBodyNode("vehicle")
        self.node.addShape(shape, ts)
        self.node.setMass(800.0)
        self.node.setDeactivationEnabled(False)

        np = render.attachNewNode(self.node)
        np.setPos(position)
        world.attachRigidBody(self.node)

        # Vehicle
        self.vehicle = BulletVehicle(world, self.node)
        self.vehicle.setCoordinateSystem(ZUp)
        world.attachVehicle(self.vehicle)

        self.yugoNP = loader.loadModel("models/yugo/yugo.egg")
        self.yugoNP.reparentTo(np)

        # Create wheels
        for fb, y in (("F", 1.05), ("B", -1.05)):
            for side, x in (("R", 0.7), ("L", -0.7)):
                np = loader.loadModel("models/yugo/yugotire%s.egg" % side)
                np.reparentTo(render)
                is_front = fb == "F"
                self.add_wheel(Point3(x, y, 0.3), is_front, np)

    def add_wheel(self, position, is_front, np):
        wheel = self.vehicle.createWheel()

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(position)
        wheel.setFrontWheel(is_front)

        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(0.25)
        wheel.setMaxSuspensionTravelCm(40.0)

        wheel.setSuspensionStiffness(40.0)
        wheel.setWheelsDampingRelaxation(2.3)
        wheel.setWheelsDampingCompression(4.4)
        wheel.setFrictionSlip(100.0)
        wheel.setRollInfluence(0.1)


class BulletApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.setFrameRateMeter(True)
        self.globalClock = ClockObject.getGlobalClock()

        self.cam.setPos(0, -15, 10)
        self.cam.lookAt(0, 0, 0)

        self.render.setShaderAuto()

        self.create_lights()

        self.initialise_physics(debug=False)

        self.create_ground()

        # create a pyramid of barrels
        width = 0.8
        height = 1.0
        num_rows = 5
        for row in range(num_rows):
            for column in range(num_rows - row):
                row_start = 0.5 * width * (row - num_rows + 1)
                position = [
                        row_start + column * width,
                        0,
                        row * height]
                self.create_barrel(position, height)

        self.taskMgr.add(self.game_loop, 'update')

        self.vehicle = Vehicle((0.0, -5.0, 0.5), self.render, self.world)
        self.vehicle.vehicle.applyEngineForce(400, 2)
        self.vehicle.vehicle.applyEngineForce(400, 3)
        self.vehicle.vehicle.setSteeringValue(0, 0)
        self.vehicle.vehicle.setSteeringValue(0, 1)

    def initialise_physics(self, debug=False):
        """Create Bullet world for physics objects"""

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

    def create_barrel(self, position, height=1.0):
        """
        Create a barrel with the centre of the base at the given position
        """

        # Get barrel textures
        colour = random.choice(("black", "blue", "green", "red", "yellow"))
        texture = self.loader.loadTexture("models/barrel/diffus_" + colour + ".tga")
        normal_map = self.loader.loadTexture("models/barrel/normal_hard_bumps.tga")
        specular_map = self.loader.loadTexture("models/barrel/specular_rust.tga")
        nm_ts = TextureStage('normal')
        nm_ts.setMode(TextureStage.MNormal)
        s_ts = TextureStage('specular')
        s_ts.setMode(TextureStage.MGloss)

        # Create graphical model for barrel
        barrel = self.loader.loadModel("models/barrel/metal_barrel.egg")
        barrel.setTexture(texture)
        barrel.setTexture(nm_ts, normal_map)
        barrel.setTexture(s_ts, specular_map)
        barrel_min, barrel_max = barrel.getTightBounds()

        # Y is the cylinder axis here
        scale = height  / (barrel_max[2] - barrel_min[2])
        radius = (barrel_max[0] - barrel_min[0]) * scale * 0.5

        # Create a barrel for physics
        barrel_shape = BulletCylinderShape(radius, height, ZUp)
        barrel_node = BulletRigidBodyNode('barrel')
        barrel_node.setMass(8.0)
        barrel_node.setFriction(100.0)
        barrel_node.addShape(barrel_shape)

        np = self.render.attachNewNode(barrel_node)
        position[2] = position[2] + 0.5 * height
        np.setPos(*position)
        np.setHpr(0, 0, 0)
        self.world.attachRigidBody(barrel_node)

        barrel.setScale(scale, scale, scale)
        barrel.setPos(0.0, 0.0, -0.5)
        barrel.setHpr(0, 0, 0)
        barrel.reparentTo(np)

    def create_ground(self):
        """Create ground model and physical model"""

        # Create shape for physics
        ground_shape = BulletPlaneShape(Vec3(0, 0, 1), 0)
        ground_node = BulletRigidBodyNode('ground')
        ground_node.addShape(ground_shape)
        np = self.render.attachNewNode(ground_node)
        np.setPos(0, 0, 0)
        self.world.attachRigidBody(ground_node)

        # Create graphics
        diffuse = self.loader.loadTexture("models/floor/diffuse.tga")
        normal_map = self.loader.loadTexture("models/floor/normal.tga")
        specular_map = self.loader.loadTexture("models/floor/specular.tga")
        for texture in (diffuse, normal_map, specular_map):
            texture.setWrapU(Texture.WMRepeat)
            texture.setWrapV(Texture.WMRepeat)
        nm_ts = TextureStage('normal')
        nm_ts.setMode(TextureStage.MNormal)
        s_ts = TextureStage('specular')
        s_ts.setMode(TextureStage.MGloss)

        size = 40.
        texture_size = 12.0
        # Could use the CardMaker but this doesn't create the
        # tangents and binormals required for the normal map
        floor = self.loader.loadModel("models/floor/flat.egg")
        floor.setScale(size, 1, size)
        floor.setPos(-size / 2., -size / 2., 0)
        floor.setHpr(0, -90, 0)
        floor.setTexture(diffuse)
        floor.setTexture(nm_ts, normal_map)
        floor.setTexture(s_ts, specular_map)
        for ts in (TextureStage.getDefault(), nm_ts, s_ts):
            floor.setTexScale(ts, size / texture_size, size / texture_size)

        floor.reparentTo(np)

    def create_lights(self):
        """Create an ambient light and a spotlight for shadows"""
        self.render.clearLight()

        alight = AmbientLight('ambientLight')
        alight.setColor(Vec4(0.3, 0.3, 0.3, 1))
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

    def game_loop(self, task):
        dt = self.globalClock.getDt()
        self.world.doPhysics(dt)
        return task.cont


app = BulletApp()
app.run()
