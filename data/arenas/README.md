Creating a Level
================

Each level is defined by a json file that defines things like the
level terrain, sky and models. Levels can be designed
in Blender and have special properties assigned to objects that
are used in the game.

The Yabee plugin for blender can be used to export an egg file
that is used by Panda. When using the exporter only the selected
objects in the scene are exported.
Note that currently Yabee doesn't output relative texture paths so
the egg file needs to be modified by hand to fix the texture paths.

Also note that for normal maps to be used, you have to select "Panda"
for the TBS generate option (generates tangents and binormals).

Terrain
-------

The terrain used by the game is defined with a grayscale png
heightmap. This must have dimensions 2^n + 1, eg. 129 by 129.
The terrain should not be exported from Blender as part of the egg file
but you can display it in blender by creating a grid mesh with dimensions
equal to the image (so 1 pixel = 1 m) and applying a
displacement modifier using the heightmap.
Make sure the terrain grid has a position of 0, 0, 0 and the midlevel
for the displacement modifier is set to 0.5. The strength of the modifier
needs to match the heightRange value in the level json file.
You can use the image editor in Blender to edit the heightmap or any
other image editor.

Properties
----------

Properties can be defined using the game logic editor in Blender.
These are exported as tags in the egg file.
The following properties can be used:

## marker

A string property set on an empty object that defines a special
position used by the game.
Possible values are:

* spawnPosition

  A starting position for vehicles in the level.

* itemPosition (todo)

  A location for special items to appear, eg. health or weapons.

## physicsObject

Possible values:

* barrier

  The object is treated as a static barrier and the geometry
  is _not displayed_. Note that the scale, translation and rotation
  are not used so these need to be applied in Blender before export.
  This is done by selecting Apply from the object menu and
  then selecting Rotation & Scale.

## linkedPhysicsObject

A string describing a physical object (eg. cylinder, cube, convex hull)
to link to this model.

Possible values:

* cylinder [radius] [height]

  A cylinder with the axial direction aligned with the Z axis.

## mass

The mass in kg of an object if it has a linked physics object.
Without it's mass set an object will be static; setting the mass
turns it into a rigid body.

## collisionSounds

Format:

    thresholdAcceleration maximumAcceleration sound1 sound2...

Eg:

    100.0 500.0 data/sounds/sound1.wav data/sounds/sound2.wav

A list of sounds to choose from when this object is impacted,
in ascending order of collision magnitude. This only works on
rigid bodies. The threshold acceleration is the minimum acceleration
before a sound is played and maximum acceleration is the acceleration
for maximum volume.
