import sys
from direct.gui import DirectGui
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import Vec3


class MainMenu(object):
    """
    The main menu shown when the game initially starts
    """

    def __init__(self, base):
        self.base = base
        self.guiItems = []

        # Extra arguments to pass when creating a button
        self.buttonArgs = {
            "pad": (0.5, 0.1),
            "scale": 0.1,
        }

        # Create title
        self.guiItems.append(
            OnscreenText(text="Red Gravel",
                pos=(0.0, 0.65), scale=0.15,
                fg=(1., 0., 0., 1.)))

        # Create buttons to start from 1 to 4 player games
        for numPlayers in range(1, 5):
            pos = (0.0, 0, 0.55 - 0.2 * numPlayers)
            self.guiItems.append(DirectGui.DirectButton(
                pos=Vec3(*pos),
                text="%d Player Game" % numPlayers,
                command=self.startGame,
                extraArgs=(numPlayers, ),
                **self.buttonArgs))

        self.guiItems.append(DirectGui.DirectButton(
            pos=Vec3(0.0, 0, -0.6),
            text="Quit",
            command=sys.exit,
            **self.buttonArgs))

    def clearMenu(self):
        """
        Remove all buttons and text etc. from the menu
        """

        for item in self.guiItems:
            item.destroy()

    def startGame(self, numPlayers):
        """
        Start running a game with the number of players
        selected from the menu
        """

        self.clearMenu()
        # Load the game
        from redgravel.game import World
        world = World(self.base, numPlayers)
        # Now start the game
        world.run()
