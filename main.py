#!/usr/bin/env python
from direct.showbase.ShowBase import ShowBase
from pandac.PandaModules import WindowProperties
from panda3d.core import loadPrcFileData

from redgravel.menu import MainMenu


# Set config variables before opening a window.
# Eventually we will store these in files in the
# user's data directory, with a default config file
# supplied with the game
config = """
win-size 1024 768
window-title Red Gravel
"""


class Game(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # Start the main menu,
        # which can start a game
        menu = MainMenu(self)


if __name__ == '__main__':
    loadPrcFileData("defaultConfig", config)
    gameApp = Game()
    gameApp.run()
