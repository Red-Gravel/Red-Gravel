#!/usr/bin/env python
from direct.showbase.ShowBase import ShowBase
from pandac.PandaModules import WindowProperties

from redgravel.menu import MainMenu


class Game(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        windowProperties = WindowProperties(self.win.getProperties())
        windowProperties.setTitle("Red Gravel")
        #windowProperties.setFullscreen(True)
        windowProperties.setSize(1024, 768)
        self.win.requestProperties(windowProperties)

        # Start the main menu,
        # which can start a game
        menu = MainMenu(self)


if __name__ == '__main__':
    gameApp = Game()
    gameApp.run()
