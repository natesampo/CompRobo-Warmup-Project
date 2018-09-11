import tty
import select
import sys
import termios
import interface
import Xlib.display as display

win32gui.GetCursorPos(point)

class TeleopC(object):
    def __init__ (self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.myspeedctrl = interface.SendSpeed()
        self.offsetX = display.Display().screen().root.query_pointer()._data['root_x']
        self.offsetY = display.Display().screen().root.query_pointer()._data['root_y']

    def getDirectionUnit(self):


    def run(self):
        while key != '\x03':
            self.key = self.getKey()
            if self.key == "a":
                self.myspeedctrl.send_speed(0,1)
            elif self.key =="s":
                self.myspeedctrl.send_speed(-1,0)
            elif self.key == "w":
                self.myspeedctrl.send_speed(1,0)
            elif self.key == "d":
                self.myspeedctrl.send_speed(0,-1)
            elif self.key == ' ':
                self.myspeedctrl.send_speed(0,0)
            elif self.key == 'l':
                self.myspeedctrl.low_rider()


if __name__ == "__main__":
    myTeleop = TeleopC()
    myTeleop.run()
