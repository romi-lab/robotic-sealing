import urx
import time
import math3d as m3d
import sys

def main():
    # UR3 robot config
    robot = urx.Robot("192.168.0.2")
    mytcp = m3d.Transform()  # create a matrix for our tool tcp
    mytcp.pos.x = -0.0002
    mytcp.pos.y = -0.144
    mytcp.pos.z = 0.05
    time.sleep(1)
    robot.set_tcp(mytcp)
    time.sleep(1)
    robot.movel([0.115, -0.65, 0.432, 0, 3.14, 0], acc=0.1, vel=0.1, wait=True)
    time.sleep(1)
    robot.close()
    sys.exit(0)


main()