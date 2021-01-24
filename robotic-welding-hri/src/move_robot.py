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
    # robot.movel([0.06099578271034049, -0.6409265345073654, 0.4309693465402603, -0.8772713095361042, 2.9140923221353168, 0.3803664113609178], acc=0.1, vel=0.1, wait=True)
    robot.movel([0.115, -0.65, 0.432, 1.0657512462740044, 2.8324065157743807, -0.015681981115605197], acc=0.1, vel=0.1, wait=True)

    time.sleep(1)
    robot.close()
    sys.exit(0)


main()