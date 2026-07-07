# Non-blocking keyboard input helper.

import fcntl
import os
import sys
import termios


def getkey():
    """Return pressed key code, or 0 when no key is available."""
    fno = sys.stdin.fileno()

    attr_old = termios.tcgetattr(fno)
    attr = termios.tcgetattr(fno)
    attr[3] = attr[3] & ~termios.ECHO & ~termios.ICANON
    termios.tcsetattr(fno, termios.TCSADRAIN, attr)

    fcntl_old = fcntl.fcntl(fno, fcntl.F_GETFL)
    fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old | os.O_NONBLOCK)

    key_code = 0

    try:
        c = sys.stdin.read(1)
        while len(c):
            key_code = (key_code << 8) + ord(c)
            c = sys.stdin.read(1)
    finally:
        fcntl.fcntl(fno, fcntl.F_SETFL, fcntl_old)
        termios.tcsetattr(fno, termios.TCSANOW, attr_old)

    return key_code


if __name__ == "__main__":
    while True:
        key = getkey()
        if key == 10:
            break
        if key:
            print(key)
