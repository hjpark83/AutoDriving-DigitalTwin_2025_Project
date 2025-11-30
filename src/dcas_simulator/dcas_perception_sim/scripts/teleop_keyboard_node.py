#!/usr/bin/env python
import sys
import termios
import tty
import rospy
from std_msgs.msg import Float32
import time

try:
    import pygame
    _HAS_PYGAME = True
except Exception:
    # install pygame: pip install pygame
    rospy.logwarn("pygame not found. Installing pygame...")
    import subprocess
    import sys
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "pygame"])
        import pygame
        rospy.loginfo("pygame successfully installed and imported")
    except subprocess.CalledProcessError as e:
        rospy.logerr(f"Failed to install pygame: {e}")
        rospy.logwarn("Please manually install pygame: pip install pygame")
    except ImportError:
        rospy.logerr("Failed to import pygame after installation")
    _HAS_PYGAME  = True


HELP = """
Keyboard Teleop (press-and-hold):
  Up    : accelerate while held
  Down  : brake while held
  Left  : steer left while held (fixed angle)
  Right : steer right while held (fixed angle)
  Space : reset immediately (accel=0, steer=0)
  q     : quit
"""


class TeleopKeyboard:
    def __init__(self) -> None:
        self.pub_steer = rospy.Publisher("/vehicle/cmd/steer", Float32, queue_size=10)
        self.pub_accel = rospy.Publisher("/vehicle/cmd/accel", Float32, queue_size=10)
        # Command magnitudes (read from params)
        self.accel_cmd = float(rospy.get_param("~accel_cmd", 1.0))      # m/s^2 when Up held
        self.brake_cmd = float(rospy.get_param("~brake_cmd", -3.0))     # m/s^2 when Down held
        self.steer_cmd_deg = float(rospy.get_param("~steer_cmd_deg", 8.0))  # deg when Left/Right held
        self.rate_hz = int(rospy.get_param("~rate_hz", 20))
        self.rate = rospy.Rate(self.rate_hz)
        self.hold_timeout_s = float(rospy.get_param("~hold_timeout_s", 0.2))
        self.use_pygame = bool(rospy.get_param("~use_pygame", False)) and _HAS_PYGAME

        # Last-command hold state
        self._last_accel = 0.0
        self._last_steer = 0.0
        self._last_t = time.time()

    def run(self) -> None:
        print(HELP)
        if self.use_pygame:
            self.run_pygame()
        else:
            self.run_terminal()

    def run_terminal(self) -> None:
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                accel = None
                steer = None
                while is_data():
                    ch = sys.stdin.read(1)
                    if ch == '\x1b':
                        if is_data() and sys.stdin.read(1) == '[' and is_data():
                            arrow = sys.stdin.read(1)
                            if arrow == 'A':
                                accel = self.accel_cmd
                            elif arrow == 'B':
                                accel = self.brake_cmd
                            elif arrow == 'C':
                                steer = -self.steer_cmd_deg
                            elif arrow == 'D':
                                steer = self.steer_cmd_deg
                    elif ch == ' ':
                        accel = 0.0
                        steer = 0.0
                    elif ch == 'q':
                        return
                self._publish_with_hold(accel, steer)
                self.rate.sleep()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    def run_pygame(self) -> None:
        pygame.init()
        screen = pygame.display.set_mode((320, 120))
        pygame.display.set_caption('Teleop (Pygame)')
        clock = pygame.time.Clock()
        try:
            while not rospy.is_shutdown():
                accel = 0.0
                steer = 0.0
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        return
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                        accel = 0.0
                        steer = 0.0
                keys = pygame.key.get_pressed()
                if keys[pygame.K_UP]:
                    accel = self.accel_cmd
                elif keys[pygame.K_DOWN]:
                    accel = self.brake_cmd
                if keys[pygame.K_RIGHT]:
                    steer = -self.steer_cmd_deg
                elif keys[pygame.K_LEFT]:
                    steer = self.steer_cmd_deg
                if keys[pygame.K_q]:
                    return
                self._publish_with_hold(accel, steer, override_timeout=True)
                clock.tick(self.rate_hz)
        finally:
            pygame.quit()

    def _publish_with_hold(self, accel, steer, override_timeout=False):
        now = time.time()
        updated = False
        if accel is not None:
            self._last_accel = accel
            updated = True
        if steer is not None:
            self._last_steer = steer
            updated = True
        if updated:
            self._last_t = now
        if override_timeout or (now - self._last_t) <= self.hold_timeout_s:
            out_accel = self._last_accel
            out_steer = self._last_steer
        else:
            out_accel = 0.0
            out_steer = 0.0
        self.pub_accel.publish(Float32(out_accel))
        self.pub_steer.publish(Float32(out_steer))


def is_data():
    import select
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main() -> None:
    rospy.init_node("teleop_keyboard_node")
    TeleopKeyboard().run()


if __name__ == "__main__":
    main()


