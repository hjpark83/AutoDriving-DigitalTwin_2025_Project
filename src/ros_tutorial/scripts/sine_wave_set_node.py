#!/usr/bin/env python3
import threading
import sys
import rospy
from ros_tutorial.msg import TutorialSineWaveInput

class SineWaveSetNode:
    # Node, Parameter 정의 (amplitude, period, noise_standard_deviation)
    def __init__(self):
        rospy.init_node("sine_wave_set_node", anonymous=False)

        self.topic_name = rospy.get_param("~sine_command_topic", "/sine_wave/input")

        self.period = rospy.get_param("~initial_period", 2.0)
        self.noise_standard_deviation = rospy.get_param("~initial_noise_standard_deviation", 0.02)
        self.amplitude = rospy.get_param("~initial_amplitude", 0.3)

        self.publisher = rospy.Publisher(self.topic_name, TutorialSineWaveInput, queue_size=10)
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 5.0)

        self.input_thread_should_run = True
        self.input_thread = threading.Thread(target=self.HandleUserInput)
        self.input_thread.daemon = True
        self.input_thread.start()

    def HandleUserInput(self):
        usage = (
            "\n[SineWaveSetNode] Command Examples:\n"
            "  period 1.5\n"
            "  noise 0.05\n"
            "  amplitude 0.4\n"
            "  show\n"
        )
        sys.stdout.write(usage)
        sys.stdout.flush()

        while self.input_thread_should_run and not rospy.is_shutdown():
            line = sys.stdin.readline()
            if not line:
                rospy.sleep(0.1)
                continue
            tokens = line.strip().split()
            if len(tokens) == 0:
                continue

            command = tokens[0].lower()
            try:
                if command == "period" and len(tokens) == 2:
                    value = float(tokens[1])
                    if value <= 0.0:
                        rospy.logwarn("period must be positive.")
                    else:
                        self.period = value
                        rospy.loginfo("period updated: %.6f" % self.period)

                elif command == "noise" and len(tokens) == 2:
                    value = float(tokens[1])
                    if value < 0.0:
                        rospy.logwarn("noise_standard_deviation must be greater than or equal to 0.")
                    else:
                        self.noise_standard_deviation = value
                        rospy.loginfo("noise_standard_deviation updated: %.6f" % self.noise_standard_deviation)

                elif command == "amplitude" and len(tokens) == 2:
                    value = float(tokens[1])
                    if value < 0.0:
                        rospy.logwarn("amplitude must be greater than or equal to 0.")
                    else:
                        self.amplitude = value
                        rospy.loginfo("amplitude updated: %.6f" % self.amplitude)

                elif command == "show":
                    rospy.loginfo("current settings | period: %.6f, noise_standard_deviation: %.6f, amplitude: %.6f",
                                self.period, self.noise_standard_deviation, self.amplitude)
                else:
                    rospy.logwarn("unknown command: %s", line.strip())
                    sys.stdout.write(usage)
                    sys.stdout.flush()
            except ValueError:
                rospy.logwarn("invalid numeric format: %s", line.strip())

    # TODO: sine wave 메시지 발행
    def Run(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            message = TutorialSineWaveInput() # Publish 대상 메시지

            message.period = self.period
            message.noise_standard_deviation = self.noise_standard_deviation
            message.amplitude = self.amplitude

            self.publisher.publish(message)
            rate.sleep()

if __name__ == "__main__":
    node = SineWaveSetNode()
    node.Run()
