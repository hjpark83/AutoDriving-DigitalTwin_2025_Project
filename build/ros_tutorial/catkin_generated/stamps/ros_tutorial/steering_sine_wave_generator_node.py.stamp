#!/usr/bin/env python3
import math
import random
import rospy
from std_msgs.msg import Float64
from ros_tutorial.msg import TutorialSineWaveInput

class SteeringSineWaveGeneratorNode:
    def __init__(self):
        rospy.init_node("steering_sine_wave_generator_node", anonymous=False)

        self.input_topic_name = rospy.get_param("~sine_command_topic", "/sine_wave/input")
        self.output_topic_name = rospy.get_param("~steering_command_topic", "/steering_cmd")

        self.max_steering_radians = math.radians(40.0)
        self.period = rospy.get_param("~initial_period", 2.0)
        self.noise_standard_deviation = rospy.get_param("~initial_noise_standard_deviation", 0.02)
        self.amplitude = rospy.get_param("~initial_amplitude", 0.3)

        # 위상 연속성 유지를 위한 상태 (phi(t) = 2π f t + phase_offset)
        self.frequency = 1.0 / self.period if self.period > 0.0 else 0.0
        self.phase_offset = 0.0
        self.last_time = rospy.get_time()

        self.publisher = rospy.Publisher(self.output_topic_name, Float64, queue_size=10)
        self.subscriber = rospy.Subscriber(self.input_topic_name, TutorialSineWaveInput, self.OnSineWaveParameters)

        self.loop_rate_hz = rospy.get_param("~loop_rate_hz", 50.0)

    # TODO: sine wave 파라미터 업데이트 및 위상 연속성 유지
    def OnSineWaveParameters(self, message):
        now = rospy.get_time()
        elapsed = now - self.last_time
        if elapsed < 0.0:
            elapsed = 0.0

        # 현재 위상 계산
        current_phase = 2.0 * math.pi * self.frequency * now + self.phase_offset

        # 새 파라미터 적용
        self.period =  message.period
        self.frequency = 1/self.period if self.period > 0.0 else 0.0
        self.noise_standard_deviation = message.noise_standard_deviation
        self.amplitude = message.amplitude

        # 위상 연속성: phi_new(now) == current_phase  =>  phase_offset_new = current_phase - 2π f_new * now
        self.phase_offset = current_phase - 2.0 * math.pi * self.frequency * now
        self.last_time = now

        rospy.loginfo("SineWave parameter update | period: %.6f, amplitude: %.6f, noise_std: %.6f",
                      self.period, self.amplitude, self.noise_standard_deviation)

    def Run(self):
        rate = rospy.Rate(self.loop_rate_hz)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            phase = 2.0 * math.pi * self.frequency * now + self.phase_offset
            pure_value = self.amplitude * math.sin(phase)
            noise_value = random.gauss(0.0, self.noise_standard_deviation)
            steering_value = pure_value + noise_value

            # 포화
            steering_value = max(-self.max_steering_radians, min(self.max_steering_radians, steering_value))

            self.publisher.publish(Float64(data=steering_value))
            rate.sleep()

if __name__ == "__main__":
    node = SteeringSineWaveGeneratorNode()
    node.Run()
