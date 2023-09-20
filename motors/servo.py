from gpiozero import AngularServo
from time import sleep, time
from gpiozero.pins.pigpio import PiGPIOFactory
from math import degrees, asin


class Servo():
    # Initialize a servo motor
    def __init__(self, pin, dy=3.5, r=7.5, start_angle=36, lower_border=None, upper_border=None,
                 min_pulse_width=0.5 / 1000, max_pulse_width=2.5 / 1000, initial_angle=None) -> None:

        """
        Class for a servo motor, returns a servo-motor object

        :param pin: control pin \\ USE GPIO NUMERATION
        :param dy: horizontal distance between stepper axis and cannon axis [mm]
        :param r: radius of a disk that is connected to the cannon [mm]
        :param start_angle: this parameter is used to set up the zero-position of the cannon correctly
        :param lower_border: the lowest angle allowed
        :param upper_border: the highest angle allowed
        :param min_pulse_width: refer to motor documentation
        :param max_pulse_width: refer to motor documentation
        :param initial_angle: angle at which the servo will be turned during initialization \\ None --> do not turn
        """

        # Find min and max angles so that servo's 0 would correspond with cannon's 0
        alpha0 = degrees(asin(dy / r))
        # To ensure that the servo rotates in the right direction, minimum and maximum angles should be switched
        self.min_angle = 180 - start_angle + alpha0
        self.max_angle = alpha0 - start_angle

        self.pin = pin
        self.lower_border = lower_border if lower_border is not None else self.max_angle  # max_angle because those are switched
        self.upper_border = upper_border if upper_border is not None else self.min_angle  # min_angle for the same reason
        self.initial_angle = initial_angle if self.lower_border is not None else self.lower_border
        self.angle = self.initial_angle
        self.time_of_last_turn = -1

        factory = PiGPIOFactory()  # Needed for removing the trilling? when servo is supposed to stay still

        # Actual initialization
        self.servo = AngularServo(pin, min_angle=self.min_angle, max_angle=self.max_angle,
                                  min_pulse_width=min_pulse_width, max_pulse_width=max_pulse_width,
                                  initial_angle=self.initial_angle, pin_factory=factory)

    # Function that commands servo to turn to a certain angle
    def rotate_to(self, angle, delay=0):
        # Taking borders into account
        if self.lower_border is not None:
            if angle < self.lower_border:
                print("Cannot turn any lower due to the lower border")
                return
        if self.upper_border is not None:
            if angle > self.upper_border:
                print("Cannot turn any higher due to the upper border")
                return
        self.servo.angle = angle  # Actual turning
        self.angle = self.servo.angle  # Remembering the current angle

        if delay:
            sleep(delay)

    # Function that commands servo to turn by a certain angle
    def rotate_by(self, speed, turn_coefficient=4, delay=0.15, do_not_use_sleep=False, min_activation_speed=0.1):
        if abs(speed) < min_activation_speed:
            return

        delta = speed * turn_coefficient  # Speed determines the angle; 0 < speed < 1
        turn_angle = self.angle + delta

        # Taking borders into account
        if self.lower_border is not None:
            if turn_angle < self.lower_border:
                print("Cannot turn any lower due to the lower border")
                return
        if self.upper_border is not None:
            if turn_angle > self.upper_border:
                print("Cannot turn any higher due to the upper border")
                return

        if not do_not_use_sleep:  # If "USE sleep" for delay
            # Regular turning
            self.servo.angle = turn_angle
            self.angle = self.servo.angle

        if delay:
            if do_not_use_sleep:  # If sleep is not an option because other parts should still work
                if time() - self.time_of_last_turn > delay:  # Check if the delay time has passed
                    # Regular turn
                    self.servo.angle = turn_angle
                    self.angle = self.servo.angle

                    # Remember the time of the "last" turning
                    self.time_of_last_turn = time()
            else:
                # Otherwise, if delay is specified, do it plain and simple
                sleep(delay)

    def kill(self):
        self.servo.close()


if __name__ == "__main__":
    dy = 3.5
    r = 7.5
    start_angle = 52  # Angle between the servo's "original" zero-position 
    # and the line that goes through the center of the disk and is parallel to cannon's zero-angle
    # this parameter is used to set up the zero-position of the cannon correctly

    servo = Servo(24, lower_border=None, upper_border=None, dy=dy, r=r, start_angle=start_angle, initial_angle=0)
    print(f'My min angle is {servo.min_angle} and my max angle is {servo.max_angle}')
    try:
        while 1:
            angle = float(input('Enter angle: '))
            servo.rotate_to(angle)

    except KeyboardInterrupt:
        print("\nThe program closed via the Ctrl + C command")

    finally:
        servo.angle = 0
        print('Goodbye!')
        sleep(1)
        servo.kill()
