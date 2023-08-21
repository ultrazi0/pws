import RPi.GPIO as GPIO
from time import sleep
from configparser import ConfigParser
from math_part import angle_turret, set_angle_between_borders

halfstepping = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1],
    [1, 0, 0, 1]
]

fullstepping_power = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1]
]

fullstepping_light = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]


class Stepper:
    def __init__(self, in1, in2, in3, in4, initial_angle=0, initial_step=0, upper_border=None, lower_border=None,
        gear_ratio=(63+277/405), number_of_teeth=32, config=None, config_file = None) -> None:
        
        if (upper_border is not None) and (initial_angle > upper_border) and not config:
            raise ValueError("Initial angle cannot be greater than the upper border")
        if (lower_border is not None) and (initial_angle < lower_border) and not config:
            raise ValueError("Initian angle cannot be smaller than the lower border")
        
        self.pins = [in1, in2, in3, in4]
        # Set up pins
        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
        
        self.initial_angle = initial_angle  # not used
        self.angle = initial_angle  # current angle
        self.upper_border = upper_border
        self.lower_border = lower_border
        self.initial_step = initial_step  # not used
        self.step = initial_step  # current step
        self.gear_ratio = gear_ratio  # Characteristic of a motor
        self.number_of_teeth = number_of_teeth  # teeth INSIDE the stepper motor \ characteristic of a motor
        self.config = config  # config object
        self.config_file = config_file
        self.time_of_the_last_turn = -1
        self.turret_angle = float(config['values']['turret angle']) if (config is not None) else None
    
    # Rotates stepper by a specified amount of steps
    def rotate(self, steps, method=fullstepping_power, delay=0.001, turn_coefficient=1):
        steps = round(steps * turn_coefficient)
        
        # Method choosing -- two available
        if method == halfstepping:
            d_angle = 360 / (self.gear_ratio * self.number_of_teeth * 2)
        elif (method == fullstepping_power) or (method == fullstepping_light):
            d_angle = 360 / (self.gear_ratio * self.number_of_teeth)
        else:
            # If another method chosen
            raise RuntimeError('Specified method not supported')
        
        # Start rotating
        if steps > 0:  # Choose the direction
            for step in range(steps):

                new_angle = self.angle + d_angle  # angle after turning by 1 step
                
                # Canon angle after turning by 1 step -- IF config is given
                if self.turret_angle is not None:
                    new_turret_angle = self.turret_angle + 1 / angle_turret(1/d_angle, config=self.config)
                
                # Take borders into account, only upper border because here it is turning right
                if self.upper_border is not None:
                    # If stepper works as a turret, then check the turret angle  
                    if self.turret_angle is not None:
                        if new_turret_angle > self.upper_border:
                            print("Cannot go any further right due to the borders")
                            return False  # Quit the function
                    else:  # Else the stepper's angle
                        if new_angle > self.upper_border:
                            print("Cannot go any further right due to the borders")
                            return False  # Quit the function
                
                self.step += 1  # The next step
                # Check if the full cycle has been completed --> start from the beginning
                if self.step > len(method)-1:
                    self.step = 0
                
                # Set pins in the position (ON or OFF) according to the method
                for pin in range(len(self.pins)):
                    GPIO.output(self.pins[pin], method[self.step][pin])
                
                # Update angles
                if self.turret_angle is not None:
                    self.turret_angle = new_turret_angle
                self.angle = new_angle

                sleep(delay)  # Wait until everything is placed in position
        else:  # Turning counter-clockwise
            for step in range(-steps):  # Steps is negative --> range should be positive
                
                new_angle = self.angle - d_angle  # Angle decreases
                
                # Canon angle after turning by 1 step -- IF config is given
                if self.turret_angle is not None:
                    new_turret_angle = self.turret_angle - 1 / angle_turret(1/d_angle, config=self.config)
                
                # Take borders into account
                if self.lower_border is not None:
                    if self.turret_angle is not None:
                        if new_turret_angle < self.lower_border:
                            print("Cannot go any further left due to the borders")
                            return False
                    else:
                        if new_angle < self.lower_border:
                            print("Cannot go any further left due to the borders")
                            return False

                # Rotating in other direction, so taking steps back
                self.step -= 1
                # Check if the full cycle has been completed --> start from the beginning
                if self.step < 0:
                    self.step = len(method) - 1
                
                # Set pins in the position (ON or OFF) according to the method
                for pin in range(len(self.pins)):
                    GPIO.output(self.pins[pin], method[self.step][pin])
                
                # Update the angles
                if self.turret_angle is not None:
                    self.turret_angle = new_turret_angle
                self.angle = new_angle

                sleep(delay)  # Wait until everything is placed in position
        
        # Turn off all the pins so that there is no current, and hence the stepper can cool off
        for pin in range(len(self.pins)):
            GPIO.output(self.pins[pin], 0)
            
        self.angle = set_angle_between_borders(self.angle)  # Set the angle between -180 and 180
        if self.turret_angle is not None: # Set turret angle between -180 and 180
            self.turret_angle = set_angle_between_borders(self.turret_angle)
        
        # Write down the values to config
        if self.config is not None:
            self.config['values']['angle stepper'] = str(self.angle)
            self.config['values']['step'] = str(self.step)
            self.config['values']['turret angle'] = str(self.turret_angle)

            with open(self.config_file, 'w') as cfg_file:
                self.config.write(cfg_file)

    # Turn stepper to a specified angle
    def turn_to(self, angle, delay=0.002, method=fullstepping_light):
        turn_angle = angle - self.angle  # The angle by which the stepper should be rotated

        # Calculate the needed amount of steps
        if method == halfstepping:
            amount_of_steps = turn_angle * (self.gear_ratio * self.number_of_teeth * 2) / 360  # For halfstepping
        elif (method == fullstepping_power) or (method == fullstepping_light):
            amount_of_steps = turn_angle * (self.gear_ratio * self.number_of_teeth) / 360  # For fullstepping

        # Rotate by that amount of steps
        self.rotate(round(amount_of_steps), method=method, delay=delay)
    
    # Turn stepper by a specified angle
    def turn_by(self, angle, delay=0.002, method=fullstepping_light):
        # Calculate the needed amount of steps
        if method == halfstepping:
            amount_of_steps = angle * (self.gear_ratio * self.number_of_teeth *2) / 360  # For halfstepping
        elif (method == fullstepping_power) or (method == fullstepping_light):
            amount_of_steps = angle * (self.gear_ratio * self.number_of_teeth) / 360  # For fullstepping

        # Rotate
        self.rotate(round(amount_of_steps), method=method, delay=delay)



if __name__ == "__main__":
    GPIO.setmode(GPIO.BOARD)

    config = ConfigParser()
    config.read('cfg.ini')

    init_angle = float(config['values']['angle stepper'])
    init_step = int(config['values']['step'])

    stepper = Stepper(11, 13, 15, 16, initial_angle=init_angle, initial_step=init_step, config=config, config_file='cfg.ini')

    # stepper.rotate(1000)
    # sleep(2)
    # stepper.rotate(-1000)

    # for i in range(1000):
    #     stepper.rotate(1, do_not_use_sleep=True)
    #     sleep(0.001)

    try:
        while 1:
            try:
                inp = input('Enter angle: ')
                mode, angle = inp.split()
            except ValueError:
                mode = 'by'
                angle = inp
            angle = float(angle)

            if mode == 'by':
                stepper.turn_by(angle)
            if mode == 'to':
                stepper.turn_to(angle)

            print(stepper.angle, stepper.step)
    except KeyboardInterrupt:
        print("\nThe program closed via the Ctrl + C command")

    finally:
        print('Goodbye!')
        sleep(1)
        GPIO.cleanup()

    
