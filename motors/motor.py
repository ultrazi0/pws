import RPi.GPIO as gp
from time import sleep
from numpy import sign

class Motor():  # This class was used to drive motors with L293N, not it's practically useless
    def __init__(self, plus, min, en, side, min_speed=0, min_activation_speed=None) -> None:
        self.plus = plus
        self.min = min
        self.en = en
        self.side = side  # Only 'l' or 'r'
        self.min_speed = min_speed
        self.min_activation_speed = min_activation_speed

        if self.side not in ('l', 'r'):
            raise ValueError("The side parameter must be 'l' for left or 'r' for right") 

        gp.setup(self.plus, gp.OUT)
        gp.setup(self.min, gp.OUT)
        gp.setup(self.en, gp.OUT)

        self.pwm = gp.PWM(self.en, 100)
        self.pwm.start(0)

    def move(self, speed, turn=0, time=0):
        """
        Set the motor's speed (-1<=speed<=1) and turn (-1<=turn<=1).
        Positive turn stands for right, negative for left
        Positive speed stands for forward, negative for backwards

        """
        speed *= 100
        speed = int(speed)
        turn *= 100
        turn = int(turn)
        
        if turn:
            if self.side == 'l':
                speed += turn
            else:
                speed -= turn
        
        if self.min_activation_speed is not None:
            if abs(speed) < self.min_activation_speed:
                speed = 0

        if 0 < abs(speed) < self.min_speed:
                speed = sign(speed) * self.min_speed
        
        if speed > 100:
            speed = 100
        if speed < -100:
            speed = -100

        if speed > 0:
            gp.output(self.plus, 1)
            gp.output(self.min, 0)
        elif speed <= 0:
            gp.output(self.min, 1)
            gp.output(self.plus, 0)

        
        self.pwm.ChangeDutyCycle(abs(speed))        

        if time:
            sleep(time)
            self.pwm.ChangeDutyCycle(0)
    

    def stop(self):
        self.pwm.ChangeDutyCycle(0)


class Motor_PWM():
    def __init__(self, pinA, pinB, side, min_speed=0, min_activation_speed=None) -> None:
        self.pinA = pinA
        self.pinB = pinB
        self.side = side
        self.min_speed = min_speed
        self.min_activation_speed = min_activation_speed

        if self.side not in ('l', 'r'):
            raise ValueError("The side parameter must be 'l' for left or 'r' for right") 

        # Set up the pins
        gp.setup(self.pinA, gp.OUT)
        gp.setup(self.pinB, gp.OUT)

        # Set up PWM
        self.pwmA = gp.PWM(self.pinA, 100)
        self.pwmB = gp.PWM(self.pinB, 100)

        # Set the start value
        self.pwmA.start(0)
        self.pwmB.start(0)
    
    def move(self, speed, turn=0, time=0):
        """
        Set the motor's speed (-1<=speed<=1) and turn (-1<=turn<=1).
        Positive turn stands for right, negative for left
        Positive speed stands for forward, negative for backwards

        """

        # Convert float into integer
        speed *= 100
        speed = int(speed)
        turn *= 100
        turn = int(turn)
        
        # Add turn to speed in order to determine the resulting speed of the motor
        if turn:
            if self.side == 'l':
                if speed < -1.5*self.min_activation_speed:
                    turn = -turn
                speed += turn
            else:
                if speed < -1.5*self.min_activation_speed:
                    turn = -turn
                speed -= turn
        
        # Check the min limit
        if self.min_activation_speed is not None:
            if abs(speed) < self.min_activation_speed:
                speed = 0

        # If speed is lower than the minimal speed possible
        if 0 < abs(speed) < self.min_speed:
                speed = sign(speed) * self.min_speed
        
        # Set speed between limits
        if speed > 100:
            speed = 100
        if speed < -100:
            speed = -100

        # Activate the motor
        if speed > 0:
            self.pwmA.ChangeDutyCycle(abs(speed))
            self.pwmB.ChangeDutyCycle(0)
        elif speed <= 0:
            self.pwmB.ChangeDutyCycle(abs(speed))
            self.pwmA.ChangeDutyCycle(0)

        # Sleeping
        if time:
            sleep(time)
            self.pwmA.ChangeDutyCycle(0)
            self.pwmB.ChangeDutyCycle(0)
    
    def stop(self):
        self.pwmA.ChangeDutyCycle(0)
        self.pwmB.ChangeDutyCycle(0)
    


if __name__ == "__main__":
    gp.setmode(gp.BOARD)

    #m1 = Motor(19, 21, 23, 'l', min_speed=25, min_activation_speed=20)
    m2 = Motor_PWM(8, 10, 'l', min_speed=10, min_activation_speed=10)

    sleep(1)

    #m1.move(1, 0, 5)
    m2.move(-0.4, 0, 2)
    print(1)
    m2.move(0.6, 0, 2)
    # print(1)
    # m2.move(0.6, 0, 3)
    # print(2)
    
    # m2.move(0, 0, 0.002)
    # sleep(0.01)
    # #m2.move(1, 0, 3)

    # for i in range(50, 65):
    #     print(i/100)
    #     m2.move(i/100, 0, 2.5)
    #     m2.stop()
    #     sleep(1)


    gp.cleanup()