import RPi.GPIO as GPIO
from time import sleep, time

class Canon:
    def __init__(self, in1, in2, en, trigger_switch, config=None) -> None:
        """
        ***DO NOT CHANGE THE POLARITY***
        
        in1 - the positive pole;
        in2 - the negative pole;
        en - the enable pin;
        trigger_switch - the pin that shows wheather the canon is charged 
        """
        self.plus = in1
        self.min = in2
        self.en = en
        self.trigger_switch = trigger_switch

        # Initialize the pins
        GPIO.setup(self.plus, GPIO.OUT)
        GPIO.setup(self.min, GPIO.OUT)
        GPIO.setup(self.en, GPIO.OUT)
        GPIO.setup(self.trigger_switch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # This one reads the pin; pull down is needed to damp fluctuations

        # Initialize constants
        self.SHOOT_BUTTON_DELAY = float(config['constants']['hold shooting button']) if config is not None else 2.5
        self.last_time_checked = -1

        # Set polarity
        GPIO.output(self.plus, 1)
        GPIO.output(self.min, 0)

        # Set up the PWM
        self.pwm = GPIO.PWM(self.en, 100)
        self.pwm.start(0)
    
    def shoot(self):
        print('Yes, Sir!')
        
        self.pwm.ChangeDutyCycle(100)  # Activate the motor
        
        print('Charging...')
        while not GPIO.input(self.trigger_switch):  # Wait until the canon is charged
            sleep(0.01)
        while GPIO.input(self.trigger_switch):  # This gives the switch time to get back into its initial state
            sleep(0.005)
        print('FIRE!')
        self.pwm.ChangeDutyCycle(0)  # Deactivate the motor
    
    def check_command(self):
        if time() - self.last_time_checked >= self.SHOOT_BUTTON_DELAY:
            return True
        else:
            self.last_time_checked = time()
            return False

if __name__ == "__main__":
    GPIO.setmode(GPIO.BOARD)
    canon = Canon(3, 5, 7, 36)
    print('Prepairing...')
    sleep(1)
    canon.shoot()
    sleep(1)
    print("We're done, Sir!")
    GPIO.cleanup()
    sleep(0.5)
    print('Goodbye!')
