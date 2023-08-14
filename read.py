from time import sleep
import RPi.GPIO as GPIO
 
GPIO.setmode(GPIO.BCM)  # Set's GPIO pins to BCM GPIO numbering
INPUT_PIN = 16
GPIO.setup(INPUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Use Pull Down to prevent false turnings on (which occur due to fluctuations)

# # Create a function to run when the input is high
# def inputLow(channel):
#     print('0');

# GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=inputLow, bouncetime=200) # Wait for the input to go low, run the function when it does

# # Start a loop that never ends
# while True:
#     print('3.3');
#     sleep(1);


try:
    while not GPIO.input(INPUT_PIN):
        print('It is still OFF')
        sleep(0.25)
    print("Now it is on, I'm quiting")
finally:
    GPIO.cleanup()
    print('\nGoodbye!')
