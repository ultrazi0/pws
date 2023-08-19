from gpiozero import LED
from time import sleep

led = LED(25)
led1 = LED(11)

led.on()
led1.on()
input('Press Enter...')
led.off()
led1.off()