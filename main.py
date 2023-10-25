from machine import Pin
from machine import PWM
from time import sleep
import utime
import machine, neopixel


frequency = 50
# minDutyCycle = 0
maxDutyCycle = 65025
minDutyCycleScale = 5.3
maxDutyCycleScale = 10.9
dutyCycleIncVal = 0.25
currentDutyCycleScale = minDutyCycleScale
dutyCycle = 0

pwmPin = Pin(29)	# Declare the pin for PWM output.
pwmOutput = PWM(pwmPin) # Define a PWM object.
upButton   = machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_UP)
downButton = machine.Pin(1, machine.Pin.IN, machine.Pin.PULL_UP)
lastTimeButtonPressed = utime.ticks_ms()
buttonDebounceTime    = 150

# Neopixel stuff
np = neopixel.NeoPixel(machine.Pin(16), 1)
rgbMax = 70                    # Real max is 255
rgbMin = 6                     # Real min is 0
# Used to store which of neoR, neoG or neoB we are currently incrementing.
# Where 0 = neoR, 1 = neoG and 2 = neoB
incrementingB = False
incrementingG = False
# incrementingR = False
neoR = rgbMin
neoG = rgbMin
neoB = rgbMin


# Setup handler for input.
def buttonInputHandler(pin):
    global lastTimeButtonPressed
    # Handle debounce...
    currentTime = utime.ticks_ms()
    if utime.ticks_diff(currentTime, lastTimeButtonPressed) > buttonDebounceTime:
        # If after debounce time call correct button func...
        lastTimeButtonPressed = currentTime
        if pin is upButton:
            setDutyUp()
        if pin is downButton:
            setDutyDown()


upButton.irq(handler   = buttonInputHandler, trigger = machine.Pin.IRQ_RISING)
downButton.irq(handler = buttonInputHandler, trigger = machine.Pin.IRQ_RISING)


def setDutyUp():
    # If we wan't to modify a global variable within a function we must declare it as "global"
    # within that function.
    global currentDutyCycleScale

    if (currentDutyCycleScale + dutyCycleIncVal) <= maxDutyCycleScale:
        newDutyCycle = int((currentDutyCycleScale + dutyCycleIncVal) * maxDutyCycle / 100)
        currentDutyCycleScale += dutyCycleIncVal
        pwmOutput.duty_u16(newDutyCycle)

        
def setDutyDown():
    # If we wan't to modify a global variable within a function we must declare it as "global"
    # within that function.
    global currentDutyCycleScale

    if (currentDutyCycleScale - dutyCycleIncVal) >= minDutyCycleScale:
        newDutyCycle = int((currentDutyCycleScale - dutyCycleIncVal) * maxDutyCycle / 100)
        currentDutyCycleScale -= dutyCycleIncVal
        pwmOutput.duty_u16(newDutyCycle)


def neopixelMagic():
    global incrementingB
    global incrementingG
    global neoB
    global neoG
    global neoR
            
    if incrementingB:
        neoB += 1
        if(neoB > rgbMax):
            neoB = rgbMin
            incrementingB = False
    elif incrementingG:
        neoG += 1
        if(neoG > rgbMax):
            neoG = rgbMin
            incrementingG = False
        incrementingB = True
    elif not incrementingB and not incrementingG:
    	neoR += 1
        if(neoR > rgbMax):
            neoR = rgbMin
        incrementingG = True

    global np
    np[0] = (neoR, neoG, neoB)
    np.write()

def main():
    # Set the frequency.
    pwmOutput.freq(int(frequency))
    sleep(1)
    # Set initial duty cycle.
    newDutyCycle = int(5 * maxDutyCycle / 100)
    pwmOutput.duty_u16(newDutyCycle)
    sleep(2)
    # Set minimum duty cycle.
    newDutyCycle = int(currentDutyCycleScale * maxDutyCycle / 100)
    pwmOutput.duty_u16(newDutyCycle)

    while True:
        neopixelMagic()
        sleep(0.005)

# Call main() func.
main()
