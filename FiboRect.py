from machine import Pin, SoftI2C
from neopixel import NeoPixel
import ssd1306
from time import sleep, ticks_ms, ticks_diff
from apds9960.const import *
from apds9960 import uAPDS9960 as APDS9960

# Initialize I2C
bus = SoftI2C(sda=Pin(18), scl=Pin(23))  # Use SoftI2C for APDS9960

# Initialize OLED display
oled = ssd1306.SSD1306_I2C(128, 64, 22, 21, 60)

# Initialize Neopixel
np = NeoPixel(Pin(16, Pin.OUT), 5)  # 5 LEDs
brightness_levels = [10, 50, 100, 150, 255]  # 5 brightness levels (1-5)
current_brightness = 2  # Start at level 3 (index 2)

# Initialize APDS9960 sensor
try:
    apds = APDS9960(bus)
    apds.enableGestureSensor()
    print("APDS9960 sensor initialized successfully.")
except Exception as e:
    print("Error initializing APDS9960 sensor:", e)
    raise

# Gesture directions
dirs = {
    APDS9960_DIR_LEFT: "left",
    APDS9960_DIR_RIGHT: "right",
    APDS9960_DIR_UP: "up",
    APDS9960_DIR_DOWN: "down",
}

# Initialize Touch Sensor
SENSOR_PIN = 4  # Use GPIO4 as the touch sensor pin
try:
    touch_sensor = Pin(SENSOR_PIN, Pin.IN)  # Create a Pin object for the touch sensor
    print("Touch sensor initialized successfully.")
except ValueError as e:
    print("Error initializing touch sensor:", e)
    raise

TOUCH_THRESHOLD = 1  # Adjust based on your setup (1 for HIGH, 0 for LOW)

# Global variables
is_on = False  # Tracks whether the NeoPixels are ON or OFF
current_color = "yellow"  # Initial color is yellow
prev_touch_state = 0  # Track the previous state of the touch sensor
timer_running = False  # Tracks if the timer is running
ambient_mode = False  # Tracks if ambient mode is active
last_touch_time = 0  # Track the last touch event time
DEBOUNCE_INTERVAL = 200  # 200 ms debounce interval

# Function to set NeoPixel color and brightness
def set_neopixel_color(color, brightness_level):
    brightness = brightness_levels[brightness_level]
    if color == "yellow":
        rgb = (int(255 * brightness / 255), int(255 * brightness / 255), 0)
    elif color == "green":
        rgb = (0, int(255 * brightness / 255), 0)
    elif color == "red":
        rgb = (int(255 * brightness / 255), 0, 0)
    elif color == "purple":
        rgb = (int(128 * brightness / 255), 0, int(128 * brightness / 255))
    else:
        rgb = (0, 0, 0)  # Default to OFF
    for i in range(np.n):
        np[i] = rgb
    np.write()

# Function to turn off NeoPixel
def turn_off_neopixel():
    for i in range(np.n):
        np[i] = (0, 0, 0)
    np.write()

# Function to blink NeoPixels
def blink_neopixels(color, times=3, delay=0.5):
    for _ in range(times):
        set_neopixel_color(color, current_brightness)
        sleep(delay)
        turn_off_neopixel()
        sleep(delay)

# Function to clear the OLED display
def clear_display():
    oled.fill(0)  # Clear the screen
    oled.show()

# Function to display the initial state menu
def display_initial_menu():
    if not is_on:  # Only display if the system is ON
        return
    clear_display()
    oled.text("Welcome", 0, 0)  # Title
    oled.text("Left: Timer", 0, 15)  # Left gesture instruction
    oled.text("Right: Ambient", 0, 25)  # Right gesture instruction
    oled.text("Up: Brightness +", 0, 35)  # Up arrow for brightness increase
    oled.text("Down: Brightness -", 0, 45)  # Down arrow for brightness decrease
    oled.show()

# Countdown timer function
def countdown_timer(seconds):
    global timer_running, current_brightness, is_on
    timer_running = True
    remaining_time = seconds
    start_time = ticks_ms()
    while remaining_time > 0:
        # Check touch sensor during the timer
        touch_state = touch_sensor.value()  # Read the current state of the touch sensor
        current_time = ticks_ms()
        if prev_touch_state == 0 and touch_state == 1:  # Detect rising edge (touched)
            if ticks_diff(current_time, last_touch_time) > DEBOUNCE_INTERVAL:
                is_on = not is_on
                if not is_on:
                    print("NeoPixels turned OFF during timer")
                    turn_off_neopixel()
                    clear_display()  # Clear the OLED when the system is turned off
                    timer_running = False
                    return  # Exit the timer immediately
                last_touch_time = current_time

        if not is_on:  # Exit timer if the system is turned off
            break

        # Update display and NeoPixels
        if ticks_diff(ticks_ms(), start_time) >= 1000:  # Update every second
            start_time = ticks_ms()
            remaining_time -= 1
            clear_display()
            minutes = remaining_time // 60
            secs = remaining_time % 60
            time_str = f"Time Left: {minutes:02}:{secs:02}"
            oled.text(time_str, 0, 20)
            oled.show()
            set_neopixel_color(current_color, current_brightness)

        # Check for gestures during the timer
        if apds.isGestureAvailable():
            motion = apds.readGesture()
            gesture = dirs.get(motion, "unknown")
            print(f"Gesture detected during timer: {gesture}")
            if gesture == "right":  # Exit timer if "right" gesture is detected
                print("Exiting timer due to 'right' gesture")
                break
            elif gesture == "up":  # Increase brightness
                if current_brightness < len(brightness_levels) - 1:
                    current_brightness += 1
                    print(f"Brightness increased to level {current_brightness + 1}")
                    set_neopixel_color(current_color, current_brightness)
                    clear_display()
                    oled.text(time_str, 0, 20)
                    oled.text(f"Brightness: {current_brightness + 1}", 0, 30)
                    oled.show()
            elif gesture == "down":  # Decrease brightness
                if current_brightness > 0:
                    current_brightness -= 1
                    print(f"Brightness decreased to level {current_brightness + 1}")
                    set_neopixel_color(current_color, current_brightness)
                    clear_display()
                    oled.text(time_str, 0, 20)
                    oled.text(f"Brightness: {current_brightness + 1}", 0, 30)
                    oled.show()

        sleep(0.05)  # Small delay to reduce CPU usage

    timer_running = False
    if is_on:
        print("Timer Complete!")
        blink_neopixels(current_color)  # Blink NeoPixels after timer ends
        set_neopixel_color(current_color, current_brightness)  # Reset to normal brightness
        clear_display()
        oled.text("Timer Complete!", 0, 20)
        oled.show()
        sleep(2)  # Pause for 2 seconds
        display_initial_menu()  # Return to the initial menu

# Function to check gestures with timeout
def check_gesture_with_timeout(apds, timeout=0.5):
    start_time = ticks_ms()
    while ticks_diff(ticks_ms(), start_time) < timeout * 1000:
        if apds.isGestureAvailable():
            return apds.readGesture()
        sleep(0.05)  # Small delay to reduce CPU usage
    return None  # Return None if no gesture is detected within the timeout

# Main program
clear_display()  # Ensure the OLED is blank at startup
sleep(1)

print("System Initialized")
apds.enableGestureSensor()

while True:
    try:
        # Check touch sensor
        touch_state = touch_sensor.value()
        current_time = ticks_ms()
        if prev_touch_state == 0 and touch_state == 1:  # Detect rising edge (touched)
            if ticks_diff(current_time, last_touch_time) > DEBOUNCE_INTERVAL:
                is_on = not is_on
                if is_on:
                    print("NeoPixels turned ON (Yellow)")
                    current_color = "yellow"
                    set_neopixel_color(current_color, current_brightness)
                    display_initial_menu()
                else:
                    print("NeoPixels turned OFF")
                    turn_off_neopixel()
                    clear_display()
                    ambient_mode = False
                last_touch_time = current_time
        prev_touch_state = touch_state

        # Check gesture sensor with timeout
        motion = check_gesture_with_timeout(apds)
        if motion is None:
            continue  # Skip processing if no gesture is detected

        if is_on:
            gesture = dirs.get(motion, "unknown")
            print(f"Gesture detected: {gesture}")

            if gesture == "left" and not timer_running:
                if ambient_mode:
                    print("Exiting Ambient Mode via 'left' gesture")
                    ambient_mode = False
                    current_color = "yellow"
                    set_neopixel_color(current_color, current_brightness)
                    clear_display()
                    display_initial_menu()
                else:
                    print("Starting 25-minute timer for reading light")
                    clear_display()
                    oled.text("Starting Timer...", 0, 0)
                    oled.show()
                    sleep(2)
                    countdown_timer(25 * 60)

            elif gesture == "right":
                if not ambient_mode:
                    print("Switching to Ambient Mode")
                    ambient_mode = True
                    current_color = "purple"
                    set_neopixel_color(current_color, 1)
                    clear_display()
                    oled.text("Ambient Mode", 0, 0)
                    oled.text(f"Brightness: {current_brightness + 1}", 0, 10)
                    oled.show()

            elif gesture == "up":
                if ambient_mode:
                    if current_brightness < len(brightness_levels) - 1:
                        current_brightness += 1
                        print(f"Ambient Brightness increased to level {current_brightness + 1}")
                        set_neopixel_color(current_color, current_brightness)
                        clear_display()
                        oled.text("Ambient Mode", 0, 0)
                        oled.text(f"Brightness: {current_brightness + 1}", 0, 10)
                        oled.show()
                else:
                    if current_brightness < len(brightness_levels) - 1:
                        current_brightness += 1
                        print(f"Brightness increased to level {current_brightness + 1}")
                        set_neopixel_color(current_color, current_brightness)
                        clear_display()
                        oled.text(f"Brightness: {current_brightness + 1}", 0, 20)
                        oled.show()
                        sleep(2)
                        display_initial_menu()

            elif gesture == "down":
                if ambient_mode:
                    if current_brightness > 0:
                        current_brightness -= 1
                        print(f"Ambient Brightness decreased to level {current_brightness + 1}")
                        set_neopixel_color(current_color, current_brightness)
                        clear_display()
                        oled.text("Ambient Mode", 0, 0)
                        oled.text(f"Brightness: {current_brightness + 1}", 0, 10)
                        oled.show()
                else:
                    if current_brightness > 0:
                        current_brightness -= 1
                        print(f"Brightness decreased to level {current_brightness + 1}")
                        set_neopixel_color(current_color, current_brightness)
                        clear_display()
                        oled.text(f"Brightness: {current_brightness + 1}", 0, 20)
                        oled.show()
                        sleep(2)
                        display_initial_menu()

    except Exception as e:
        print("Error during execution:", e)
        sleep(1)  # Wait before retrying
    sleep(0.05)  # Small delay to reduce CPU usage