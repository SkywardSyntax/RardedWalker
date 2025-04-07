import time
import board
import neopixel
import os
import sys

# Check for root permissions
if os.geteuid() != 0:
    print("This script requires root permissions. Please run with sudo.")
    sys.exit(1)

# Configuration for the LED strip
LED_COUNT = 8        # Number of LED pixels.
LED_PIN = board.D18  # GPIO pin connected to the pixels (18 is default for DIN).
LED_BRIGHTNESS = 0.2 # Brightness of the LEDs (0.0 to 1.0).

# Create NeoPixel object with appropriate configuration.
pixels = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS, auto_write=False)

def color_wipe(color, wait_ms=50):
    """Wipe color across display a pixel at a time."""
    for i in range(LED_COUNT):
        pixels[i] = color
        pixels.show()
        time.sleep(wait_ms / 1000.0)

def theater_chase(color, wait_ms=50, iterations=10):
    """Movie theater light style chaser animation."""
    for j in range(iterations):
        for q in range(3):
            for i in range(0, LED_COUNT, 3):
                if i + q < LED_COUNT:
                    pixels[i + q] = color
            pixels.show()
            time.sleep(wait_ms / 1000.0)
            for i in range(0, LED_COUNT, 3):
                if i + q < LED_COUNT:
                    pixels[i + q] = (0, 0, 0)

def rainbow_cycle(wait_ms=20, iterations=5):
    """Draw rainbow that uniformly distributes itself across all pixels."""
    for j in range(256 * iterations):
        for i in range(LED_COUNT):
            pixel_index = (i * 256 // LED_COUNT) + j
            pixels[i] = wheel(pixel_index & 255)
        pixels.show()
        time.sleep(wait_ms / 1000.0)

def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

if __name__ == "__main__":
    try:
        while True:
            # Color wipe animations.
            color_wipe((255, 0, 0))  # Red wipe
            color_wipe((0, 255, 0))  # Green wipe
            color_wipe((0, 0, 255))  # Blue wipe

            # Theater chase animations.
            theater_chase((127, 127, 127))  # White theater chase
            theater_chase((127, 0, 0))      # Red theater chase
            theater_chase((0, 0, 127))      # Blue theater chase

            # Rainbow animations.
            rainbow_cycle()
    except KeyboardInterrupt:
        # Turn off all the pixels on exit.
        pixels.fill((0, 0, 0))
        pixels.show()