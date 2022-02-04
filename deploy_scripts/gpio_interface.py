import time
import mcpgpio

"""
This demonstrates the required functions in this module, for the specific case
of an MCP2221 HID-based GPIO controller. You can roll your own with whatever
you have to interface with, though.
"""
gpio_dev = [None]
def gpio_init():
    gpio_dev[0] = mcpgpio.init()
    mcpgpio.gpios_input(gpio_dev[0])

def gpio_toggle_reset(hold_time=0.1):
    mcpgpio.gpios_output(gpio_dev[0], 0)
    time.sleep(hold_time/2)
    mcpgpio.gpios_input(gpio_dev[0])
    time.sleep(hold_time/2)

if __name__ == '__main__':
    gpio_init()
    print("gpio inited, performing device reset")
    gpio_toggle_reset()
