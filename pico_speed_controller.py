from machine import Pin, ADC, PWM, I2C
import i2c_lcd
import lcd_api
from time import sleep

# Constants
PWM_LED_PIN = 2          # GPIO pin for the LED
FREQ_POT_PIN = 28        # GPIO pin for the frequency potentiometer
DUTY_POT_PIN = 27        # GPIO pin for the duty cycle potentiometer
I2C_SDA_PIN = 0          # GPIO pin for I2C SDA
I2C_SCL_PIN = 1          # GPIO pin for I2C SCL
I2C_ADDR = 0x27          # I2C address of the LCD

# Frequency settings
MIN_FREQ = 500           # Minimum PWM frequency in Hz
MAX_FREQ = 5000          # Maximum PWM frequency in Hz

# Duty cycle settings
MIN_DUTY = 0             # Minimum duty cycle (0%)
MAX_DUTY = 65535         # Maximum duty cycle (100%)

# Initialize I2C
i2c = I2C(0, sda=Pin(I2C_SDA_PIN), scl=Pin(I2C_SCL_PIN))

# Initialize LCD
lcd = i2c_lcd.I2cLcd(i2c, I2C_ADDR, 2, 16)  # 2 rows, 16 columns

# Initialize PWM LED
led = PWM(Pin(PWM_LED_PIN))

# Initialize potentiometers
freq_pot = ADC(Pin(FREQ_POT_PIN))
duty_pot = ADC(Pin(DUTY_POT_PIN))

def read_frequency():
    """Read the frequency potentiometer value and map it to a frequency."""
    pot_value = freq_pot.read_u16()  # Read raw ADC value (0-65535)
    freq = MIN_FREQ + (MAX_FREQ - MIN_FREQ) * (pot_value / 65535)
    return int(freq)

def read_duty_cycle():
    """Read the duty cycle potentiometer value and map it to a duty cycle."""
    pot_value = duty_pot.read_u16()  # Read raw ADC value (0-65535)
    duty = MIN_DUTY + (MAX_DUTY - MIN_DUTY) * (pot_value / 65535)
    return int(duty)

# Display static information on LCD
lcd.clear()
lcd.move_to(0, 0)
lcd.putstr("Freq:     Hz")
lcd.move_to(0, 1)
lcd.putstr("Duty:    %")

while True:
    # Read and set PWM frequency from frequency potentiometer
    frequency = read_frequency()
    led.freq(frequency)
    
    # Read and set PWM duty cycle from duty cycle potentiometer
    duty_cycle = read_duty_cycle()
    led.duty_u16(duty_cycle)
    
    # Calculate duty cycle percentage
    duty_percentage = (duty_cycle / MAX_DUTY) * 100
    
    # Update only the dynamic parts of the display
    lcd.move_to(6, 0)  # Move to the position where frequency is displayed
    lcd.putstr(f"{frequency:>4} Hz")  # Display frequency with padding for alignment
    
    lcd.move_to(6, 1)  # Move to the position where duty cycle is displayed
    lcd.putstr(f"{duty_percentage:>5.1f}%")  # Display duty cycle with padding for alignment
    
    sleep(0.1)  # Small delay to avoid high CPU usage

