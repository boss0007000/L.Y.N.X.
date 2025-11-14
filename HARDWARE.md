# Hardware Setup Guide for L.Y.N.X. Robot

This guide provides detailed instructions for assembling and wiring your L.Y.N.X. robot.

## Bill of Materials (BOM)

### Essential Components

| Component | Quantity | Description | Example Part |
|-----------|----------|-------------|--------------|
| Raspberry Pi 4 | 1 | 4GB or 8GB RAM recommended | Raspberry Pi 4 Model B |
| Raspberry Pi OS | 1 | 64-bit recommended | Raspberry Pi OS Lite/Desktop |
| USB Camera | 1 | Any USB webcam | Logitech C270 |
| DC Motors | 2 | 6V or 12V with encoders (optional) | TT Motor with Encoder |
| Servo Motor | 1 | For steering | SG90 or MG90S |
| Motor Driver | 1 | H-bridge driver | L298N or TB6612 |
| Battery/Power Supply | 1 | 7.4V LiPo or 12V battery | 2S LiPo Battery |
| Voltage Regulator | 1 | 5V/3A for Raspberry Pi | Step-down converter |
| Chassis | 1 | Robot platform | Custom or commercial |
| Wheels | 2 | For rear motors | Rubber wheels |
| Front Wheel | 1 | Steering wheel | Caster or servo-mounted wheel |
| Jumper Wires | 20+ | Male-to-female | Standard jumper wires |

### Optional Components

- Wheel Encoders (for odometry)
- IMU (for better orientation)
- LIDAR (for real-time obstacle detection)
- LED indicators
- Emergency stop button

## Wiring Diagram

### Raspberry Pi GPIO Connections

```
Raspberry Pi 4 GPIO Layout (BCM numbering):

                    3V3  [1] [2]  5V
                  GPIO2  [3] [4]  5V
                  GPIO3  [5] [6]  GND
                  GPIO4  [7] [8]  GPIO14
                    GND  [9] [10] GPIO15
                 GPIO17 [11] [12] GPIO18
                 GPIO27 [13] [14] GND
                 GPIO22 [15] [16] GPIO23
                    3V3 [17] [18] GPIO24
                 GPIO10 [19] [20] GND
                  GPIO9 [21] [22] GPIO25
                 GPIO11 [23] [24] GPIO8
                    GND [25] [26] GPIO7
                  GPIO0 [27] [28] GPIO1
                  GPIO5 [29] [30] GND
                  GPIO6 [31] [32] GPIO12
                 GPIO13 [33] [34] GND
                 GPIO19 [35] [36] GPIO16
                 GPIO26 [37] [38] GPIO20
                    GND [39] [40] GPIO21
```

### Motor Driver Connections (L298N Example)

**L298N Module:**
```
Raspberry Pi          L298N Module          Motors
-----------          -------------          ------
GPIO 17    ------>   IN1
GPIO 17    ------>   IN2                    Left Motor
GPIO 18    ------>   IN3                    (via OUT1, OUT2)
GPIO 18    ------>   IN4
                     
                     OUT1        -------->  Motor Left +
                     OUT2        -------->  Motor Left -
                     OUT3        -------->  Motor Right +
                     OUT4        -------->  Motor Right -
                     
                     +12V        <--------  Battery +
                     GND         <--------  Battery -
                     5V          -------->  (Optional: Power for Pi)
                     
GND        ------>   GND (common ground)
```

**Enable Pins (PWM):**
- Connect ENA and ENB to 5V for full speed
- Or connect to PWM-capable GPIO pins for speed control

### Servo Motor Connection (Steering)

```
Raspberry Pi          Servo Motor
-----------          ------------
GPIO 27    -------->  Signal (Yellow/White)
5V         -------->  VCC (Red)
GND        -------->  GND (Brown/Black)
```

### USB Camera Connection

```
Raspberry Pi          USB Camera
-----------          -----------
USB Port   <------->  USB Cable
```

### Power Supply Wiring

```
Battery/Power Supply:
                     +7.4V/12V
                        |
                        +-----> Motor Driver (+12V)
                        |
                        +-----> Voltage Regulator Input
                                     |
                                     +-----> 5V Output to Raspberry Pi USB-C
                                     
GND (All components must share common ground)
```

## Assembly Instructions

### Step 1: Prepare the Chassis

1. Assemble the robot chassis according to manufacturer instructions
2. Mount the motors to the chassis
3. Attach wheels to motors
4. Install the front steering mechanism

### Step 2: Mount the Raspberry Pi

1. Secure the Raspberry Pi to the chassis using standoffs
2. Ensure good ventilation around the Pi
3. Consider adding a heatsink or fan

### Step 3: Install the Motor Driver

1. Mount the L298N (or equivalent) motor driver
2. Keep it close to the motors to minimize wire length
3. Ensure heat dissipation space

### Step 4: Wire the Motors

1. Connect left motor to OUT1 and OUT2
2. Connect right motor to OUT3 and OUT4
3. Test motor direction:
   ```bash
   # Make sure motors spin in correct direction
   # If backward, swap the motor wires
   ```

### Step 5: Wire the Servo

1. Connect servo signal wire to GPIO 27
2. Connect servo power to 5V (use separate regulator if needed)
3. Connect ground to common ground
4. Test servo range:
   ```python
   # Test script to check servo movement
   import RPi.GPIO as GPIO
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(27, GPIO.OUT)
   pwm = GPIO.PWM(27, 50)
   pwm.start(7.5)  # Center position
   ```

### Step 6: Connect the Camera

1. Plug USB camera into Raspberry Pi USB port
2. Mount camera facing forward
3. Secure cable to prevent disconnection

### Step 7: Power Connections

1. Connect battery to motor driver
2. Connect voltage regulator to battery
3. Connect regulator output (5V) to Raspberry Pi
4. **IMPORTANT:** Verify voltages with multimeter before powering on
5. Connect all grounds together (common ground)

### Step 8: Final Checks

Before powering on:
- [ ] All connections are secure
- [ ] No short circuits visible
- [ ] Voltages are correct (test with multimeter)
- [ ] Motors can spin freely
- [ ] Servo can move freely
- [ ] Camera is securely mounted
- [ ] Emergency stop accessible

## Testing the Hardware

### Test 1: Power On

1. Connect battery
2. Raspberry Pi should boot (LED should light up)
3. Wait for boot to complete (~30 seconds)

### Test 2: GPIO Test

```bash
# Install GPIO utilities
sudo apt install wiringpi

# Check GPIO status
gpio readall
```

### Test 3: Camera Test

```bash
# List video devices
ls /dev/video*

# Test camera
v4l2-ctl --list-devices
raspistill -o test.jpg  # If using Pi Camera
```

### Test 4: Motor Test

```bash
# Basic motor test
python3 << EOF
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

# Spin motors briefly
GPIO.output(17, GPIO.HIGH)
GPIO.output(18, GPIO.HIGH)
time.sleep(1)
GPIO.output(17, GPIO.LOW)
GPIO.output(18, GPIO.LOW)

GPIO.cleanup()
print("Motor test complete")
EOF
```

### Test 5: Servo Test

```bash
python3 << EOF
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.OUT)
pwm = GPIO.PWM(27, 50)

pwm.start(0)
# Test left, center, right
for pos in [5, 7.5, 10]:
    pwm.ChangeDutyCycle(pos)
    time.sleep(1)

pwm.stop()
GPIO.cleanup()
print("Servo test complete")
EOF
```

## Troubleshooting

### Motors Don't Spin

- Check motor driver power supply
- Verify GPIO connections
- Test motor driver with multimeter
- Check for loose wires

### Servo Doesn't Move

- Check 5V power supply
- Verify GPIO 27 connection
- Test with different duty cycles
- Check servo is not mechanically blocked

### Camera Not Detected

- Try different USB port
- Check with `lsusb` command
- Test camera on another computer
- Check cable is not damaged

### Raspberry Pi Doesn't Boot

- Check 5V power supply voltage
- Verify SD card is properly inserted
- Check power supply current rating (min 3A)
- Test with known-good power supply

## Mechanical Considerations

### Front-Wheel Steering Setup

```
    [Servo]
       |
    [Wheel]    <- Front steering wheel
    
[Motor]      [Motor]
  |             |
[Wheel]      [Wheel]    <- Rear drive wheels
```

The servo controls the front wheel steering angle while the rear motors provide drive power.

### Wheel Base Measurement

Measure the distance between the front and rear axles. This is the `wheel_base` parameter:

```
wheel_base = distance from front axle to rear axle (in meters)
```

Update in `motor_config.yaml`:
```yaml
wheel_base: 0.30  # Example: 30 cm
```

### Wheel Radius Measurement

Measure your wheel diameter and calculate radius:

```
wheel_radius = diameter / 2 (in meters)
```

Update in `motor_config.yaml`:
```yaml
wheel_radius: 0.05  # Example: 5 cm radius
```

## Safety Guidelines

⚠️ **IMPORTANT SAFETY RULES:**

1. **Always test with robot elevated** (wheels off ground) first
2. **Start with low speeds** until you verify everything works
3. **Keep emergency stop accessible** (Ctrl+C or physical button)
4. **Disconnect battery** when making wiring changes
5. **Use proper fuses** to protect against shorts
6. **Keep hands clear** of moving parts
7. **Test in open areas** away from stairs/edges
8. **Never leave robot unattended** while powered

## Maintenance

### Regular Checks

- Inspect wires for damage weekly
- Check motor mounting screws
- Clean camera lens
- Check battery voltage
- Verify all connections are tight

### Battery Care

- Don't over-discharge LiPo batteries (use voltage alarm)
- Store at storage voltage (3.8V per cell)
- Check for swelling or damage
- Use proper LiPo charging procedure

## Next Steps

Once hardware is assembled and tested:
1. Install software (see QUICKSTART.md)
2. Configure parameters in config files
3. Calibrate motors and steering
4. Test individual components
5. Run full system integration test

## Additional Resources

- [Raspberry Pi GPIO Guide](https://www.raspberrypi.org/documentation/usage/gpio/)
- [L298N Motor Driver Tutorial](https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/)
- [Servo Motor Control](https://www.raspberrypi.org/documentation/usage/gpio/python/)
- [USB Camera on Raspberry Pi](https://www.raspberrypi.org/documentation/usage/webcams/)

Good luck with your build! 🔧
