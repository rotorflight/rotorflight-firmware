# Castle ESC Telemetry (Live Link)

Instead of providing telemetry on a separate serial port, Castle Creations
ESCs provide telemetry on the same wire as the throttle.  To use this,
you must use the Castle Link software to enable Live Link on the ESC.

In addition, you must add a pull-up resistor between the throttle line and
the ESC power line (or some other source of power if not using the ESC BEC).
The size of this resistor must be such as to provide a logic 1 on the input
port (V_Ih) but not to exceed the voltage tolerance on the port.  The
Castle ESC includes a 6.65Kohm pull-down, so an 8.2Kohm pull-up resistor will
work for 5V-11V on a 5V tolerant pin, and up to 7.4V on a 3.3V pin.

Once this is done, enable CASTLE PWM mode with

```
set motor_pwm_protocol = CASTLE
```

The motor_pwm_rate is limited to a maximum of 100Hz when using CASTLE PWM mode.

Leave the serial ESC sensor protocol off:

```
set esc_sensor_protocol = OFF
```

ESC Telemetry will be automatically collected from the Castle ESC when the
CASTLE PWM protocol is selected.

Note that some Castle ESCs do not provide current values.
