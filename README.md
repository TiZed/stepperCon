<b>stepperCon</b> is a PIC18F46K22 based stepper motor controller.

It was created as an exercise in controlling stepper motors for a CNC project. The board and firmware were designed to drive a L298P driver but can probably be adapted to other drivers.

'master' branch - Has hybrid PWM-analog current control.
'a2d_pid' branch - Has the ADC PI current control, not stable yet.

Some of the feature of this project:
- Micro-stepping, selectable up to 16 micro-steps.
- Current limiting.
- I2C interface for configuration.
- Two indicator LEDs.
