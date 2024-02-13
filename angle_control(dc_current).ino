/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 *
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 9);
InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50, A0, _NC, A2);
//magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 6, 5, 7);
// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// velocity set point variable
float target_velocity = 20;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

void setup() {

  // initialise magnetic sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  // link current sense and driver
  current_sense.linkDriver(&driver);
  motor.linkCurrentSense(&current_sense);
  //current_sense.gain_b = -1;
 // current_sense.skip_align = true;


  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::dc_current;
  motor.controller = MotionControlType::angle;

  // contoller configuration
  // default parameters in defaults.h
  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I= 1.2;
  motor.LPF_current_q.Tf = 0.25;
  //motor.PID_current_q.limit = 0.6;


  // velocity PI controller parameters
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 1.5;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.05f;
  motor.current_limit = 4;
  motor.velocity_limit = 40;

  motor.P_angle.P = 5;

  // use monitoring with serial
  Serial.begin(9600);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_VOLT_Q | _MON_VEL | _MON_ANGLE; 
    // downsampling
  //motor.monitor_downsample = 100; // default 10

  // initialize motor
  motor.init();
  motor.voltage_sensor_align = 2;
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor.monitor();

  // user communication
  command.run();
}
