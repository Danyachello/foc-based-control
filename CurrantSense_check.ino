#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 6, 5, 7);

// encoder instance
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 9);

// current sensor
InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, A0, A2, A1);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

void setup() { 
  
  // initialize encoder sensor hardware
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // current sense init hardware
  current_sense.init();
  current_sense.linkDriver(&driver);
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  //current_sense.gain_b *= -1;
  current_sense.skip_align = true;

  // aligning voltage
  motor.voltage_sensor_align = 2;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  // add current limit
  motor.phase_resistance = 25; // [Ohm]
  motor.voltage_limit = 2;   // [Amps] - if phase resistance defined
  //motor.voltage_limit = 3;

  // use monitoring with serial 
  Serial.begin(9600);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 100; // set downsampling can be even more > 100
  motor.monitor_variables = _MON_VOLT_Q | _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // set the initial motor target
  motor.target = 0.05; // Amps - if phase resistance defined  
  //
  // add target command T
  // command.add('T', doTarget, "target current"); // - if phase resistance defined
  command.add('T', doTarget, "target voltage");
  command.verbose = VerboseMode::nothing; // disable commander output to serial

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  // display the currents
  motor.monitor();
  // user communication
  command.run();
}