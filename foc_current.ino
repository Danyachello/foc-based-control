#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 6, 5, 7);

// encoder instance
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 9);

// current sensor
InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50.0, A0, _NC, A2);

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

  // set torque mode:
  motor.torque_controller = TorqueControlType::foc_current; 
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // foc current control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I= 1.2;
  //motor.PID_current_q.D = 0.002;
  motor.PID_current_d.P= 1.5;
  motor.PID_current_d.I = 0;
  motor.LPF_current_q.Tf = 0.25; 
  motor.LPF_current_d.Tf = 0.25; 


  motor.voltage_limit = 10;
  motor.target = 1;

  // use monitoring with serial 
  Serial.begin(9600);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 100; // set downsampling can be even more > 100
  motor.monitor_variables = _MON_VOLT_Q| _MON_CURR_Q | _MON_CURR_D; // set monitoring of d and q currents

  // initialize motor
  motor.init();

  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target current");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  motor.monitor();
  // user communication
  command.run();
}