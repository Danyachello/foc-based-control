#include <SimpleFOC.h>

// magnetic sensor instance - SPI
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 9);
InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50, A0, _NC, A2);
unsigned long intervalTime = 2000;
unsigned long lastMeasureTime = 0;
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7, NOT_SET, 335);
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 6, 5, 7);


// velocity set point variable

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }

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
  driver.pwm_frequency = 20000;
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // set motion control loop to be used
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::angle;

 
 // foc current control parameters (Arduino UNO/Mega)
 /*
  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I= 1.2;
  motor.PID_current_d.P= 1.5;
  motor.PID_current_d.I = 1.2;
  motor.LPF_current_q.Tf = 0.25; 
  motor.LPF_current_d.Tf = 0.25; 
*/
  // velocity PI controller parameters
  motor.PID_velocity.P = 0.25;
  motor.PID_velocity.I = 1.5;
  motor.PID_velocity.D = 0.004;

  // jerk control using voltage voltage ramp
  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.10f;
  motor.current_limit = 10;
  motor.velocity_limit = 10;
  motor.LPF_angle.Tf = 0.0025f;

  motor.P_angle.P = 4;
  //motor.P_angle.D = 0.001;
  //motor.P_angle.I = 0.25;

  // use monitoring with serial
  Serial.begin(460800);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_CURR_Q | _MON_VEL | _MON_ANGLE; 
    // downsampling
  motor.monitor_downsample = 100; // default 10

  // initialize motor
  motor.init();
  motor.voltage_sensor_align = 2;
  // align sensor and start FOC
  motor.initFOC();
  motor.target = 5;

  // add target command T
  command.add('T', doTarget, "target angle");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
 
  motor.loopFOC();
  //PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  //ABCurrent_s ABCurrent = current_sense.getABCurrents(currents);
  //myTime = millis();



  
  motor.move();
  /*
   if ( millis() - lastMeasureTime > intervalTime ) {
    lastMeasureTime += intervalTime;
    //Serial.print(ABCurrent.alpha*1000); // milli Amps
    //Serial.print("\t");
    Serial.println(motor.current_sp);
  }
  */
  

  motor.monitor();

  // user communication
  command.run();
}
