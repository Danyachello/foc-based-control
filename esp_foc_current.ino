#include <SimpleFOC.h>

//unsigned long intervalTime = 2000;
//unsigned long lastMeasureTime = 0;
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 27, 16, 14);
Encoder encoder = Encoder(18,19, 1000, 23);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}


// current sensor
InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50.0, 2, _NC, 36);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor.target, cmd); }
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() { 

  Serial.begin(115200);
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_EXTERN;

  encoder.init();
  // Hardware interrupt enable
  encoder.enableInterrupts(doA, doB, doIndex);

  Serial.println("Encoder ready");
  
  // initialize encoder sensor hardware
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
 
  // current sense init hardware
  current_sense.init();
  current_sense.linkDriver(&driver);
  current_sense.gain_c *= -1;
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set torque mode:
  motor.torque_controller = TorqueControlType::foc_current; 
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  

  // foc current control parameters (Arduino UNO/Mega)
  
  //motor.PID_current_q.P = 0.52;
  //motor.PID_current_q.I= 0.4;//2.2;
  //motor.PID_current_q.D= 1.0; //0.005;
  //motor.PID_current_d.P= 0.52;
  //motor.PID_current_d.I = 0.4;
  

  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I= 1;//2.2;
  //motor.PID_current_q.D= 0.1025; //0.005;
  motor.PID_current_d.P= 1.5;
  motor.PID_current_d.I = 1;
  //motor.PID_current_d.D = 0.003;
  motor.LPF_current_q.Tf = 0.25; 
  motor.LPF_current_d.Tf = 0.25;

  motor.voltage_limit = 12;
  motor.target = 1.0;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 3000; // set downsampling can be even more > 100
  motor.monitor_variables = _MON_CURR_Q| _MON_CURR_D; // set monitoring of d and q currents

  // initialize motor
  motor.init();

  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target current");
  command.add('m',onMotor,"my motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  //encoder.update();
  motor.loopFOC();
  //PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  //ABCurrent_s ABCurrent = current_sense.getABCurrents(currents);

  // Motion control function
  motor.move();
 
  motor.monitor();
  // user communication
  command.run();
}