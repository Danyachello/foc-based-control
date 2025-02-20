#include <SimpleFOC.h>


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(5, 27, 16, 14);
Encoder encoder = Encoder(18,19, 1000, 23);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}
float target = 0.0;
float k_v = 0.1;
float k_p = 2.0;

// current sensor
//InlineCurrentSense current_sense = InlineCurrentSense(0.001, 50.0, 2, _NC, 36);

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target, cmd); }
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doSpring(char* cmd) { command.scalar(&k_p, cmd); }
void doDamp(char* cmd) { command.scalar(&k_v, cmd); }


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
  //current_sense.init();
  //current_sense.linkDriver(&driver);
  //current_sense.gain_c *= -1;
  // link the current sense to the motor
  //motor.linkCurrentSense(&current_sense);
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set torque mode:
  motor.torque_controller = TorqueControlType::voltage; 
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  

  //motor.PID_current_q.P = 1.5;
  //motor.PID_current_q.I= 1;//2.2;
  //motor.PID_current_q.D= 0.1025; //0.005;
  //motor.PID_current_d.P= 1.5;
  //motor.PID_current_d.I = 1;
  //motor.PID_current_d.D = 0.003;
  //motor.LPF_current_q.Tf = 0.25; 
  //motor.LPF_current_d.Tf = 0.25;

  motor.voltage_limit = 4.0;
  //motor.target = 1.0;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 2000; // set downsampling can be even more > 100
  motor.monitor_variables = _MON_ANGLE| _MON_VOLT_Q; // set monitoring of d and q currents

  // initialize motor
  motor.init();

  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target current");
  command.add('m',onMotor,"my motor");
  command.add('D',doDamp,"damp");
  command.add('S',doSpring,"spring");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);
}

void loop() {

  // main FOC algorithm function
  //encoder.update();
  motor.loopFOC();

  float p_0 = target;
  float v_0 = 0.0;
  int  k = 1;
  if(fabs(p_0 - motor.shaft_angle) < 0.15) k = 0;
  
  // Motion control function
  motor.move(k_p*(p_0 - motor.shaft_angle) + k_v*k*(v_0 - motor.shaft_velocity));
 
  motor.monitor();
  // user communication
  command.run();
}