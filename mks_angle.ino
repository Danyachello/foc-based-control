#include <SimpleFOC.h> //импорт библиотеки для векторного управления
#include <TrapezoidalPlanner.h> // импорт библиотеки для управления профилем скорости

BLDCMotor motor = BLDCMotor(7); //инициализация BLDC двигателя
BLDCDriver3PWM driver = BLDCDriver3PWM(32,33,25,22);
Encoder encoder = Encoder(19,18,1000,15);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doIndex(){encoder.handleIndex();}
//float target_angle = 10; //желаемое значение угла поворота

InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, 39, 36, _NC);

TrapezoidalPlanner planner(5);//инициализация планировщика траектории

Commander command = Commander(Serial);//инициализация класса для коммуникации с пользователем во время работы

//Определение команд для управления внутренними параметрами двигателя и планировщиком траектории
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doPlanner(char *cmd){
  planner.doTrapezoidalPlannerCommand(cmd);
}

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
  planner.linkMotor(&motor);
  
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);
  //motor.voltage_sensor_align = 0.5;
  //motor.voltage_limit = 1.0;
 
  // current sense init hardware
  current_sense.init();
  current_sense.linkDriver(&driver);
  //current_sense.gain_b *= -1;
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);
  //motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // set torque mode:
  motor.torque_controller = TorqueControlType::foc_current; 
  // set motion control loop to be used
  motor.controller = MotionControlType::angle;

  motor.voltage_limit = 0.7;
  motor.current_limit = 4;
  motor.voltage_sensor_align = 0.5;
  // foc current control parameters (Arduino UNO/Mega)
  
  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I= 1.0;//2.2;
  //motor.PID_current_q.D= 0.005;
  motor.PID_current_d.P= 0.8;
  motor.PID_current_d.I = 0.5;

  motor.LPF_current_q.Tf = 0.25; 
  motor.LPF_current_d.Tf = 0.25;

  
  motor.PID_velocity.P = 1.0;
  motor.PID_velocity.I = 2.5;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 500;

  motor.LPF_velocity.Tf = 0.25f;
  //Serial.println(planner.isPlannerMoving());
  
  //motor.current_limit = 5;
  motor.velocity_limit = 50;
  motor.LPF_angle.Tf = 0.25f;
  
  motor.P_angle.P = 2.0;
  motor.P_angle.D = 0;
  motor.P_angle.I = 0;

  //motor.target = -10;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  Serial.println(planner.isPlannerMoving());
  motor.monitor_downsample = 4000; // set downsampling can be even more > 100
  motor.monitor_variables = _MON_VEL| _MON_ANGLE; // set monitoring of d and q currents

  // initialize motor
  motor.init();

  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  //command.add('T', doTarget, "target current");
  command.add('M',onMotor,"my motor");
  command.add('G', doPlanner, "Motion Planner");


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
  planner.runPlannerOnTick();
}