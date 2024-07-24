#include <SimpleFOC.h>
#include <TrapezoidalPlanner.h>

//unsigned long intervalTime = 2000;
//unsigned long lastMeasureTime = 0;
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);   //(9, 5, 6, 8);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);
unsigned long startTime = 0;
bool eventTriggered = false;

float target = 0;

// current sensor
//InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, 39, 36, _NC);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 10.0, 34, 35, 32);

TrapezoidalPlanner planner(5);//инициализация планировщика траектории


// instantiate the commander
Commander command = Commander(Serial);
void doPlanner(char *cmd){
  planner.doTrapezoidalPlannerCommand(cmd);
}
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&target, cmd); }

void setup() { 
  Serial.begin(115200);//определение обменя информации по последовательному порту
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  Serial.println("Encoder ready");
  
  //motor.linkSensor(&encoder);//подключение энкодера к драйверу

  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;//ограничение максимального напряжения драйвера
  driver.init();

  motor.linkDriver(&driver);
  planner.linkMotor(&motor);
//подключение драйвера к двигателю
 
  // link the driver to the current sense
  current_sense.linkDriver(&driver);
  current_sense.gain_c *=-1;
  //current_sense.skip_align;

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  //motor.voltage_sensor_align = 0.4;
  
  
  motor.torque_controller = TorqueControlType::foc_current;//выбор алгоритма управления контуром тока
  
  motor.controller = MotionControlType::angle;//выбор алгоритма управления приводом

  //задание значений регулятора
  
  motor.PID_current_q.P = 2;
  motor.PID_current_q.I= 1.0;//2.2;
  motor.PID_current_q.D= 0; //0.005;
  motor.PID_current_d.P= 0.8;
  motor.PID_current_d.I = 0.5;
  motor.PID_current_d.D = 0.000;
  motor.LPF_current_q.Tf = 0.25; 
  motor.LPF_current_d.Tf = 0.25;

  motor.voltage_limit = 1.2;//ограничение напряжения 
  motor.current_limit = 12;

  motor.PID_velocity.P = 1.0;
  motor.PID_velocity.I = 2.5;
  motor.PID_velocity.D = 0;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 500;

  motor.LPF_velocity.Tf = 0.25f;
  motor.voltage_sensor_align = 1.0;

  motor.velocity_limit = 10;
  motor.LPF_angle.Tf = 0.25f;
  
  motor.P_angle.P = 2;
  motor.P_angle.D = 0;
  motor.P_angle.I = 0;

  //motor.target = motor.shaft_angle;
 
  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 1000; // set downsampling can be even more > 100
  motor.monitor_variables = _MON_ANGLE| _MON_CURR_Q| _MON_TARGET; // set monitoring of d and q currents
  //target = motor.shaft_angle;

  // initialize motorG
  motor.zero_electric_angle = 5.55;
  //motor.phase_inductance = 0.01;
  motor.sensor_direction = Direction::CCW;
  motor.init();

  // align sensor and start FOC
  motor.initFOC();
  //motor.target = motor.shaft_angle;

  // add target command T
  command.add('G', doPlanner, "Motion Planner");
  command.add('m',onMotor,"my motor");
  command.add('T',doTarget,"target");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);
  planner.Vmax_ = 20;
  planner.Amax_= 5;
  planner.Dmax_ = 5;
}

void loop() {

  // main FOC algorithm function
  motor.loopFOC();

  if (!eventTriggered) {
    // Проверяем, было ли событие
    if (motor.voltage.q >= 1.1) {
      eventTriggered = true;
      startTime = millis();  // Запоминаем время начала события
      //Serial.println("Событие произошло, начинаем отсчет 20 секунд");
    }
  } else {
    unsigned long currentTime = millis();
    if (((currentTime - startTime) >= 10000) && (motor.voltage.q >= 1.1)) {  // Проверяем, прошло ли 10 секунд
      Serial.println("Прошло 10 секунд с момента события");
      eventTriggered = false;
      motor.disable();
    }
    else if(((currentTime - startTime) >= 10000) && (motor.voltage.q < 1.1)){
      eventTriggered = false;
    }
  }
  
  // Motion control function
  motor.move();
 
  motor.monitor();
  // user communication
  command.run();
  planner.runPlannerOnTick();
}
