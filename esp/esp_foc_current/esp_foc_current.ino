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

  /*
  В функции setup происходит определение адгоритма управления, настройка
  регуляторов и определени е необходимых ограничений для безопасно й работы привода
  */

  Serial.begin(115200);//определение обменя информации по последовательному порту
  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_EXTERN;

  encoder.init();

  encoder.enableInterrupts(doA, doB, doIndex);//разрешение прерываний для обработки сигналов с энкодера

  Serial.println("Encoder ready");
  
  motor.linkSensor(&encoder);//подключение энкодера к драйверу

  driver.voltage_power_supply = 12;//ограничение максимального напряжения драйвера
  driver.init();

  motor.linkDriver(&driver);//подключение драйвера к двигателю
 
  current_sense.init();
  current_sense.linkDriver(&driver);//подключение датчика тока к драйверу
  
  motor.linkCurrentSense(&current_sense);//подключение датчика тока к двигателю
  
  motor.torque_controller = TorqueControlType::voltage;//выбор алгоритма управления контуром тока
  
  motor.controller = MotionControlType::torque;//выбор алгоритма управления приводом
  motor.voltage_sensor_align = 1;
  motor.curr

  //задание значений регулятора
  
  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I= 1;//2.2;
  motor.PID_current_q.D= 0.1025; //0.005;
  motor.PID_current_d.P= 1.5;
  motor.PID_current_d.I = 1;
  motor.PID_current_d.D = 0.003;
  motor.LPF_current_q.Tf = 0.25; 
  motor.LPF_current_d.Tf = 0.25;

  motor.voltage_limit = 4.0;//ограничение напряжения 

  //передача определенных переменных по последовательному порту
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 2000; 
  motor.monitor_variables = _MON_ANGLE| _MON_VOLT_Q;
  
  motor.init();

  motor.initFOC();

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