#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33);  //(9, 5, 6, 8);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 5);

float target = 0.1;
float k_v = 0.0;
float k_p = 0.50;
unsigned long startTime = 0;
bool eventTriggered = false;

// current sensor
//InlineCurrentSense current_sense = InlineCurrentSense(0.01, 50.0, 39, 36, _NC);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.005, 10.0, 34, 35, 32);


// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target, cmd);
}
void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}
void doSpring(char* cmd) {
  command.scalar(&k_p, cmd);
}
void doDamp(char* cmd) {
  command.scalar(&k_v, cmd);
}
//void doOff(char* cmd) { command.scalar(&flag, cmd); }



void setup() {

  /*
  В функции setup происходит определение адгоритма управления, настройка
  регуляторов и определени е необходимых ограничений для безопасно й работы привода
  */

  Serial.begin(115200);  //определение обменя информации по последовательному порту
  sensor.init();
  // link the motor to the sensor
  motor.linkSensor(&sensor);
  Serial.println("Encoder ready");


  //motor.linkSensor(&encoder);//подключение энкодера к драйверу

  driver.voltage_power_supply = 12;  //ограничение максимального напряжения драйвера
  driver.init();

  motor.linkDriver(&driver);  //подключение драйвера к двигателю

  // link the driver to the current sense
  current_sense.linkDriver(&driver);
  //current_sense.gain_b *=-1;
  //current_sense.skip_align;

  // current sense init hardware
  current_sense.init();
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  motor.voltage_sensor_align = 0.4;
  //current_sense.skip_align  = true; // default false


  motor.torque_controller = TorqueControlType::foc_current;  //выбор алгоритма управления контуром тока

  motor.controller = MotionControlType::torque;  //выбор алгоритма управления приводом

  //задание значений регулятора

  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I = 1;  //2.2;
  motor.PID_current_q.D = 0;  //0.005;
  motor.PID_current_d.P = 0.8;
  motor.PID_current_d.I = 0.5;
  motor.PID_current_d.D = 0.000;
  motor.LPF_current_q.Tf = 0.25;
  motor.LPF_current_d.Tf = 0.25;

  //motor.voltage_limit = 0.7;//ограничение напряжения
  motor.current_limit = 6;
  motor.voltage_limit = 4;
  driver.voltage_limit = 12;

  //передача определенных переменных по последовательному порту
  motor.useMonitoring(Serial);
  motor.monitor_downsample = 2000;
  motor.monitor_variables = _MON_CURR_Q | _MON_CURR_D;

  motor.zero_electric_angle = 1.9102;
  motor.sensor_direction = Direction::CCW;

  motor.init();

  motor.initFOC();

  command.add('T', doTarget, "target current");
  command.add('m', onMotor, "my motor");
  command.add('D', doDamp, "damp");
  command.add('S', doSpring, "spring");
  //command.add('O',doOff,"Off");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target current using serial terminal:"));
  _delay(1000);
  //target = motor.shaft_angle;
}


void loop() {

  // main FOC algorithm function
  motor.loopFOC();
  float p_0 = target;
  float v_0 = 0.0;
  int k = 1;
  if (fabs(p_0 - motor.shaft_angle) < 0.15) k = 0;


  //motor.disable();

  // Motion control function
  //motor.move(k_p * (p_0 - motor.shaft_angle) + k_v * k * (v_0 - motor.shaft_velocity));
  motor.move(target);


  //else if(motor.voltage.q > 1.5){
  //  motor.disable();
  // }

  if (!eventTriggered) {
    // Проверяем, было ли событие
    if (motor.voltage.q > 1.6) {
      eventTriggered = true;
      startTime = millis();  // Запоминаем время начала события
      Serial.println("Событие произошло, начинаем отсчет 20 секунд");
    }
  } else {
    unsigned long currentTime = millis();
    if (currentTime - startTime >= 20000) {  // Проверяем, прошло ли 10 секунд
      Serial.println("Прошло 20 секунд с момента события");
      eventTriggered = false;
      motor.disable();
    }
  }

  //Serial.print(motor.Ua); // milli Amps
  //Serial.print("\t");
  //Serial.println(motor.Ub); // milli Amps
  //Serial.print("\t");
  //Serial.println(motor.Uc); // milli Amps
  //motor.move(target);



  motor.monitor();
  // user communication
  command.run();
}