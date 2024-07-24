#include <SimpleFOC.h>
#include <EEPROM.h>  // импортируем библиотеку

MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 53);
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
unsigned long start = 0;
// velocity set point variable
float target_velocity = 20;
int i = 0;
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


  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity;

  // contoller configuration
 // foc current control parameters (Arduino UNO/Mega)
  motor.PID_current_q.P = 1.5;
  motor.PID_current_q.I= 1;
  //motor.PID_current_q.D = 0.002;
  motor.PID_current_d.P= 1.5;
  motor.PID_current_d.I = 1;
  motor.LPF_current_q.Tf = 0.25; 
  motor.LPF_current_d.Tf = 0.25; 


  // velocity PI controller parameters
  motor.PID_velocity.P = 0.20;
  motor.PID_velocity.I = 2.0;
  motor.PID_velocity.D = 0.006;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.15f;
  motor.current_limit = 10;
  //motor.velocity_limit = 30;

  //motor.P_angle.P = 2;
  //motor.P_angle.I = 0.5;

  // use monitoring with serial
  // выводим в сериал
  Serial.begin(115200);

  motor.useMonitoring(Serial);
  motor.monitor_variables = _MON_ANGLE | _MON_VEL | _MON_CURR_D; 
  motor.monitor_downsample = 50; // default 10

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
  start = millis();
  
}

void loop() {
  
  motor.loopFOC();

  //PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  //ABCurrent_s ABCurrent = current_sense.getABCurrents(currents);
  target_velocity = trap_create(start, 10);
  motor.move(target_velocity);
  motor.monitor();
  /*
  int adr = i;
  if(millis() < 15000){
    EEPROM_float_write(adr, ABCurrent.alpha);
    i += 4;
  }
  */

  // user communication
  command.run();
}

float trap_create(unsigned long start, float value){
  float target = 0;
  if(millis() - start < 5000.00){
    target = value;
  }
  else if(((millis() - start) < 9000.00  &&((millis() - start) > 5000.00)){
    target = 0.0;
  }
  return target;
  else if(((millis() - start) > 9000.00) && ((millis() - start) < 14000.00)){
    target = -value;
  }
}

// чтение
float EEPROM_float_read(int addr) {    
  byte raw[4];
  for(byte i = 0; i < 4; i++) raw[i] = EEPROM.read(addr+i);
  float &num = (float&)raw;
  return num;
}

// запись
void EEPROM_float_write(int addr, float num) {
  byte raw[4];
  (float&)raw = num;
  for(byte i = 0; i < 4; i++) EEPROM.update(addr+i, raw[i]);
}

