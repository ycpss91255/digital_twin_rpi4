#include "gpio_ctrl/motor_control.h"

MotorControl::MotorControl(MD02Data DevPin) {
  this->DevPin.A.PWM = DevPin.A.PWM;
  this->DevPin.A.DIR = DevPin.A.DIR;
  this->DevPin.A.CS = DevPin.A.CS;
  this->DevPin.A.ENC[0] = DevPin.A.ENC[0];
  this->DevPin.A.ENC[1] = DevPin.A.ENC[1];

  this->DevPin.B.PWM = DevPin.B.PWM;
  this->DevPin.B.DIR = DevPin.B.DIR;
  this->DevPin.B.CS = DevPin.B.CS;
  this->DevPin.B.ENC[0] = DevPin.B.ENC[0];
  this->DevPin.B.ENC[1] = DevPin.B.ENC[1];

  this->DevPin.SLP = DevPin.SLP;

  init();
}

MotorControl::~MotorControl() {
  pigpio_stop(PI);
}

void MotorControl::init() {
  PI = pigpio_start(0, 0);
  if(PI < 0){
      printf("Can't connect to pigpio daemon\n");
      pigpio_stop(PI);
  }
  else{
  // TODO: CS pin = 50mV/A + 50mV, 0A->50mV

  int PWM_A = set_mode(PI, this->DevPin.A.PWM, PI_OUTPUT);
  int DIR_A = set_mode(PI, this->DevPin.A.DIR, PI_OUTPUT);
  int CS_A = set_mode(PI, this->DevPin.A.CS, PI_INPUT);
  int ENC_A1 = set_mode(PI, this->DevPin.A.ENC[0], PI_INPUT);
  int ENC_A2 = set_mode(PI, this->DevPin.A.ENC[1], PI_INPUT);

  int PWM_B = set_mode(PI, this->DevPin.B.PWM, PI_OUTPUT);
  int DIR_B = set_mode(PI, this->DevPin.B.DIR, PI_OUTPUT);
  int CS_B = set_mode(PI, this->DevPin.B.CS, PI_INPUT);
  int ENC_B1 = set_mode(PI, this->DevPin.B.ENC[0], PI_INPUT);
  int ENC_B2 = set_mode(PI, this->DevPin.B.ENC[1], PI_INPUT);

  int SLP = set_mode(PI, this->DevPin.SLP, PI_OUTPUT);

  if((PWM_A != 0) && (DIR_A != 0) && (CS_A != 0) && (ENC_A1 != 0) && (ENC_A2 != 0) && \
     (PWM_B != 0) && (DIR_B != 0) && (CS_B != 0)  && (ENC_B1 != 0) && (ENC_B2 != 0) && \
     (SLP != 0) ){
    printf("Set up pin error\n");
    pigpio_stop(PI);

#ifdef DEBUG
    printf("init(DEBUG)\n");
    printf("PWM_A Pin %d Status = %s\n", this->DevPin.A.PWM, (PWM_A == 0) ? "OK" : "ERROR");
    printf("DIR_A Pin %d Status = %s\n", this->DevPin.A.DIR, (DIR_A == 0) ? "OK" : "ERROR");
    printf("CS_A Pin %d Status = %s\n", this->DevPin.A.CS, (CS_A == 0) ? "OK" : "ERROR");
    printf("ENC_A1 Pin %d Status = %s\n", this->DevPin.A.ENC[0], (ENC_A1 == 0) ? "OK" : "ERROR");
    printf("ENC_A2 Pin %d Status = %s\n", this->DevPin.A.ENC[1], (ENC_A2 == 0) ? "OK" : "ERROR");

    printf("PWM_B Pin %d Status = %s\n", this->DevPin.B.PWM, (PWM_B == 0) ? "OK" : "ERROR");
    printf("DIR_B Pin %d Status = %s\n", this->DevPin.B.DIR, (DIR_B == 0) ? "OK" : "ERROR");
    printf("CS_B Pin %d Status = %s\n", this->DevPin.B.CS, (CS_B == 0) ? "OK" : "ERROR");
    printf("ENC_B1 Pin %d Status = %s\n", this->DevPin.B.ENC[0], (ENC_B1 == 0) ? "OK" : "ERROR");
    printf("ENC_B2 Pin %d Status = %s\n", this->DevPin.B.ENC[1], (ENC_B2 == 0) ? "OK" : "ERROR");

    printf("SLP Pin %d Status = %s\n", this->DevPin.SLP, (SLP == 0) ? "OK" : "ERROR");
#endif
    }
  }
}

void MotorControl::setSpeed(float A_percent, float B_percent) {
  this->DevCmd.SLP = ((A_percent == 0) && (B_percent == 0)) ? 0 : 1;
  gpio_write(PI, this->DevPin.SLP, this->DevCmd.SLP);

  setPWM(this->DevPin.A, this->DevCmd.A, A_percent);
  setPWM(this->DevPin.B, this->DevCmd.B, B_percent);
}

void MotorControl::setPWM(MotorData Pin, MotorData Data, float percent){
  Data.DIR = (percent >= 0) ? 1 : 0;

  percent = (percent == 0) ? 0 : \
    (percent > 100 || percent < -100) ? 255 : round(abs(percent) * 2.55);

  Data.PWM = (int)percent;

  gpio_write(PI, Pin.DIR, Data.DIR);
  set_PWM_dutycycle(PI, Pin.PWM, Data.PWM);
}

// int64_t MotorControl::getEnc(){
//   // retrun
// }

void MotorControl::readEnc(){

}
