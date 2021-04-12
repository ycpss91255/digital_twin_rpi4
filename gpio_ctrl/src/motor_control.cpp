#include "gpio_ctrl/motor_control.h"

MotorControl::MotorControl(MD02Data DevPin) {
  this->DevPin.A.PWM = DevPin.A.PWM;
  this->DevPin.A.DIR = DevPin.A.DIR;
  this->DevPin.A.CS = DevPin.A.CS;
  this->DevPin.A.gpio_enA = DevPin.A.gpio_enA;
  this->DevPin.A.gpio_enB = DevPin.A.gpio_enB;

  this->DevPin.B.PWM = DevPin.B.PWM;
  this->DevPin.B.DIR = DevPin.B.DIR;
  this->DevPin.B.CS = DevPin.B.CS;
  this->DevPin.B.gpio_enA = DevPin.B.gpio_enA;
  this->DevPin.B.gpio_enB = DevPin.B.gpio_enB;

  this->DevPin.SLP = DevPin.SLP;

  init();
}

MotorControl::~MotorControl() {
  clear();
}

void MotorControl::init() {
  pi = pigpio_start(0, 0);
  if(pi < 0){
    printf("Can't connect to pigpio daemon\n");
    clear();
  }
  else{
  // TODO: CS pin = 50mV/A + 50mV, 0A->50mV

  MD02Data PS; // PinStatus

  PS.A.PWM = set_mode(pi, this->DevPin.A.PWM, PI_OUTPUT);
  PS.A.DIR = set_mode(pi, this->DevPin.A.DIR, PI_OUTPUT);
  PS.A.CS = set_mode(pi, this->DevPin.A.CS, PI_INPUT);
  PS.A.gpio_enA = set_mode(pi, this->DevPin.A.gpio_enA, PI_INPUT);
  PS.A.gpio_enA |= set_pull_up_down(pi, this->DevPin.A.gpio_enA, PI_PUD_UP);
  PS.A.gpio_enA |= set_glitch_filter(pi, this->DevPin.A.gpio_enA, this->glitch);
  PS.A.gpio_enB = set_mode(pi, this->DevPin.A.gpio_enB, PI_INPUT);
  PS.A.gpio_enB |= set_pull_up_down(pi, this->DevPin.A.gpio_enB, PI_PUD_UP);
  PS.A.gpio_enB |= set_glitch_filter(pi, this->DevPin.A.gpio_enB, this->glitch);

  PS.B.PWM = set_mode(pi, this->DevPin.B.PWM, PI_OUTPUT);
  PS.B.DIR = set_mode(pi, this->DevPin.B.DIR, PI_OUTPUT);
  PS.B.CS = set_mode(pi, this->DevPin.B.CS, PI_INPUT);
  PS.B.gpio_enA = set_mode(pi, this->DevPin.B.gpio_enA, PI_INPUT);
  PS.B.gpio_enA |= set_pull_up_down(pi, this->DevPin.B.gpio_enA, PI_PUD_UP);
  PS.B.gpio_enB |= set_glitch_filter(pi, this->DevPin.B.gpio_enB, this->glitch);
  PS.B.gpio_enB = set_mode(pi, this->DevPin.B.gpio_enB, PI_INPUT);
  PS.B.gpio_enB |= set_pull_up_down(pi, this->DevPin.B.gpio_enB, PI_PUD_UP);
  PS.B.gpio_enB |= set_glitch_filter(pi, this->DevPin.B.gpio_enB, this->glitch);

  PS.SLP = set_mode(pi, this->DevPin.SLP, PI_OUTPUT);


  if(((PS.A.PWM || PS.A.DIR || PS.A.CS || PS.A.gpio_enA || PS.A.gpio_enB) != 0) \
  || ((PS.B.PWM || PS.B.DIR || PS.B.CS || PS.B.gpio_enA || PS.B.gpio_enB) != 0) \
  || (PS.SLP != 0)){
    printf("Set up pin error\n");
    clear();

#ifdef DEBUG
    printf("init(DEBUG)\n");
    printf("PWM_A Pin %d Status = %s\n", this->DevPin.A.PWM, (PS.A.PWM == 0) ? "OK" : "ERROR");
    printf("DIR_A Pin %d Status = %s\n", this->DevPin.A.DIR, (PS.A.DIR == 0) ? "OK" : "ERROR");
    printf("CS_A Pin %d Status = %s\n", this->DevPin.A.CS, (PS.A.CS == 0) ? "OK" : "ERROR");
    printf("ENC_A1 Pin %d Status = %s\n", this->DevPin.A.gpio_enA, (PS.A.gpio_enA == 0) ? "OK" : "ERROR");
    printf("ENC_A2 Pin %d Status = %s\n", this->DevPin.A.gpio_enB, (PS.A.gpio_enB == 0) ? "OK" : "ERROR");

    printf("PWM_B Pin %d Status = %s\n", this->DevPin.B.PWM, (PS.B.PWM == 0) ? "OK" : "ERROR");
    printf("DIR_B Pin %d Status = %s\n", this->DevPin.B.DIR, (PS.B.DIR == 0) ? "OK" : "ERROR");
    printf("CS_B Pin %d Status = %s\n", this->DevPin.B.CS, (PS.B.CS == 0) ? "OK" : "ERROR");
    printf("ENC_B1 Pin %d Status = %s\n", this->DevPin.B.gpio_enA, (PS.B.gpio_enA == 0) ? "OK" : "ERROR");
    printf("ENC_B2 Pin %d Status = %s\n", this->DevPin.B.gpio_enB, (PS.B.gpio_enB == 0) ? "OK" : "ERROR");

    printf("SLP Pin %d Status = %s\n", this->DevPin.SLP, (PS.SLP == 0) ? "OK" : "ERROR");
#endif
    }
  }
}

void MotorControl::clear() {
  pigpio_stop(pi);
}

void MotorControl::setSpeed(float A_percent, float B_percent) {
  this->DevCmd.SLP = ((A_percent == 0) && (B_percent == 0)) ? 0 : 1;
  gpio_write(pi, this->DevPin.SLP, this->DevCmd.SLP);

  setPWM(this->DevPin.A, this->DevCmd.A, A_percent);
  setPWM(this->DevPin.B, this->DevCmd.B, B_percent);
}

void MotorControl::setPWM(MotorData Pin, MotorData Data, float percent){
  Data.DIR = (percent >= 0) ? 1 : 0;

  percent = (percent == 0) ? 0 : \
    (percent > 100 || percent < -100) ? 255 : round(abs(percent) * 2.55);

  Data.PWM = (int)percent;

  gpio_write(pi, Pin.DIR, Data.DIR);
  set_PWM_dutycycle(pi, Pin.PWM, Data.PWM);
}

// int64_t MotorControl::getEnc(){
//   // retrun
// }

void MotorControl::readEnc() {

}
