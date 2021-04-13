#include "gpio_ctrl/motor_control.h"

MotorControl::MotorControl(MotorPin Pin) {
  this->Pin.PWM = Pin.PWM;
  this->Pin.DIR = Pin.DIR;
  this->Pin.CS = Pin.CS;
  this->Pin.gpio_enA = Pin.gpio_enA;
  this->Pin.gpio_enB = Pin.gpio_enB;

  init();
}
MotorControl::MotorControl(MotorPin Pin, int SLP_pin) {
  this->Pin.PWM = Pin.PWM;
  this->Pin.DIR = Pin.DIR;
  this->Pin.CS = Pin.CS;
  this->Pin.gpio_enA = Pin.gpio_enA;
  this->Pin.gpio_enB = Pin.gpio_enB;
  init();
  setDevSLP(SLP_pin);
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
  MotorPin PinStatus;

  PinStatus.PWM = set_mode(pi, this->Pin.PWM, PI_OUTPUT);
  PinStatus.DIR = set_mode(pi, this->Pin.DIR, PI_OUTPUT);


  PinStatus.CS = set_mode(pi, this->Pin.CS, PI_INPUT);

  PinStatus.gpio_enA = set_mode(pi, this->Pin.gpio_enA, PI_INPUT);
  PinStatus.gpio_enA |= set_pull_up_down(pi, this->Pin.gpio_enA, PI_PUD_UP);
  PinStatus.gpio_enA |= set_glitch_filter(pi, this->Pin.gpio_enA, this->glitch);

  PinStatus.gpio_enB = set_mode(pi, this->Pin.gpio_enB, PI_INPUT);
  PinStatus.gpio_enB |= set_pull_up_down(pi, this->Pin.gpio_enB, PI_PUD_UP);
  PinStatus.gpio_enB |= set_glitch_filter(pi, this->Pin.gpio_enB, this->glitch);

  #ifdef DEBUG
    Pin_printf("PWM", this->Pin.PWM, PinStatus.PWM);
    Pin_printf("DIR", this->Pin.DIR, PinStatus.DIR);
    Pin_printf("CS", this->Pin.CS, PinStatus.CS);
    Pin_printf("GPIO_ENA", this->Pin.gpio_enA, PinStatus.gpio_enA);
    Pin_printf("GPIO_ENB", this->Pin.gpio_enB, PinStatus.gpio_enB);
  #endif

  if((PinStatus.PWM || PinStatus.DIR || PinStatus.CS || PinStatus.gpio_enA || PinStatus.gpio_enB) != 0){
    printf("Set up pin error\n");
    clear();
    }
  }
}

void MotorControl::clear() {
  gpio_write(pi, this->Pin.DIR, 0);
  set_PWM_dutycycle(pi, this->Pin.PWM, 0);
  pigpio_stop(pi);
}

void MotorControl::setSpeed(float percent) {

  this->Cmd.DIR = (percent >= 0) ? 1 : 0;

  percent = (percent == 0) ? 0 : \
    (percent > 100 || percent < -100) ? 255 : round(abs(percent) * 2.55);
  this->Cmd.PWM = (int)percent;

  #ifdef DEBUG
    printf("CMD DIR = %d\n", this->Cmd.DIR);
    printf("CMD PWM = %d\n", this->Cmd.PWM);

  #endif
  gpio_write(pi, this->Pin.DIR, this->Cmd.DIR);
  set_PWM_dutycycle(pi, this->Pin.PWM, this->Cmd.PWM);

}

void MotorControl::setGlitch(int glitch){
  if(glitch >= 0 && (this->glitch != glitch)){
    this->glitch = glitch;
    set_glitch_filter(pi, this->Pin.gpio_enA, this->glitch);
    set_glitch_filter(pi, this->Pin.gpio_enB, this->glitch);
  }
}

void MotorControl::setDevSLP(int SLP_pin){
  this->SLP_pin = SLP_pin;
  int PinStatus = set_mode(pi, this->SLP_pin, PI_OUTPUT);
  gpio_write(pi, this->SLP_pin, 1);
  PinStatus |= set_pull_up_down(pi, this->SLP_pin, PI_PUD_UP);

  #ifdef DEBUG
    Pin_printf("SLP", this->SLP_pin, PinStatus);
  #endif
}

void MotorControl::getCSValue(){
  // TODO : CS pin = 50mV/A + 50mV, 0A->50mV read value range (0~255)
  // TODO : get CS value function
}

// DEBUG pin function
#ifdef DEBUG
  void MotorControl::Pin_printf(const char*text, int pin, int status){
    printf("%s Pin %d Status = %s\n", text, pin, (status == 0) ? "OK" : "Error");
  }
#endif

