#ifndef MotorTypedef_H
#define MotorTypedef_H
typedef struct {
  int PWM;
  int DIR;
  int CS;
  int gpio_enA;
  int gpio_enB;
}MotorPin;

typedef struct {
  int LevA;
  int LevB;
  int Status;
}EncoderData;

#endif // MotorTypedef_H
