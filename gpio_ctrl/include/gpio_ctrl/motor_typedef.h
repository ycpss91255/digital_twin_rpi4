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
  int gpio_enA;
  int LevA;
  int gpio_enB;
  int LevB;

}EncoderData;

#endif // MotorTypedef_H
