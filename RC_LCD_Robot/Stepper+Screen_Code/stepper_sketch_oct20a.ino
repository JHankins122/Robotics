#include <Stepper.h>

const int stepsPerRevolution = 2038;

Stepper myStepper = Stepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {

}

void loop(){
  //Rotate CW slowly
  myStepper.setSpeed(100);
  myStepper.step(stepsPerRevolution);
  delay(1000);
  
  //RotateCCW quickly
  myStepper.setSpeed(700);
  myStepper.step(-stepsPerRevolution);
  delay(1000);
    

}
