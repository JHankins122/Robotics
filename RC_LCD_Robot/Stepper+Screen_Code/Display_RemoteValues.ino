
/*
 *  JHankins122
 *  - Headers: Stepper Motors, I2C, LCD, 
 * 
 * 
 */


#include <Stepper.h>
#include <AccelStepper.h> // Include the AccelStepper library:
#include <MultiStepper.h>
#include <IRremote.hpp>
#include <Wire.h>
#include <hd44780.h>                       // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h> // i2c expander i/o class header


// Motor pin definitions:
#define leftmotor1  8      // IN1 on the ULN2003 driver
#define leftmotor2  9      // IN2 on the ULN2003 driver
#define leftmotor3  10     // IN3 on the ULN2003 driver
#define leftmotor4  11     // IN4 on the ULN2003 driver

#define rightmotor1 4      // IN1 on the ULN2003 driver
#define rightmotor2 5      // IN2 on the ULN2003 driver
#define rightmotor3 6      // IN3 on the ULN2003 driver
#define rightmotor4 7      // IN4 on the ULN2003 driver

// Define the AccelStepper interface type; 4 wire motor in half step mode:
#define MotorInterfaceType 8

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper library with 28BYJ-48 stepper motor:
AccelStepper leftStepper = AccelStepper(MotorInterfaceType, leftmotor1, leftmotor3, leftmotor2, leftmotor4);
AccelStepper rightStepper = AccelStepper(MotorInterfaceType, rightmotor1, rightmotor3, rightmotor2, rightmotor4);


hd44780_I2Cexp lcd; // declare lcd object: auto locate & auto config expander chip


// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
// Infrared Reciever
const int RECV_PIN = 12;
IRrecv irrecv(RECV_PIN);
decode_results results;
// Stepper Motors
// Motors : Left & Right pins 4,5,6,7, 8,9,10,11

void setup()
 
{
    // Set the maximum steps per second:
  
  rightStepper.setMaxSpeed(1000);
  leftStepper.setMaxSpeed(1000);
  // Setup IR
  irrecv.enableIRIn(); // Start the receiver
  irrecv.blink13(true);
  // Setup LCD
  int status;
  Serial.begin(96000);
	status = lcd.begin(LCD_COLS, LCD_ROWS);
	if(status) // non zero status means it was unsuccesful
	{
		// hd44780 has a fatalError() routine that blinks an led if possible
		// begin() failed so blink error code using the onboard LED if possible
		hd44780::fatalError(status); // does not return
	}

	// initalization was successful, the backlight should be on now

	// Print a message to the LCD
  lcd.clear();
  lcd.setCursor(4,0);
  lcd.print("Made by");
  lcd.setCursor(3,1);
  lcd.print("JHankins122");
              rightStepper.setSpeed(-500);
            leftStepper.setSpeed(500);
            // Step the motor with constant speed as set by setSpeed():
            leftStepper.runSpeed();
            rightStepper.runSpeed();
}

void loop(){
  if (irrecv.decode(&results)){
        lcd.clear();
        lcd.print(results.value, HEX);
        Serial.println(results.value, HEX);
        irrecv.resume();
        //lcd.print("Press Another");
        if (results.value == 0xFF629D)
        {
            // Set the speed of the motor in steps per second:
            lcd.print("Up arrow");
            rightStepper.setSpeed(-500);
            leftStepper.setSpeed(500);
            // Step the motor with constant speed as set by setSpeed():
            leftStepper.runSpeed();
            rightStepper.runSpeed();
        }
  }
}
