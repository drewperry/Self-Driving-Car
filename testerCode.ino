#include <Arduino.h>
#include <Servo.h>
#include <Ultrasonic.h>
#include "pin_numbers.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_BNO055.h>


// The following minimum and maximum command values are technically arbitrary, so I'm going to pick values
// such that negative means reverse, postive means forward, and zero means stop, since that makese sense to me.
#define THRUST_CMD_MIN -100
#define THRUST_CMD_MAX 100
// notice I didn't actually specify a 0 value, 0 will mean stop as a consequence of having a symetric minimum and maximum value: 0 will be in the
// middle of these two values, which will correspond to the middle value of the servo minimum and maximum, which corresponds to "stop".

// Same as above, I'm just picking values so that negative means left and postivie right, and the numbers should
// roughly correspond with the steering angle which we know is constrained to -35 to 35 degrees.
#define STEERING_CMD_MIN -35
#define STEERING_CMD_MAX 35

// The following pin numbers are determined by the PCB design and should not be changed,
// unless you have had to re-wire the servo lines to different pins for some reason.
// This sets the steering servo pin to 44.
#define ARDUINO_STEERING_PIN 44
// This sets the thrust servo pin to 46.
#define ARDUINO_THRUST_PIN 46

// This sets the minimum and maximum values in microseconds
// of the steering servo pulse width, These values are determined by the hardware and should not be changed.
#define STEERING_SERVO_MIN 919
#define STEERING_SERVO_MAX 2088
// This sets the minimum and maximum values in microseconds
// of the drive servo pulse width
#define THRUST_SERVO_MIN 1292
#define THRUST_SERVO_MAX 1688
// Strictly speaking, the above #define lines are not necessary, See note 1 for an explanation.
  // This sets the ultrasonic distance sensor trigger pin to output mode
// These are the steering and thrust servo objects
// They store configuration information for each servo
Servo steering_servo;
Servo thrust_servo;

// These will be our variables to control the thrust and steering. Values will be on the range defined by the *_CMD_MIN and *_CMD_MAX lines at the top of this file.
int thrust_command = 0;
int steering_command = 0;
int safetyDistance = 1000;
int lidarDistance = 700;
int ultrasonic_distance;
double initial_theta;
boolean isTurningLidar;
boolean isTurningUltras;
double t;

Adafruit_VL53L0X optical_sensor = Adafruit_VL53L0X();
Adafruit_BNO055 imu_sensor = Adafruit_BNO055( 55 );
VL53L0X_RangingMeasurementData_t optical_distance_object;
int optical_distance;
imu::Vector<3> euler_angles;



//Ultrasonic ultrasonicThree(ULTRASONIC_TRIGGER_PIN_3, ULTRASONIC_ECHO_PIN_3);  // for a sensor mounted on the left side
Ultrasonic ultrasonicFour(ULTRASONIC_TRIGGER_PIN_4, ULTRASONIC_ECHO_PIN_4); // for a sensor mounted on the right side
void setup() {
  delay(4000);
   initial_theta = euler_angles.x();

  // These lines configure the arduino hardware to interface with the two servos
  steering_servo.attach( ARDUINO_STEERING_PIN, STEERING_SERVO_MIN, STEERING_SERVO_MAX );
  thrust_servo.attach( ARDUINO_THRUST_PIN, THRUST_SERVO_MIN, THRUST_SERVO_MAX );
    // This sets the ultrasonic distance sensor trigger pin to output mode
  pinMode( ULTRASONIC_TRIGGER_PIN_4, OUTPUT );
  // This sets the ultrasonic distance sensor echo pin to input mode
  pinMode( ULTRASONIC_ECHO_PIN_4, INPUT );
  Serial.begin(9600);
  delay(10);
  Serial.println("start up");

    if ( !imu_sensor.begin() )
  {
      // This prints an error stating that Arduino was unable to connect to the BNO055
      Serial.println(F("Cannot successfully connect to BNO055."));
      // This pauses the program execution
      while(1);
  }
    imu_sensor.setExtCrystalUse( true );

    if ( !optical_sensor.begin() )
  {
    // If defined by the user, this prints an error to the terminal

      // This prints an error stating that Arduino was unable to connect to the VL53L0X
      Serial.println(F("Cannot successfully connect to VL53L0X."));
      // This pauses the program execution
      while(1);
  }
}
unsigned long printInterval = 500;
unsigned long lastPrint = millis();
void loop() {
  optical_distance = measure_optical_distance();
  ultrasonic_distance = measure_ultrasonic_distance( ULTRASONIC_TRIGGER_PIN_4, ULTRASONIC_ECHO_PIN_4);
  euler_angles = imu_sensor.getVector( Adafruit_BNO055::VECTOR_EULER );
  int constant = -6;

double  theta_sensor = euler_angles.x() - initial_theta;

  thrust_command = 27;
    if (isTurningLidar)
  {
    delay(700);
    steering_command = 15;
   int steering_value = constrain(
                         map(steering_command, STEERING_CMD_MIN, STEERING_CMD_MAX, STEERING_SERVO_MIN, STEERING_SERVO_MAX),
                         STEERING_SERVO_MIN, STEERING_SERVO_MAX);
       steering_servo.write( steering_value);

    isTurningLidar = false;
    isTurningUltras = false;
    delay(1200);
    steering_command = 0;
  }
  else if (isTurningUltras)
  {
       delay(700);
    steering_command = 25;

    isTurningUltras = false;
    isTurningLidar = false;
   int steering_value = constrain(
                         map(steering_command, STEERING_CMD_MIN, STEERING_CMD_MAX, STEERING_SERVO_MIN, STEERING_SERVO_MAX),
                         STEERING_SERVO_MIN, STEERING_SERVO_MAX);
       steering_servo.write( steering_value);
       delay(1200);
     steering_command = 0;
  }
  else if (ultrasonic_distance <= safetyDistance || optical_distance <= safetyDistance)
  {
    if (ultrasonic_distance <= safetyDistance && !isTurningUltras)
    {
      steering_command = -25;
      isTurningUltras = true;
      
    }
    else if (optical_distance <= lidarDistance && optical_distance != -1 && !isTurningLidar)
    {
      steering_command = -20;
      isTurningLidar = true;
    }
  }
//  else if ( ( theta_sensor < 0.5 ) && ( theta_sensor > -0.5) )
//  {
//    steering_command = 0;
//  }
//  else
//  {
//     steering_command = constant * theta_sensor * steering_command;
//
//  }
  
   int thrust_value = constrain( //note in previous examples we did not use the constrain function. See Note 2
                       map(thrust_command, THRUST_CMD_MIN, THRUST_CMD_MAX, THRUST_SERVO_MIN, THRUST_SERVO_MAX),
                       THRUST_SERVO_MIN, THRUST_SERVO_MAX);

   int steering_value = constrain(
                         map(steering_command, STEERING_CMD_MIN, STEERING_CMD_MAX, STEERING_SERVO_MIN, STEERING_SERVO_MAX),
                         STEERING_SERVO_MIN, STEERING_SERVO_MAX);

  
  steering_servo.write( steering_value);
  thrust_servo.write(thrust_value);

  

  
          
  // Here's where you might read from the sensors to determine what values you want to set thrust_command and steering_command to.
  //unsigned int ultrasonicFour = measure_ultrasonic_distance( ULTRASONIC_TRIGGER_PIN_4, ULTRASONIC_ECHO_PIN_4 );
//  optical_distance = measure_optical_distance();
//  euler_angles = imu_sensor.getVector( Adafruit_BNO055::VECTOR_EULER );

 // thrust_command = 0;   // 0 should be stop, negative values reverse, positive values forward. Values may range from THRUST_CMD_MIN and THRUST_CMD_MAX
 // steering_command = 0; // 0 is straight ahead, negative values left, positive values right. Values may range from STEERING_CMD_MIN to STEERING_CMD_MAX


  // this is what actually updates the servo controller and changes the hardware value
//  if (optical_distance > safetyDistance)
//  {
//    thrust_command = 30;
//    steering_command = 0;
//    
//  }
//  else if (optical_distance <= safetyDistance)
//  {
//    steering_command = 15;
//    thrust_command = 15;
//
//    t = time();
//    is_turning = true;
//    
//    
//  }
//
//  if ( ( is_turning ) && ( t > 5) )
//  {
//
//    steering_command = -15
//    thrust_command = 15;
//
//    is_turning_back = true;
//  }


  
   // this map function takes a variable on a range between THRUST_CMD_MIN and THRUST_CMD_MAX and returns a corresponding value
  // on the range of THRUST_SERVO_MIN and THRUST_SERVO_MAX
  if (millis() - lastPrint > printInterval)
  {
  lastPrint = millis();
  Serial.println("**********************");
  Serial.print("thrust_value = ");
  Serial.println(thrust_value);
  
  Serial.print("thrust_command = ");
  Serial.println(thrust_command);
  
  Serial.print("steering_command = ");
  Serial.println(steering_command);
  
  Serial.print("optical_distance: ");
  Serial.println(optical_distance);
  Serial.print("IMU: ");
  Serial.print("{ theta_x, theta_y, theta_z } = { ");
    // This prints the X angle to the serial terminal
  Serial.print( euler_angles.x() );
    // This prints a comma for the displayed vector
  Serial.print(", ");
    // This prints the Y angle to the serial terminal
    Serial.print( euler_angles.y() );
    // This prints a comma for the displayed vector
    Serial.print(", ");
    // This prints the Z angle to the serial terminal
    Serial.print( euler_angles.z() );
    Serial.println("");
    Serial.print("initial theta: ");
    Serial.println(initial_theta);
    Serial.println("t: ");
    Serial.print(t);
  }
  //thrust_servo.writeMicroseconds(thrust_value);
 // steering_servo.writeMicroseconds(steering_value);

  // As a general good practice, only call functions that will make changes to the hardware, such as the above writeMicrosecond calls, in just one place in the main loop.
  // It usually makes sense to make this happen at the end of the loop, just before the closing }. 
  // This will make any problems much easier to debug than if these calls were sprinkled throughout more complex code.
}

/* Note 1
 *  We could also just put all the numbers directly into the code where needed and the result would be the exact same program uploaded
 *  to the Arduino. The #define lines are to help us humans stay organized and remember what all the values mean, and collect them in 
 *  a place that makes it easier to go back and change if we ever want to. When numbers show up directly in the code 
 *  we call them "magic numbers", because their meaning is obscured, and there is no context for the value (so the joke is that they are magic).
 *  It is generally considered bad practice to use magic numbers because it makes it difficult for someone reading the code to understand the meaning.
 *  That "someone" could very easily be you a couple weeks after you've last looked, and have since forgotten the meaning behind all the magic numbers!
 */

/* Note 2
 *  The map function by itself does not prevent the output value going beyond the minimum or maximum output range. For example
 *  map(x, -50, 50, -100, 100);
 *  Will take the value in x, assumed to be on a range of [-50, 50] and map to [-100, 100]. 
 *  However, if x is greater than 50, or less than -50, map will happily return a value greater than 100, or less than -100.
 *  Wrapping map in constrain like so:
 *  constrain(map(x, -50, 50, -100, 100), -100, 100);
 *  will force the output value to within the bounds set by constrain. When you are just typing in numbers manually this may not seem very useful,
 *  but when values for the commanded thrust or steering are generated by parts of the algorithm, we would like some way of making sure that they do not
 *  exceed the maximum range of the servo values!
 */
 unsigned int measure_ultrasonic_distance( int trigger_pin, int echo_pin )
{

  // This function returns the distance measured by the ultrasonic distance
  // sensors in units of millimeters.

  // This is the number of instances to average the distance measurement over
  int sample_number = 5;

  // This initializes an indexing variable for counting the number of
  // distance measurement samples
  int sample_index;

  // This initializes the distance measurement variable
  unsigned long int echo_duration = 0;

  // This iterates through the distance sample measurements adding the
  // current measurement value to a running sum
  for ( sample_index = 0; sample_index < sample_number; sample_index++ )
  {

    // This sets the trigger pin to low to avoid extraneous noise in the
    // distance measurement
    digitalWrite( trigger_pin, LOW );
    // This delays for a short period to ensure that the pin is fully set
    // to zero
    delayMicroseconds( 2 );
  
    // This sets the trigger pin to high to trigger the ultrasonic sensor
    // to emit the burst of ultrasonic signals
    digitalWrite( trigger_pin, HIGH );
    // This pauses for 10 microseconds to ensure that the ultrasonic sensor
    // fully triggers
    delayMicroseconds( 10 );
    // This sets the trigger pin to low so that the ultrasonic sensor will
    // only trigger a single time
    digitalWrite( trigger_pin, LOW );
  
    // This reads the echo pin and returns the travel time in microseconds
    // and adds it to the running sum
    echo_duration += pulseIn( echo_pin, HIGH );

    // This pauses for 10 milliseconds to ensure that any echos don't
    // contaminate the next measurement
    delay( 10 );

  }

  // This calculates the distance in millimeters by averageing over the
  // measured samples and dividing by twice the speed of sound
  unsigned int echo_distance = ( ( float ) echo_duration / ( float ) sample_number ) / 5.9;

  // This returns the distance from the ultrasonic sensor
  return echo_distance;
  
}
