// You should not have to change these values unless you alter the PCB in some way.

// This defines the pin number for the first status LED indicator
#define LED_STATUS_PIN_1 40
// This defines the pin number for the second status LED indicator
#define LED_STATUS_PIN_2 48

// This defines the analog read pin for measuring the battery voltage
// from the Vin pin
#define VIN_VOLTAGE_READ_PIN A3

// This defines the trigger pin of the first ultrasonic sensor
#define ULTRASONIC_TRIGGER_PIN_1 3
// This defines the echo pin of the first ultrasonic sensor
#define ULTRASONIC_ECHO_PIN_1 2

// This defines the trigger pin of the second ultrasonic sensor
#define ULTRASONIC_TRIGGER_PIN_2 5
// This defines the echo pin of the second ultrasonic sensor
#define ULTRASONIC_ECHO_PIN_2 4

// This defines the trigger pin of the third ultrasonic sensor
#define ULTRASONIC_TRIGGER_PIN_3 7
// This defines the echo pin of the third ultrasonic sensor
#define ULTRASONIC_ECHO_PIN_3 6

// This defines the trigger pin of the fourth ultrasonic sensor
#define ULTRASONIC_TRIGGER_PIN_4 9
// This defines the echo pin of the fourth ultrasonic sensor
#define ULTRASONIC_ECHO_PIN_4 8

// This defines the pin for reading in the thrust servo signal
// from the RC receiver
#define RC_THRUST_PIN 11
// This defines the pin for reading in the steering servo signal
// from the RC receiver
#define RC_STEERING_PIN 12

// This sets the steering servo pin to 44
#define ARDUINO_STEERING_PIN 44
// This sets the thrust servo pin to 46
#define ARDUINO_THRUST_PIN 46

// This sets the minimum and maximum values in microseconds
// of the steering servo pulse width
#define STEERING_SERVO_MIN 919
#define STEERING_SERVO_MAX 2088
// This sets the minimum and maximum values in microseconds
// of the drive servo pulse width
#define THRUST_SERVO_MIN 1292
#define THRUST_SERVO_MAX 1688
