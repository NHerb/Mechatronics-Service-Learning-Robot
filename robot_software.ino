// Any text that starts with double-slashes (like this line) are comments - lines that don't change the
// robot's behavior but provide useful info on the code

// The sensors and motors are connected to numbered pins according to the LaunchPad's datasheet, but
// We're renaming them so that the code further down is a little easier to read.  It's confusing
// to refer to the motors or sensors by number!
const float FRONT_DISTANCE_TOLERANCE = 10.00;
const float SIDE_DISTANCE_TOLERANCE = 7.00;

const int motor_pin1 = 37;
const int motor_pin2 = 38;
const int motor_pin3 = 39;
const int motor_pin4 = 40;
const int side_sensor = 33;
const int front_sensor = 23;
float side_reading;
float front_reading;
unsigned long previous_action = 0;

// Important numbers that come from the sensor's datasheet.  Used to figure out how far the robot's sensor are from whatever they're sensing.
const float DIST_LOW = (1.0 / 80.0);
const float DIST_HIGH = (1.0 / 10.0);
const float VOLT_LOW = 0.42;
const float VOLT_HIGH = 2.60;

// Function declarations
float convert_reading_to_distance(int reading);
float remap(float value, float input_low, float input_high, float output_low, float output_high);
void update_sensors();

// This function, setup(), runs one time - right when the robot is turned on or immediately after a reset
void setup() {

  // Setting up motor pins are output pins, because we are sending a signal to them.
  pinMode(37, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(39, OUTPUT);
  pinMode(40, OUTPUT);

  // Setting up sensor pins are input, because we are reading data from them
  pinMode(side_sensor, INPUT);
  pinMode(front_sensor, INPUT);
  Serial.begin(115200);
  delay(2000);
  update_sensors();
}



// This function runs over and over until the power is turned off
void loop() {
  if (millis() - previous_action > 41) {
    previous_action = millis();
    float previous_front_reading = front_reading;
    float previous_side_reading = side_reading;
    update_sensors();
    front_reading = (front_reading + previous_front_reading) / 2.0;
    side_reading = (side_reading + previous_side_reading) / 2.0;

    Serial.print("FRONT:  ");
    Serial.print(front_reading);
    Serial.print("\tSIDE:  ");
    Serial.print(side_reading);
    Serial.print("\t\t");


    // The readings are crazy if outside expected boundaries (less than 10cm, greater than 80cm)
    // So let's just cap invalid results to max sensor range
    if (front_reading < 0 or front_reading > 30.00)
      front_reading = 30.00;
    if (side_reading < 0 or side_reading > 30.00)
      side_reading = 30.00;


    Serial.print("FRONT:  ");
    Serial.print(front_reading);
    Serial.print("\tSIDE:  ");
    Serial.print(side_reading);
    Serial.print("\n");

    if (front_reading < FRONT_DISTANCE_TOLERANCE) {
      digitalWrite(motor_pin1, LOW);
      digitalWrite(motor_pin2, HIGH);
      digitalWrite(motor_pin3, HIGH);
      digitalWrite(motor_pin4, LOW);
      int temp = random(0, 2);
      delay(temp * 150 + 160);
    } else {
      digitalWrite(motor_pin1, LOW);
      digitalWrite(motor_pin2, HIGH);
      digitalWrite(motor_pin3, LOW);
      digitalWrite(motor_pin4, HIGH);
      delay(50);
    }

  }
}

// Send in a sensor reading, get a distance in centimeters back
float convert_reading_to_distance(int reading) {
  float result = 0;
  result = reading * (5.0 / 1023.0);
  result = (result - VOLT_LOW) * ((DIST_HIGH - DIST_LOW) / (VOLT_HIGH - VOLT_LOW)) + DIST_LOW;
  result = (1.0 / result);
  result = result * 0.393 * 2.1;
  return result;

}

float remap(float value, float input_low, float input_high, float output_low, float output_high) {
  return ( ( (value - input_low) * (output_high - output_low) / (input_high - input_low)) + output_low );
}

void update_sensors() {

  side_reading = analogRead(side_sensor);
  front_reading = analogRead(front_sensor);

  side_reading = convert_reading_to_distance(side_reading);
  front_reading = convert_reading_to_distance(front_reading);


}

