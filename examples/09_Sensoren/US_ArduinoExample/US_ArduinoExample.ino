#define TRIGGER_PIN  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     7  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define SPEED        0.0333
// 333 m/s = 33300 cm/s = 33.300 cm/ms = 0.0333 cm/mus

const int ledPin =  13;      // the number of the LED pin

unsigned long getDuration(int tPin,int ePin){
  // Run-time measurment between activation of tPin and ePin
  // Used for ultrasonic measurements here.
  // returns Duration in [ns]
  digitalWrite(tPin, LOW);  // Reset the trigger pin.
  delayMicroseconds(2); 
  digitalWrite(tPin, HIGH);  // Start a measurement.
  delayMicroseconds(10); // 
  digitalWrite(tPin, LOW);   // Complete the pulse.
  // https://www.arduino.cc/reference/de/language/functions/advanced-io/pulsein/
  return pulseIn(ePin, HIGH);  // Wait for a reflection pulse [ms]
}

void setup() {
  Serial.begin(9600); 
  analogReadResolution(10);
  pinMode(ledPin, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.println("Los gehts!");
}
 
void loop() {
  long duration = getDuration(TRIGGER_PIN, ECHO_PIN);
  Serial.print((float)duration/2,2);
  Serial.print(" ");
  Serial.println((float)duration/2*SPEED,2);
  delay(50);                     
}
