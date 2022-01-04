#define USE_USBCON

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher pub_range( "range_data", &range_msg);

const int trigger_pin = 6;
const int echo_pin    = 7;
unsigned long range_timer;

char frameid[] = "/us_ranger";

// 333 m/s = 0.333 m/ms = 0.000333 m/mus
const float sonic_speed = 0.000333;

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

void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.3;
  range_msg.min_range = 0.03;
  range_msg.max_range = 1.0;

  pinMode(trigger_pin, OUTPUT);
  pinMode(echo_pin, INPUT);

  //Serial.begin(9600);
}

void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis()-range_timer) > 50){
    long duration = getDuration(trigger_pin, echo_pin);
    float result = ((float)duration)/2*sonic_speed;
    if (result > range_msg.max_range)
      result = 0;
    //Serial.println(int(result * 100));
    range_msg.range = result;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    range_timer =  millis() + 50;
  }
  nh.spinOnce();
}
