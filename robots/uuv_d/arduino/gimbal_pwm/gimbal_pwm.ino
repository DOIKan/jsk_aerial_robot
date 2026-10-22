#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

// Change pins 1..3 to the actual wiring. gimbal1 uses the tested pin 9.
const uint8_t kServoPins[4] = {9, 10, 11, 12};
const uint16_t kPulseMinUs = 700;
const uint16_t kPulseMaxUs = 2250;
const unsigned long kCommandTimeoutMs = 500;

ros::NodeHandle nh;
Servo servos[4];
uint16_t appliedPulses[4];
std_msgs::UInt16MultiArray appliedMsg;
ros::Publisher appliedPub("gimbal/pwm_applied", &appliedMsg);
unsigned long lastCommandMs = 0;

void pwmCallback(const std_msgs::UInt16MultiArray& msg) {
  if (msg.data_length != 4) return;
  for (uint8_t i = 0; i < 4; ++i) {
    if (msg.data[i] < kPulseMinUs || msg.data[i] > kPulseMaxUs) return;
  }
  for (uint8_t i = 0; i < 4; ++i) {
    if (!servos[i].attached()) servos[i].attach(kServoPins[i]);
    servos[i].writeMicroseconds(msg.data[i]);
    appliedPulses[i] = msg.data[i];
  }
  lastCommandMs = millis();
  appliedPub.publish(&appliedMsg);
}

ros::Subscriber<std_msgs::UInt16MultiArray> pwmSub("gimbal/pwm_us", pwmCallback);

void setup() {
  appliedMsg.data = appliedPulses;
  appliedMsg.data_length = 4;
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(pwmSub);
  nh.advertise(appliedPub);
  lastCommandMs = millis();
}

void loop() {
  nh.spinOnce();
  if (!nh.connected() || millis() - lastCommandMs > kCommandTimeoutMs) {
    for (uint8_t i = 0; i < 4; ++i) servos[i].detach();
  }
  delay(2);
}
