/* 
 * Button Example for Rosserial
 */

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

#define CE_PIN 9
#define CSN_PIN 10
#define TX_AMOUNT 2


ros::NodeHandle nh;

std_msgs::Int8 button1_msg;
ros::Publisher pub_button1("button1_state", &button1_msg);
std_msgs::Int8 button2_msg;
ros::Publisher pub_button2("button2_state", &button2_msg);

const int button1_led_pin = 3;
const int button2_led_pin = 4;

struct PayloadStruct
{
  uint8_t buttonID;
  uint8_t buttonState;
};
PayloadStruct payload;

const uint64_t address[6] = {0x7878787878LL,
                       0xB3B4B5B6F1LL,
                       0xB3B4B5B6CDLL,
                       0xB3B4B5B6A3LL,
                       0xB3B4B5B60FLL,
                       0xB3B4B5B605LL
                      };

RF24 radio(CE_PIN, CSN_PIN);

void setup()
{
  nh.initNode();
  nh.advertise(pub_button1);
  nh.advertise(pub_button2);

  payload.buttonState = 5;
  
  pinMode(button1_led_pin, OUTPUT);
  pinMode(button2_led_pin, OUTPUT);

  digitalWrite(button1_led_pin, LOW);
  digitalWrite(button2_led_pin, LOW);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(payload));
  for (uint8_t i = 0; i < TX_AMOUNT; ++i)
      radio.openReadingPipe(i, address[i]);

  radio.startListening(); // put radio in RX mode
 
}

void loop()
{
  
  //bool reading = ! digitalRead(button_pin);
  
  uint8_t pipe;
    if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
      radio.read(&payload, bytes);            // fetch payload from FIFO

      if(payload.buttonID == 1){
        digitalWrite(button1_led_pin, HIGH);
        button1_msg.data = payload.buttonState;
        delay(100);
      }
      else if(payload.buttonID == 2){
        digitalWrite(button2_led_pin, HIGH);
        button2_msg.data = payload.buttonState;
        delay(100);
      }
      else{
      digitalWrite(button1_led_pin, LOW);
      digitalWrite(button2_led_pin, LOW);
      }
//      Serial.print(F("Received "));
//      Serial.print(bytes);                    // print the size of the payload
//      Serial.print(F(" bytes on pipe "));
//      Serial.print(pipe);                     // print the pipe number
//      Serial.print(F(" RECEIVED!   Button: "));
//      Serial.print(payload.buttonID);           // print the payload's origin
//      Serial.print(F("   State: "));
//      Serial.println(payload.buttonState);      // print the payload's number
    }
    else{
      digitalWrite(button1_led_pin, LOW);
      digitalWrite(button2_led_pin, LOW);
    }

  pub_button1.publish(&button1_msg);
  pub_button2.publish(&button2_msg);
  
  nh.spinOnce();
  delay(100);
}
