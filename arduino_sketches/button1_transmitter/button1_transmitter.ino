//Transmitter1 coding for the nrf24L01 radio transceiver.
//https://www.electroniclinic.com/
//editted by Gizem

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define CE_PIN 9
#define CSN_PIN 10
#define LED_PIN 8

struct PayloadStruct
{
  uint8_t buttonID;
  uint8_t buttonState;
};
PayloadStruct payload;


const uint64_t address = 0xB3B4B5B6F1LL;

RF24 radio(CE_PIN, CSN_PIN);
int data[2]; // depending on the number of sensors you want to use
const uint8_t button_pin = 2; //vresistor define as interrupt later
int button_val;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }
  Serial.println("NRF transmitter-1 is set");
  radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(payload));

  payload.buttonID = 1;
  payload.buttonState = false;

  radio.stopListening(); // put radio in TX mode
  radio.openWritingPipe(address);

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

  pinMode(button_pin, INPUT);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // read the value at analog input
  payload.buttonState = digitalRead(button_pin);
  digitalWrite(LED_PIN, !payload.buttonState);

  unsigned long start_timer = micros();                    // start the timer
  bool report = radio.write(&payload, sizeof(payload));    // transmit & save the report
  unsigned long end_timer = micros();  

  if (report) {
      // payload was delivered

      Serial.print(F("Transmission of buttonID "));
      Serial.print(payload.buttonID);                       // print payloadID
      Serial.print(F(":"));
      Serial.print(payload.buttonState);                          // print nodeID
      Serial.print(F(" successful!    "));
      Serial.print(F(" Time to transmit: "));
      Serial.print(end_timer - start_timer);                 // print the timer result
      Serial.println(F(" us"));
    } else {
      Serial.println(F("Transmission failed or timed out")); // payload was not delivered
    }                                 // increment payload number

    // to make this example readable in the serial monitor
    delay(500); // slow transmissions down by 1 second


  //  if(button_val)
  //    Serial.println("pressed-1");
  //
  //  data[0] = 231;  // code to identify the transmitter.
  //  data[1] = button_val;
  //  radio.write( data, sizeof(data) );
}
