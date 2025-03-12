// Set the board type to "Arduino Nano Every". It may be necessary to first add
// "Arduino megaAVR Boards" in the board manager.  Do NOT install the Adafruit_VL53L0X
// library or it may conflict with the local copy that contains a correction to be
// compatible with Arduino IDE 2.3.4.

#include "frc_mcp2515.h"
#include "frc_CAN.h"
#include "Adafruit_VL53L0X.h"
#include <FastLED.h>  // This library needs to be installed 

//Adafruit_VL53L0X range_sensor = Adafruit_VL53L0X();

// Define the CAN Bus chipo select pin and the interrupt pin
#define CAN_CS 8
#define CAN_INTERRUPT 2
#define CAN_PERIOD_MS 100
#define CAN_PACKET_SIZE 8
#define CAN_DEVICE_API 0x123  
#define CAN_DEVICE_ID 6       // 6 for elevator (left), 7 for climb (right) !!! UPDATE ME WHEN FLASHING !!! 
#define NUM_LEDS 79           // Elevator (left) 79, Climb (right) 67 !!!!!! LEAVE AT 79 UNLESS YOU ACTUALLY NEED TO CHANGE THIS!!!! 


// LED Stuff
#define LED_PIN 3 // Using D3
int headlights = 4;
CRGB leds[NUM_LEDS];


// Define the time-of-flight (TOF) reset pin
//#define XSHUT 20

// Global variables
unsigned long long lastSendMs = 0; // track how long since we last sent a CAN packet
unsigned int rgb[] = {0,0,0}; // Define the last command that was set by the CANbus communications

// Create an MCP2515 device. Only need to create 1 of these
frc::MCP2515 mcp2515{CAN_CS};

// Create an FRC CAN Device. You can create up to 16 of these in 1 progam
// Any more will overflow a global array.  The device number, manufacturer 
// and devicetype are basically filters for all incoming data.  You will 
// receive all callbacks associated with these and further process it
// based on the API ID and data packets.

frc::CAN frcCANDevice{CAN_DEVICE_ID, frc::CANManufacturer::kTeamUse, frc::CANDeviceType::kMiscellaneous};


// Callback function. This will be called any time a new message is received
// Matching one of the enabled devices.
void CANCallback(frc::CAN* can, int apiId, bool rtr, const frc::CANData& data) {
  /*
    Serial.print("In callback. API: ");
    Serial.print(apiId, HEX);

    Serial.print(" RTR: ");
    Serial.print(rtr);

    Serial.print(" Data: ");

    for (int i = 0; i < data.length; i++) {
      Serial.print(data.data[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");*/

  if (apiId == 0) {
    if (data.data[0] == 0) {
      digitalWrite(headlights, LOW);

    } else {
      digitalWrite(headlights, HIGH);
    }

  } else if (apiId == 1) {
    // Print the first data element 
    Serial.println(data.data[0], HEX);
    // Define the last command with this new data
    //lastCommand = data.data[0];

    rgb[0] = data.data[0];
    rgb[1] = data.data[1];
    rgb[2] = data.data[2];
  }

  // Show that the message has been received 
  //digitalWrite(headlights, HIGH);
}

// Callback function for any messages not matching a known device.
// This would still have flags for RTR and Extended set, its a raw ID
void UnknownMessageCallback(uint32_t id, const frc::CANData& data) {
    /*Serial.print("Unknown message ");
    Serial.print(id & CAN_EFF_MASK, HEX);

    Serial.print(" Data: ");

    for (int i = 0; i < data.length; i++)
    {
      Serial.print(data.data[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");    */
}


void setup() {
    // Begin serial port
    Serial.begin(115200);

    // Define LED outputs
    pinMode(headlights, OUTPUT);

    // Test headlights by cycling three times
    for (int i = 0; i < 3; i++) {
      digitalWrite(headlights, HIGH);
      delay(100);
      digitalWrite(headlights, LOW);
      delay(100);
    }

    // Setup LED strings
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);


    // Initialize the MCP2515. If any error values are set, initialization failed
    auto err = mcp2515.reset();

    // CAN rate must be 1000KBPS to work with the FRC Ecosystem
    // Clock rate must match clock rate of CAN Board.
    err = mcp2515.setBitrate(frc::CAN_1000KBPS, frc::CAN_CLOCK::MCP_16MHZ);

    // Set up to normal CAN mode to allow it to communicate to the RIO. If you
    // just want to monitor the bus, you can use listen only mode.
    err = mcp2515.setNormalMode();
    //err = mcp2515.setListenOnlyMode();

    // Prepare our interrupt pin
    pinMode(CAN_INTERRUPT, INPUT);
    
    // Set up FRC CAN to be able to use the CAN Impl and callbacks
    // Last parameter can be set to nullptr if unknown messages should be skipped
    frc::CAN::SetCANImpl(&mcp2515, CAN_INTERRUPT, CANCallback, UnknownMessageCallback);

    // All CAN Devices must be added to the read list. Otherwise they will not be handled correctly.
    frcCANDevice.AddToReadList();
}



void loop() {
    int16_t distance;
    int16_t analog0;
    uint8_t data[CAN_PACKET_SIZE];
    static int partyState = 0;
    static unsigned long lastMillis = 0;
    static int i = 0;
    

    // Update must be called every loop in order to receive messages
    frc::CAN::Update();

    // Add new LED commands here. Make sure to inform Software when new commands have been added 

    // Party mode if all values are 0
    if ((rgb[0] == 0) && (rgb[1] == 0) && (rgb[2] == 0)) {
      if ((millis() - lastMillis) > 25) {
        lastMillis = millis();

        switch (partyState) {
          case 0:

            leds[i] = CRGB(0, 255, 0);
            FastLED.show();
            i++;
            
            if (i == NUM_LEDS) {
              partyState = 1;
              i = 0;
            }
            break;

          case 1:
          
            leds[i] = CRGB(255, 0, 0);
            FastLED.show();
            i++;
            
            if (i == NUM_LEDS) {
              partyState = 2;
              i = 0;
            }
            
            break;

          case 2:


            leds[i] = CRGB(0, 0, 255);
            FastLED.show();
            i++;

            if (i == NUM_LEDS) {
              partyState = 0;
              i = 0;
            }
            break;

          default:
            partyState = 0;
        }
      }

    } else { 
      // Update all of the LED Colours
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(rgb[0], rgb[1], rgb[2]);
      }
      // Display the LEDs
      FastLED.show();
    }

/*
    if (lastCommand == 0) {                     // Set all LEDs to RGB (default)
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 255, 0);
        FastLED.show();
        delay(25);
      }

      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255, 0, 0);
        FastLED.show();
        delay(25);
      }

      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 255);
        FastLED.show();
        delay(25);
      }

      // Display LEDs
      FastLED.show();

    } else if (lastCommand == 1) {              // Headlights ON
      digitalWrite(headlights, HIGH);

    } else if (lastCommand == 2) {              // Headlights OFF
      digitalWrite(headlights, LOW);

    } else if (lastCommand == 3) {              // LEDs Red
      for (int i = 0; i < NUM_LEDS; i ++) {     
        leds[i] = CRGB(255, 0, 0);
      }

      // Display LEDs
      FastLED.show();

    } else if (lastCommand == 4) {              // LEDs Green
      for (int i = 0; i < NUM_LEDS; i ++) {
        leds[i] = CRGB(0, 255, 0);
      }

      // Display LEDs
      FastLED.show();

    } else if (lastCommand == 5) {              // LEDs Blue
      for (int i = 0; i < NUM_LEDS; i ++) {
        leds[i] = CRGB(0, 0, 255);
      }

      // Display LEDs
      FastLED.show();

    } else if (lastCommand == 6) {              // LEDs Yellow
      for (int i = 0; i < NUM_LEDS; i ++) {
        leds[i] = CRGB(255, 255, 0);
      }

      // Display LEDs
      FastLED.show();

    } else if (lastCommand == 7) {              // LEDs Aqua 
      for (int i = 0; i < NUM_LEDS; i ++) {
        leds[i] = CRGB(0, 255, 255);
      }

      // Display LEDs
      FastLED.show();

    } else if (lastCommand == 8) {              // LEDs Purple 
      for (int i = 0; i < NUM_LEDS; i ++) {
        leds[i] = CRGB(255, 0, 255);
      }

      // Display LEDs
      FastLED.show();

    } else if (lastCommand == 9) {              // LEDs OFF 
      for (int i = 0; i < NUM_LEDS; i ++) {
        leds[i] = CRGB(0, 0, 0);
      }

      // Display LEDs
      FastLED.show();
    }
*/



    // Update all sensors as frequently as possible. This could also include
    // filtering, debounce, or more complex latching logic so the RIO doesn't
    // miss an input if it is reading at a slower rate.data


    /*
    // Update distance on every loop
    if (range_sensor.isRangeComplete()) {
      // Get the intager range value in mm
      distance = range_sensor.readRange();
    }

    // Read the analog IR sensor
    analog0 = analogRead(A0);

    // Writes can happen any time if you want to create a device that is more
    // interrupt driven.  Alternatively you can use a periodic send as shown
    // here.
    auto now = millis();
    if (now - lastSendMs > CAN_PERIOD_MS) {
        lastSendMs += CAN_PERIOD_MS;

        // zero memory buffer (this isn't strictly required, but did it for
        // debugging)
        memset(data, 0, CAN_PACKET_SIZE);

        // Distance sensor MSB, LSB
        data[0] = distance >> 8;
        data[1] = distance & 0xFF;

        // Analog 0 sensor MSB, LSB
        data[2] = analog0 >> 8;
        data[3] = analog0 & 0xFF;


        // Send bytes. The API command is any 10 bit value specific to the device though
        // typically used for commands and configuration. Each packet is limited to up
        // to 8 bytes so you can use multiple API numbers to send larger packets. The
        // API number is arbitrary, but must match what is read on the RIO side.
        frcCANDevice.WritePacket(data, CAN_PACKET_SIZE, CAN_DEVICE_API);
    }
    */
}
