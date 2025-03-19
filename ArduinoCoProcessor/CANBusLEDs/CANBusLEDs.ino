// Set the board type to "Arduino Nano Every". It may be necessary to first add
// "Arduino megaAVR Boards" in the board manager.  Do NOT install the Adafruit_VL53L0X
// library or it may conflict with the local copy that contains a correction to be
// compatible with Arduino IDE 2.3.4.

#include "frc_mcp2515.h"
#include "frc_CAN.h"

// FastLED by Daniel Garcia needs to be installed from the library manager.  Tested with version 3.9.14.
#include <FastLED.h>  

// Define the CAN Bus chipo select pin and the interrupt pin
#define CAN_CS 8
#define CAN_INTERRUPT 2
#define CAN_PACKET_SIZE 8
#define CAN_DEVICE_ID 6       // 6 for elevator (left), 7 for climb (right) !!! UPDATE ME WHEN FLASHING !!! 
#define NUM_LEDS 79           // Elevator (left) 79, Climb (right) 67 !!!!!! LEAVE AT 79 UNLESS YOU ACTUALLY NEED TO CHANGE THIS!!!! 

#define NUM_BLINKS 10 // Number of blinks 
#define LED_UPDATE_PERIOD_MS 104 // use something that isn't a multiple of the 20ms RoboRIO period to reduce conflicts with the LEDs

// Uncomment this line so LED updates will only occur after an CAN Bus packet.  This removes party mode functionality.
//#define IMMEDIATE_UPDATE


// LED Stuff
#define LED_PIN 3 // Using D3
#define HEADLIGHT_PIN 4
CRGB leds[NUM_LEDS];


// Define the time-of-flight (TOF) reset pin
//#define XSHUT 20

// Global variables
unsigned long long lastSendMs = 0; // track how long since we last sent a CAN packet
unsigned int rgb[] = {0,0,0}; // Define the last command that was set by the CANbus communications
unsigned int rgbBlink[] = {255,255,255,0,0,0}; // Blink on and off rgb values
int blinkCounter = NUM_BLINKS; // Number of blinks 

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

  if (apiId == 0) {
    // Turn on/off headlights
    if (data.data[0] == 0) {
      digitalWrite(HEADLIGHT_PIN, LOW);

    } else {
      digitalWrite(HEADLIGHT_PIN, HIGH);
    }

  } else if (apiId == 1) {
    // Set all the LED's to constant color (if 0,0,0 enter party mode)

    rgb[0] = data.data[0];
    rgb[1] = data.data[1];
    rgb[2] = data.data[2];

#ifdef IMMEDIATE_UPDATE
    // Update all of the LED Colours
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = CRGB(rgb[0], rgb[1], rgb[2]);
    }
    // Display the LEDs
    FastLED.show();   
#endif 

  } else if (apiId == 2) {
    // Blink NUM_BLINKS times between [RGB] (0-2) and [RGB] (3-5)

    rgbBlink[0] = data.data[0];
    rgbBlink[1] = data.data[1];
    rgbBlink[2] = data.data[2];
    rgbBlink[3] = data.data[3];
    rgbBlink[4] = data.data[4];
    rgbBlink[5] = data.data[5];

    blinkCounter = NUM_BLINKS;
  }
}

// Callback function for any messages not matching a known device.
// This would still have flags for RTR and Extended set, its a raw ID
void UnknownMessageCallback(uint32_t id, const frc::CANData& data) {

}


void setup() {
    // Begin serial port
    Serial.begin(115200);

    // Define LED outputs
    pinMode(HEADLIGHT_PIN, OUTPUT);

    // Test headlights by cycling three times
    for (int i = 0; i < 3; i++) {
      digitalWrite(HEADLIGHT_PIN, HIGH);
      delay(100);
      digitalWrite(HEADLIGHT_PIN, LOW);
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
    //frc::CAN::SetCANImpl(&mcp2515, CAN_INTERRUPT, CANCallback, UnknownMessageCallback);
    frc::CAN::SetCANImpl(&mcp2515, CAN_INTERRUPT, CANCallback, nullptr);

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
    int j = 0;
    

    // Update must be called every loop in order to receive messages
    frc::CAN::Update();

#ifndef IMMEDIATE_UPDATE

    // Only update the LED's every LED_UPDATE_PERIOD_MS. The LED write routine
    // is intensive and requires the full processor resources to make it work
    // and long LED chains effectively cause commands to be missed so we'll 
    // reduce the number of updates so we get most of the packets. It is therefore
    // preferred if the RoboRIO continuously updates at 20ms intervals.
    if ((millis() - lastMillis) > LED_UPDATE_PERIOD_MS) {
      lastMillis = millis();

      if (blinkCounter > 0) {
        if ((blinkCounter % 2) == 0) {
          // Update all of the LED Colours
          for (j = 0; j < NUM_LEDS; j++) {
            leds[j] = CRGB(rgbBlink[0], rgbBlink[1], rgbBlink[2]);
          }
          
        } else {
          // Update all of the LED Colours
          for (j = 0; j < NUM_LEDS; j++) {
            leds[j] = CRGB(rgbBlink[3], rgbBlink[4], rgbBlink[5]);
          }
        }
        
        // Display the LEDs
        FastLED.show();

        blinkCounter--;      
      

      } else if ((rgb[0] == 0) && (rgb[1] == 0) && (rgb[2] == 0)) { // Party mode if all values are 0
      
        switch (partyState) {
          case 0:

            leds[i] = CRGB(0, 255, 0);
            i++;
            FastLED.show();
            
            
            if (i >= NUM_LEDS) {
              partyState = 1;
              i = 0;
            }
            break;

          case 1:
          
            leds[i] = CRGB(255, 0, 0);
            i++;
            FastLED.show();
            
            
            if (i >= NUM_LEDS) {
              partyState = 2;
              i = 0;
            }
            
            break;

          case 2:

            leds[i] = CRGB(0, 0, 255);
            i++;
            FastLED.show();

            if (i >= NUM_LEDS) {
              partyState = 0;
              i = 0;
            }
            break;

          default:
            partyState = 0;
        }
      

      } else { 
        // Update all of the LED Colours
        for (j = 0; j < NUM_LEDS; j++) {
          leds[j] = CRGB(rgb[0], rgb[1], rgb[2]);
        }
        // Display the LEDs
        FastLED.show();
      }
    }
#endif
}
