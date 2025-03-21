// Set the board type to "Arduino Nano Every". It may be necessary to first add
// "Arduino megaAVR Boards" in the board manager.  Do NOT install the Adafruit_VL53L0X
// library or it may conflict with the local copy that contains a correction to be
// compatible with Arduino IDE 2.3.4.

#include "frc_mcp2515.h"
#include "frc_CAN.h"
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X range_sensor = Adafruit_VL53L0X();

// Define the CAN Bus chipo select pin and the interrupt pin
#define CAN_CS 10
#define CAN_INTERRUPT 2
#define CAN_PERIOD_MS 100
#define CAN_PACKET_SIZE 8
#define CAN_DEVICE_ID 4
#define CAN_DEVICE_API 0x123

// Define the time-of-flight (TOF) reset pin
#define XSHUT 20

// Global variables
unsigned long long lastSendMs = 0; // track how long since we last sent a CAN packet

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

    for (int i = 0; i < data.length; i++)
    {
      Serial.print(data.data[i], HEX);
      Serial.print(" ");
    }

    Serial.print("\n");*/
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

    // Reset TOF Sensor by toggling XSHUT pin
    pinMode(XSHUT, OUTPUT);

    digitalWrite(XSHUT, LOW);
    delay(200);
    digitalWrite(XSHUT, HIGH);

    // Initialize range sensor driver
    if (!range_sensor.begin()) {
      // TODO: Add some better error handling here to indicate on the CAN Bus that
      // the sensor is manfunctioning rather than just stopping.
      Serial.print("Failed to boot VL53L0X. Stopping.\n");
      while(1);
    }

    range_sensor.startRangeContinuous();

    
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
    int16_t analog1;
    uint8_t data[CAN_PACKET_SIZE];

    // Update must be called every loop in order to receive messages
    frc::CAN::Update();

    // Update all sensors as frequently as possible. This could also include
    // filtering, debounce, or more complex latching logic so the RIO doesn't
    // miss an input if it is reading at a slower rate.data

    // Update distance on every loop
    if (range_sensor.isRangeComplete()) {
      // Get the intager range value in mm
      distance = range_sensor.readRange();
    }

    // Read the analog IR sensor
    analog0 = analogRead(A0);

    analog1 = analogRead(A1);

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

        data[4] = analog1 >> 8;
        data[5] = analog1 & 0xFF;


        // Send bytes. The API command is any 10 bit value specific to the device though
        // typically used for commands and configuration. Each packet is limited to up
        // to 8 bytes so you can use multiple API numbers to send larger packets. The
        // API number is arbitrary, but must match what is read on the RIO side.
        frcCANDevice.WritePacket(data, CAN_PACKET_SIZE, CAN_DEVICE_API);
    }
}
