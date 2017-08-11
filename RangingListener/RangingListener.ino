#define senderID 0x05

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0 //tag sends poll
#define POLL_ACK 1 //tag receives resp
#define RANGE    2 //tag sends final
#define RANGE_REPORT 3 //tag receives successful report
#define RANGE_FAILED 255 //or tag receives failed report
#define expectedMsgId RANGE_FAILED
volatile boolean received = false;

struct Distance {
  byte sourceID;
  byte destID;
  byte msgID;
  uint16_t distance[4];
} distanceToSend;

#define LEN_DISTANCE_DATA sizeof(distanceToSend)

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  //   Serial.println("DW1000 initialized ...");
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(senderID);
  DW1000.setNetworkId(10);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.commitConfiguration();
  // attach callback for (successfully) sent and received messages
  DW1000.attachReceivedHandler(handleReceived);
  receiver();

}

void loop() {
  if (received) //only proceed if completely received of last msg
  {
//    Serial.println("jello");
    DW1000.getData((byte*)&distanceToSend, sizeof(distanceToSend));
    //only get when msg available
    byte msgId = distanceToSend.msgID;
    if (distanceToSend.destID == senderID)
    {
      if (msgId == expectedMsgId)
      {
        for (int i = 0; i < 3; i++)
        {
          if (distanceToSend.distance[i] != 0xFFFF)
          {
            Serial.print(distanceToSend.distance[i] * 10);
          }
          else
          {
            Serial.print(-1);
          }
          Serial.print(",");
        }

        if (distanceToSend.distance[3] != 0xFFFF)
        {
          Serial.print(distanceToSend.distance[3] * 10);
        }
        else
        {
          Serial.print(-1);
        }

        Serial.println();
      }
    }
    received = false;
  }
}

void handleReceived()
{
  // status change on received success
  received = true;
}

void receiver()
{
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

