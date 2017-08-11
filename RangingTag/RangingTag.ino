/*
  this two-way ranging algorithm is asymetric but sends range report seperately.
  1. TAG sends POLL
  2. ANCHOR receives POLL
  3. ANCHOR sends POLL_ACK
  4. TAG receives POLL ACK
  5. TAG sends RANGE
  6. ANCHOR receives RANGE
  7. ANCHOR sends either RANGE_REPORT or RANGE_FAILED
  8. TAG receives either RANGE_REPORT or RANGE_FAILED

  it seems timestamp increments every 1/64th of a nanosecond.
  timestamp size is 5 byte (40bits), counts up to 1099511627775 (2^40-1)
  system timestamp overflows every 17.179869184 seconds (2^40 * (1/64)*10^-9)

*/

#define senderID 0x00

#include <SPI.h>
#include <DW1000.h>

#include <Filters.h>

float filterFrequency = 0.25;
//float filterFrequency = 0.05; //to apply heavy averaging

/*

  for 0x01
  data set without correcting zero error
  meter nanosesconds
  30    2.67
  60    4.07
  90    5.28
  120   6.56
  150   7.74
  180   9.03
  210   10.20
  240   11.41
  270   12.39
  300   13.25
  330   14.40

  linear trendline from excel:
  distance = 25.542(timeNS) - 45.23


  for 0x02
  data set without correcting zero error
  meter nanosesconds
  390   16.35
  360   15.31
  330   14.21
  300   12.80
  270   11.77
  210   10.84
  210   9.77
  180   8.85
  150   7.61
  120   6.42
  90    5.02
  60    3.87
  30    2.67

  linear trendline from excel: (point at 300 excluded)
  distance = 26.43(timeNS) - 45.699



  for 0x03
  data set without correcting zero error
  meter nanosesconds
  390   16.90
  360   15.37
  330   14.17
  300   12.75
  270   12.10
  210   10.95
  210   9.60
  180   8.62
  150   7.41
  120   6.07
  90    4.86
  60    3.64
  30    2.29

  linear trendline from excel: (point at 300 excluded)
  distance = 25.178(timeNS) - 32.448


  for 0x04
  data set without correcting zero error
  meter nanosesconds
  390   16.90
  360   15.27
  330   14.06
  300   12.48
  270   12.15
  210   10.82
  210   9.55
  180   8.75
  150   7.45
  120   6.07
  90    4.92
  60    3.72
  30    2.44

  linear trendline from excel:
  distance = 25.499(timeNS) - 35.706



*/
// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0 //tag sends poll
#define POLL_ACK 1 //tag receives resp
#define RANGE 2 //tag sends final
#define RANGE_REPORT 3 //tag receives successful report
#define RANGE_FAILED 255 //or tag receives failed report
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;

// data buffer
/*#define LEN_DATA 16
  byte data[LEN_DATA];*/

struct Data {
  byte sourceID;
  byte destID;
  byte msgID;
  byte time1[5]; //size of timestamp is 5 bytes (40bits)
  byte time2[5];
} data;

struct Distance {
  byte sourceID;
  byte destID;
  byte msgID;
  uint16_t distance[4];
} distanceToSend;

uint16_t tempDistance[4] = {};


#define LEN_DATA sizeof(data)
#define LEN_DISTANCE_DATA sizeof(distanceToSend)
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 15;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3500;

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  //   Serial.println(F("### DW1000-arduino-ranging-tag ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  //   Serial.println("DW1000 initialized ...");
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(senderID);
  DW1000.setNetworkId(10);
  //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.commitConfiguration();
  //  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  /*char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000.getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000.getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000.getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
  */
  // attach callback for (successfully) sent and received messages
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  // anchor starts by transmitting a POLL message
  receiver();
  transmitPoll();
  noteActivity();
}


#define numOfAnchors 4

byte anchorID[numOfAnchors] = {0x01, 0x02, 0x03, 0x04};
byte anchorIndex = 0;

byte currDestID = anchorID[anchorIndex]; //initialise destination as first anchor

float distance[numOfAnchors] = { -1, -1, -1, -1}; //initialise as invalid

FilterOnePole lowpassFilter0( LOWPASS, filterFrequency );
FilterOnePole lowpassFilter1( LOWPASS, filterFrequency );
FilterOnePole lowpassFilter2( LOWPASS, filterFrequency );
FilterOnePole lowpassFilter3( LOWPASS, filterFrequency );
//FilterTwoPole lowpassFilter( LOWPASS, filterFrequency );
#define delayTime 2000

void loop() {
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (millis() - lastActivity > resetPeriod) {
      nextAnchor();
      resetInactive();
      //   Serial.println("TIMEOUT");
    }
    return;
  }

  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = data.msgID;
    if (msgId == POLL) {
      DW1000.getTransmitTimestamp(timePollSent);
      //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
    } else if (msgId == RANGE) {
      DW1000.getTransmitTimestamp(timeRangeSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData((byte*)&data, LEN_DATA);

    if (data.destID == senderID) {

      byte msgId = data.msgID;
      if (msgId != expectedMsgId) {
        // unexpected message, start over again
        //Serial.print("Received wrong message # "); Serial.println(msgId);
        expectedMsgId = POLL_ACK;
        transmitPoll();
        return;
      }
      if (msgId == POLL_ACK) {
        DW1000.getReceiveTimestamp(timePollAckReceived);
        expectedMsgId = RANGE_REPORT;
        transmitRange();
        noteActivity();
      } else if (msgId == RANGE_REPORT) {
        expectedMsgId = POLL_ACK;
        /* float distance;
          memcpy(&distance, data + 1, 4);
          Serial.println(distance);*/


        DW1000Time tof;
        tof.setTimestamp(data.time1);

        // distance[anchorIndex] = tof.getAsMeters();

        //calibration for each anchor
        if (anchorID[anchorIndex] == 0x01) {
          distance[anchorIndex] = 25.542 * (tof.getAsMicroSeconds() * 1000) - 45.23;
          lowpassFilter0.input(distance[anchorIndex]);
        } else if (anchorID[anchorIndex] == 0x02) {
          distance[anchorIndex] = 26.43 * (tof.getAsMicroSeconds() * 1000) - 45.699;
          lowpassFilter1.input(distance[anchorIndex]);
        } else if (anchorID[anchorIndex] == 0x03) {
          distance[anchorIndex] = 25.178 * (tof.getAsMicroSeconds() * 1000) - 32.448;
          lowpassFilter2.input(distance[anchorIndex]);
        } else if (anchorID[anchorIndex] == 0x04) {
          distance[anchorIndex] = 25.499 * (tof.getAsMicroSeconds() * 1000) - 35.706;
          lowpassFilter3.input(distance[anchorIndex]);
        }



        nextAnchor();
        /*
            Serial.print("dest: ");
            Serial.print(data.destID);

            Serial.print("\t source: ");
            Serial.print(data.sourceID);
            Serial.print("\t distance: ");

            Serial.print(distance[anchorIndex], 4);
            Serial.println();
        */

        /*   Serial.print(distance[0], 4);
           for (byte i = 1; i < numOfAnchors; i++) {
             Serial.print(',');
             Serial.print(distance[i], 4);
           }
           Serial.println();*/

        transmitPoll();
        noteActivity();
      } else if (msgId == RANGE_FAILED) {
        nextAnchor();
        expectedMsgId = POLL_ACK;
        transmitPoll();
        noteActivity();
      }
    }
  }


  //  delay(delayTime);
}

void sendDistance()
{

  DW1000.newTransmit();
  DW1000.setDefaults();
  distanceToSend = {};
  distanceToSend.sourceID = senderID; //mine
  distanceToSend.destID = 0x05; //only one receiver now
  distanceToSend.msgID = RANGE_FAILED; //enum ID, just a number

  for (int i = 0; i < 4; i++)
  {
    distanceToSend.distance[i] = tempDistance[i];
    //    Serial.print((float)distanceToSend.distance[i]/10, 4);
    //    Serial.print(",");
  }
  //  Serial.println();

  DW1000.setData((byte*)&distanceToSend, LEN_DISTANCE_DATA);
  DW1000.startTransmit();
}

//#define sampleCounter 10
int enterCounter = 0;
void nextAnchor() {
  if (anchorIndex < numOfAnchors - 1) {
    anchorIndex++;
  } else {
    anchorIndex = 0;
    enterCounter++;
    /*Serial.print(distance[0], 4);
      distance[0] = -1;
      for (byte i = 1; i < numOfAnchors; i++) {
      Serial.print(',');
      Serial.print(distance[i], 4); //print first
      distance[i] = -1; //then set invalid value
      }
      Serial.println();*/

    if (distance[0] != -1) {
      tempDistance[0] =  lowpassFilter0.output() / 10;
      Serial.print((float)lowpassFilter0.output(), 4);
    } else {
      tempDistance[0] = 0xFFFF;
      Serial.print((float)(-1),4);
    }
    Serial.print(',');

    if (distance[1] != -1)  {
      tempDistance[1] =  lowpassFilter1.output() / 10;
      Serial.print((float)lowpassFilter1.output(), 4);
    } else {
      tempDistance[1] = 0xFFFF;
      Serial.print((float)(-1),4);
    }
    Serial.print(',');

    if (distance[2] != -1)  {
      tempDistance[2] =  lowpassFilter2.output() / 10;
      Serial.print((float)lowpassFilter2.output(), 4);
      //      Serial.print(lowpassFilter2.output(), 4);
    } else {
      tempDistance[2] = 0xFFFF;
      Serial.print((float)(-1),4);
    }
    Serial.print(',');

    if (distance[3] != -1)  {
      tempDistance[3] =  lowpassFilter3.output() / 10;
      Serial.print((float)lowpassFilter3.output(), 4);
      //      Serial.print(lowpassFilter3.output(), 4);
    } else {
      tempDistance[3] = 0xFFFF;
      Serial.print((float)(-1),4);
    }
    Serial.println();

//    Serial.println(enterCounter);
//    if (enterCounter == sampleCounter)
//    {
//      Serial.println("HELLO");
//      enterCounter = 0;
      sentAck = false;
      sendDistance();
      while (!sentAck);
//    }


    for (byte i = 0; i < numOfAnchors; i++) {
      distance[i] = -1; //set invalid value, until updated with real value
    }

  }
  currDestID = anchorID[anchorIndex];
  // Serial.println(currDestID, HEX);
}



void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {
  // tag sends POLL and listens for POLL_ACK
  transmitPoll();
  expectedMsgId = POLL_ACK;
  noteActivity();
}

void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on received success
  receivedAck = true;
}

void transmitPoll() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data = {};
  data.sourceID = senderID;
  data.destID = currDestID;
  data.msgID = POLL;
  DW1000.setData((byte*)&data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRange() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data = {};
  data.sourceID = senderID;
  data.destID = currDestID;
  data.msgID = RANGE;
  // delay sending the message and remember expected future sent timestamp
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);


  DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
  round1.getTimestamp(data.time1);

  timeRangeSent = DW1000.setDelay(deltaTime);
  DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
  reply2.getTimestamp(data.time2);

  DW1000.setData((byte*)&data, LEN_DATA);
  DW1000.startTransmit();
  //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

