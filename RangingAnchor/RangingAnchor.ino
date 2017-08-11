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

#define senderID 0x01

#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define POLL 0 //anchor receives poll
#define POLL_ACK 1 //anchor sends resp
#define RANGE 2 //anchor receives final
#define RANGE_REPORT 3 //anchor sends successful report
#define RANGE_FAILED 255 //or anchor sends failed report
// message flow state
volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
// protocol error state
boolean protocolFailed = false;
// timestamps to remember
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timeRangeReceived;
// last computed range/time
DW1000Time timeComputedRange;

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


#define LEN_DATA sizeof(data)

// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 100;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3500;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

void setup() {
  // DEBUG monitoring
  Serial.begin(115200);
  delay(1000);
  // Serial.println(F("### DW1000-arduino-ranging-anchor ###"));
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  // Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(senderID);
  DW1000.setNetworkId(10);
  //DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.commitConfiguration();


  
  //see 7.2.31 and 7.2.31.1  in user manual
  // 000111111 or 0x3F for max power 
  byte txpower[LEN_TX_POWER];
  DW1000.writeValueToBytes(txpower, 0x3F3F3F3FL, LEN_TX_POWER);
  DW1000.writeBytes(TX_POWER, NO_SUB, txpower, LEN_TX_POWER);

  // Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  /* char msg[128];
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
  // anchor starts in receiving mode, awaiting a ranging poll message
  receiver();
  noteActivity();
  // for first time ranging frequency computation
  rangingCountPeriod = millis();
}

byte currSourceID;


void loop() {
  int32_t curMillis = millis();
  if (!sentAck && !receivedAck) {
    // check if inactive
    if (curMillis - lastActivity > resetPeriod) {
      resetInactive();
      Serial.println("TIMEOUT");
    }
  }
  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    // byte msgId = data[0];
    byte msgId = data.msgID;
    if (msgId == POLL_ACK) {
      //DW1000.getTransmitTimestamp(timePollAckSent);
      noteActivity();
    }
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000.getData((byte*)&data, LEN_DATA);
    Serial.println(data.destID);
    if (data.destID == senderID) {

      byte msgId = data.msgID;
      if (msgId != expectedMsgId) {
        // unexpected message, start over again (except if already POLL)
        protocolFailed = true;
      }
      if (msgId == POLL) {
        // on POLL we (re-)start, so no protocol failure
        protocolFailed = false;
        DW1000.getReceiveTimestamp(timePollReceived);
        expectedMsgId = RANGE;

        currSourceID = data.sourceID; //set future destinations based on POLL's source

        transmitPollAck();
        noteActivity();
      }
      else if (msgId == RANGE) {
        DW1000.getReceiveTimestamp(timeRangeReceived);
        expectedMsgId = POLL;
        if (!protocolFailed) {

          // asymmetric two-way ranging (more computation intense, less error prone)
          DW1000Time round1;
          DW1000Time reply2;
          round1.setTimestamp((byte*)&data.time1);
          reply2.setTimestamp((byte*)&data.time2);


          DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
          DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();

          DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
          // set tof timestamp
          timeComputedRange.setTimestamp(tof);


          //transmitRangeReport(timeComputedRange.getAsMicroSeconds());
          float distance = timeComputedRange.getAsMeters();
          // transmitRangeReport(distance);
          transmitRangeReport(timeComputedRange);
          Serial.println(distance, 4);

          /*
            Serial.print("Range: "); Serial.print(distance); Serial.print(" m");
            Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.print(" dBm");
            Serial.print("\t FP Power: "); Serial.print(DW1000.getFirstPathPower()); Serial.print(" dBm");
            Serial.print("\t Quality: "); Serial.print(DW1000.getReceiveQuality());
            Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.print(" Hz");
            Serial.println();
          */
          //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
          //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
          //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
          // update sampling rate (each second)
          successRangingCount++;
          if (curMillis - rangingCountPeriod > 1000) {
            samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
            rangingCountPeriod = curMillis;
            successRangingCount = 0;
          }
        }
        else {
          transmitRangeFailed();
        }

        noteActivity();
      }
    }
  }


}



void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}

void resetInactive() {
  // anchor listens for POLL
  expectedMsgId = POLL;
  receiver();
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

void transmitPollAck() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  //data[0] = POLL_ACK;
  data = {};
  data.sourceID = senderID;
  data.destID = currSourceID;
  data.msgID = POLL_ACK;
  // delay the same amount as ranging tag
  DW1000.setData((byte*)&data, LEN_DATA);
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  timePollAckSent = DW1000.setDelay(deltaTime);
  DW1000.startTransmit();
}

void transmitRangeReport(DW1000Time tof) {
  DW1000.newTransmit();
  DW1000.setDefaults();
  //data[0] = RANGE_REPORT;
  data = {};
  data.sourceID = senderID;
  data.destID = currSourceID;
  data.msgID = RANGE_REPORT;
  // write final ranging result
  //memcpy((byte*)&data + 1, &curRange, 4);
  tof.getTimestamp((byte*)&data.time1);
  DW1000.setData((byte*)&data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRangeFailed() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  //data[0] = RANGE_FAILED;
  data = {};
  data.sourceID = senderID;
  data.destID = currSourceID;
  data.msgID = RANGE_FAILED;
  DW1000.setData((byte*)&data, LEN_DATA);
  DW1000.startTransmit();
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  // so we don't need to restart the receiver manually
  DW1000.receivePermanently(true);
  DW1000.startReceive();
}

