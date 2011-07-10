
#include <Timer.h> 
#include "CarChat.h"
// #include "printf.h"


module CarChatC {
  uses {
    interface Boot;
    // interfaces for sending, receiving ping packets (receiver shared with other types of messages)
    interface AMSend as SendPingMsg;
    interface Receive as ReceivePing;
    interface Packet as Packet1;

    // interface for receiving infrastructure packets
    interface Receive as ReceiveInfr;

    // interfaces for infrastructure mode
    interface AMSend as SendInfrMsg;
    interface Packet as Packet2;
    interface Timer<TMilli> as InfrTimer;

    interface SplitControl as AMControl; 
    interface Timer<TMilli> as PingTimer;
    interface Timer<TMilli> as LiveZoneExitTimer;
    interface Leds;

    // for RSSI read
#ifdef SIM_MODE
    interface TossimPacket;
#else
    interface CC2420Packet;
#endif

#ifdef LOGGER_ON
    interface LogWrite;
#endif
  }
}

implementation {
  // variables used for vehicular nodes
  message_t PingPkt;
  nx_uint8_t no_ping;	// counts pings sent by this node
  nx_uint8_t number_pings; // counts pings received in general
  nx_int16_t rssi_value; // signal strength reading

  dataItem mem_data;	// as long as one single data type will be used for testing, with ID "TEST_ID"

  infrItem infr_data;

#ifdef LOGGER_ON
  logInput log_buf;	// save aggregate log data, which will all be written at once
  logLine log_line;
  nx_uint8_t log_idx;
#endif
  bool mote_busy;
  
  // variables used for infrastructure nodes
  message_t InfrPkt;
  bool radio_busy;

  nx_uint8_t state; // state in protocol description - Live Zone, Dead Zone Quiescent, Dead Zone Active

  // update state of node
  void ChangeState(nx_uint8_t new_state)
  {
    if(new_state == LIVEZ)
    {
      // toggle green LED to indicate message received from Infrastructure
      call Leds.led1Toggle();
      
      if(state != LIVEZ) { // if entering live zone, light up red LED
        dbg("CarChat"," ... entering Live Zone state\n");
        if(state == DEADZ_Q) {
          call PingTimer.stop();
          // stop PingEval timer as well (in case car was in the process of detecting new cars)
        }
        // set state to LIVEZ
        state = LIVEZ;    
        // turn on RED LED to signify in live zone
        call Leds.led0On();
      }
      else {  // if already in live zone, reset timeout timer
        dbg("CarChat"," ... though already in Live Zone state\n");
        call LiveZoneExitTimer.stop();
      }

      // run one time shot for timeout
      call LiveZoneExitTimer.startOneShot(INFR_TIMEOUT*1024);
    }
    else if(new_state == DEADZ_A)
    {
      // turn on BLUE LED to signify dead zone active
      // if coming from adv, set random backoff timer for request
      // else, send adv with metadata of state
      // set state to DEADZ_A
    }
    else if(new_state == DEADZ_Q)
    {
      // clear out all data about previous pings

      // reset PING recording fields
      number_pings = 0;

      // turn off all LEDs
      call Leds.led0Off();
      call Leds.led1Off();
      call Leds.led2Off();
      // set periodic PING timer
      call PingTimer.startPeriodic(PING_PER); 
      // set state to DEADZ_Q
      state = DEADZ_Q;
    }
    else if(new_state == INFR_NODE)
    {
       call InfrTimer.startOneShot(INFRMSG_PER); 
       dbg("InfrChat","Started infrastructure timer\n");

       // initialize dummy data for infrastructure node to hold (for dissemination)
       infr_data.totalPack = 10;
       infr_data.complData.dataID = 0xDA;
       infr_data.complData.vNum = 1;
       infr_data.complData.sourceAddr = (uint16_t)TOS_NODE_ID;
       infr_data.complData.destAddr = 0xFFFF;	// broadcast address
       infr_data.complData.pNum = 0;       // this will get incremented after each transmission, cycle on values 0..9

    }
  }

#ifdef LOGGER_ON
  void AddToLog(logLine newEntry) {
    // update all entries in log array
    log_buf.no_pings[log_idx] = newEntry.no_pings;
    log_buf.sourceAddr[log_idx] = newEntry.sourceAddr;
    log_buf.sig_val[log_idx] = newEntry.sig_val;
    log_buf.vNum[log_idx] = newEntry.vNum;
    log_buf.pNum[log_idx] = newEntry.pNum;

    log_idx = log_idx + 1;

    if(log_idx == LOG_MAX && mote_busy == FALSE) {
      mote_busy = TRUE;
      if (call LogWrite.append(&log_buf, sizeof(logInput)) != SUCCESS) {
      }
      log_idx = 0;
      mote_busy = FALSE;
    }
  }
#endif


  // after receiving a data message either from the infrastructure or from another node, see where it fits in
  void updateData(dataMsg newData) {

    if(mem_data.incomData.dataID == 0) { // started with a blank slate, initialize dataID and dType in memory
      mem_data.incomData.dataID = newData.dataID;
      mem_data.incomData.dType = newData.dType;
      mem_data.complData.dataID = newData.dataID;
      mem_data.complData.dType = newData.dType;
      mem_data.totalPack = newData.tPack;
    }
  
    if(newData.dataID != mem_data.incomData.dataID || newData.dType != mem_data.incomData.dType) {
      // error, stray data type in system
      dbg("CarChat","ERROR, unknown data type received");
    }
    else {
      if(newData.vNum > mem_data.incomData.vNum) { // received packet from new data version, reset partial version
        mem_data.incomData.vNum = newData.vNum;
        mem_data.incomData.pNum = 0;  // don't yet have any packets from this version number
      }
   
      if(newData.vNum == mem_data.incomData.vNum && newData.pNum == mem_data.incomData.pNum) { //next packet received
        mem_data.incomData.pNum++;
      }
      else { // packet out of order, discard
      }

      if(mem_data.incomData.pNum == mem_data.totalPack) { // incomplete version has in fact become complete, copy to complete version
        mem_data.complData.vNum = mem_data.incomData.vNum;
      }

      dbg("CarChat","Most recent data status is version %d in complete and version %d, packet %d in incomplete\n", 
          mem_data.complData.vNum, mem_data.incomData.vNum, mem_data.incomData.pNum);
    }
  }

  event void Boot.booted() {
    // global counter of pings transmitted (overflow not important, still helps with global ordering)
    no_ping = 0;
    // upon booting, first start radio 
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {

    if (err == SUCCESS) {	// radio successfully initiated

      if((uint16_t)TOS_NODE_ID < MAX_NODES) // this node is a vehicular node
      {

        #ifdef LOGGER_ON
          log_idx = 0;
        #endif

        // initiate DeadZoneQuiescent mode
        mote_busy = TRUE;
        atomic {
          ChangeState(DEADZ_Q);
        }
        mote_busy = FALSE;

        dbg("CarChat","Started mote in Dead Zone Quiescent Mode\n");
      }
      else { // this node should operate as an infrastructure node
        mote_busy = TRUE;
        atomic {
          ChangeState(INFR_NODE);
        }
        mote_busy = FALSE;

      }
    }
    else {
      call AMControl.start();
    }

  }

  event void AMControl.stopDone(error_t err) {
  }

  event void PingTimer.fired()
  {
    chatMsg *pChatMsg = (chatMsg*)(call Packet1.getPayload(&PingPkt,sizeof(chatMsg)));

    dbg("CarChat","Timer went off, sending PING message %d\n", no_ping); 
    
    call Leds.led0Toggle();

    pChatMsg->vNum = 0;  // ping message simply contains a ver. 0 advertisement
    pChatMsg->sourceAddr = (uint16_t)TOS_NODE_ID;
    pChatMsg->no_ping = no_ping;
 
    no_ping++;

    // send Ping including source ID for car identification
    if(call SendPingMsg.send(AM_BROADCAST_ADDR,&PingPkt,sizeof(chatMsg))==FAIL){ }
    else { }
  }

  event void LiveZoneExitTimer.fired()
  {
    // when live zone times out, time to switch back to Dead Zone Quiescent state
    dbg("CarChat","Timed out on live zone, starting to send PING messages\n"); 
    ChangeState(DEADZ_Q);
  }


  event message_t* ReceivePing.receive(message_t* msg, void* payload, uint8_t len) {

    if( (uint16_t)TOS_NODE_ID < MAX_NODES ) {

      if(state == DEADZ_Q) {	// ignore pings received while in other states

        // safer way to obtain message payload
        chatMsg *rxMsg = (chatMsg*)(call Packet1.getPayload(msg,sizeof(chatMsg)));

        dbg("CarChat","Received ping message %d from %d - checking for version number\n", rxMsg->no_ping, rxMsg->sourceAddr);
 
        #ifdef SIM_MODE
        rssi_value = (int16_t)(call TossimPacket.strength(msg));
        #else
        rssi_value = (uint16_t)(call CC2420Packet.getRssi(msg));
        #endif

        if(rxMsg->vNum == 0 && !mote_busy) {             // ping signal received, print RSSI reading and log (if logging is on)
          call Leds.led1Toggle();

          dbg("CarChat","Message has version 0, counting PING with RSSI %d\n", rssi_value);
   
          // here, record new ping, and start recording timer if appropriate (new function?)
          number_pings = number_pings + 1;
  
          #ifdef LOGGER_ON
          
          log_line.no_pings = rxMsg->no_ping;
          log_line.sourceAddr = rxMsg->sourceAddr;
          log_line.sig_val = rssi_value;
          log_line.vNum = 0;
          log_line.pNum = 0;
          log_line.sourceType = 1;

	  AddToLog(log_line);

          #endif

        } // end if(rxMsg->vNum == 0 && !mote_busy)

     } // end if(state == DEADZ_Q)
     else {
       dbg("CarChat","Received ping while in %d state, ignoring\n",state);
     } // other states, vehicular node

    } // end if( (uint16_t)TOS_NODE_ID < MAX_NODES ) 
    return msg;
  }
  

  event message_t* ReceiveInfr.receive(message_t* msg, void* payload, uint8_t len) {
    
    if( (uint16_t)TOS_NODE_ID < MAX_NODES ) {  // only process infrastructure message if node is vehicular
 
      // safer way to obtain message payload
      dataMsg *rxMsg = (dataMsg*)(call Packet2.getPayload(msg,sizeof(dataMsg)));

      dbg("CarChat","Received infrastructure message, data ID %d (type %d), version number %d, packet %d of %d\n", 
          rxMsg->dataID, rxMsg->dType, rxMsg->vNum, rxMsg->pNum, rxMsg->tPack);

      atomic {
          ChangeState(LIVEZ);
        }

      // save information on infrastructure data received
      #ifdef LOGGER_ON
          
      log_line.no_pings = 0;
      log_line.sourceAddr = rxMsg->sourceAddr;
      log_line.sig_val = ((rxMsg->dataID) << 8) + (rxMsg->dType);	
      log_line.vNum = rxMsg-> vNum;
      log_line.pNum = rxMsg-> pNum;
      log_line.sourceType = 0;

      AddToLog(log_line);

      #endif

      updateData(*rxMsg);

    }
 
    return msg;
  }

  
  event void SendPingMsg.sendDone(message_t* msg, error_t error) {
     call Leds.led0Toggle();
  }

#ifdef LOGGER_ON

  event void LogWrite.eraseDone(error_t err) {
    // nothing needs to be done here
  }



   event void LogWrite.appendDone(void* buf, storage_len_t len, 
                                 bool recordsLost, error_t err) {
    if(err==SUCCESS) {
      call LogWrite.sync();
    }
    else {
      //?
    }
  }

  event void LogWrite.syncDone(error_t err) {
  }

#endif

// ********** CODE FOR MOTE ACTING AS INFRASTRUCTURE **************
  event void InfrTimer.fired()
  {
    dataMsg *pInfrMsg = (dataMsg*)(call Packet2.getPayload( &InfrPkt,sizeof(dataMsg) ));

    dbg("InfrChat","Timer went off, sending infrastructure message sequence\n"); 
    call Leds.led2On();

    pInfrMsg->dataID = infr_data.complData.dataID;
    pInfrMsg->vNum = infr_data.complData.vNum;
    pInfrMsg->sourceAddr = infr_data.complData.sourceAddr;
    pInfrMsg->dType = infr_data.complData.dType;
    pInfrMsg->pNum = infr_data.complData.pNum;
    pInfrMsg->tPack = infr_data.totalPack;

    // send first data packet with source ID = 0 (infrastructure)
    if(call SendInfrMsg.send(AM_BROADCAST_ADDR,&InfrPkt,sizeof(dataMsg))==FAIL){ 
      dbg("InfrErr","Sending Failed\n");
    }
    else { 
      // after successfully broadcasting one packet, move on to next
      infr_data.complData.pNum = (infr_data.complData.pNum + 1) % infr_data.totalPack;
    }
  }

  event void SendInfrMsg.sendDone(message_t* msg, error_t error) {
     call Leds.led2Off();
     dbg("InfrChat", "Sending complete, starting timer again\n");
     call InfrTimer.startOneShot(INFRMSG_PER); 
  }


}
