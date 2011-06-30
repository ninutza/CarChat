
#include <Timer.h> 
#include "CarChat.h"
// #include "printf.h"


module CarChatC {
  uses {
    interface Boot;
    // interfaces for sending, receiving ping packets (receiver shared with other types of messages)
    interface AMSend as SendPingMsg;
    interface Receive as ReceivePing;
    interface Packet;

    // interface for receiving infrastructure packets
    interface Receive as ReceiveInfr;

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
  message_t PingPkt;
  nx_uint8_t no_ping;	// counts pings sent by this node
  nx_uint8_t number_pings; // counts pings received in general
  nx_int16_t rssi_value; // signal strength reading
#ifdef LOGGER_ON
  logLine log_line;
  nx_uint8_t log_idx;
#endif
  bool mote_busy;
  nx_uint8_t state; // state in protocol description - Live Zone, Dead Zone Quiescent, Dead Zone Active

  void ChangeState(nx_uint8_t new_state)
  {
    if(new_state == LIVEZ)
    {
      // turn on RED LED to signify in live zone
      // run one time shot for timeout
      // set state to LIVEZ
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
      // turn off all LEDs
      call Leds.led0Off();
      call Leds.led1Off();
      call Leds.led2Off();
      // set periodic PING timer
      call PingTimer.startPeriodic(PING_PER); 
      // reset PING recording fields
      number_pings = 0;
      // set state to DEADZ_Q
      state = DEADZ_Q;
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
    else {
      call AMControl.start();
    }

  }

  event void AMControl.stopDone(error_t err) {
  }

  event void PingTimer.fired()
  {
    chatMsg *pChatMsg = (chatMsg*)(call Packet.getPayload(&PingPkt,sizeof(chatMsg)));

    dbg("CarChat","Timer went off, sending PING message\n"); 
    
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
    // safer way to obtain message payload
    chatMsg *rxMsg = (chatMsg*)(call Packet.getPayload(msg,sizeof(chatMsg)));

    //chatMsg* rxMsg = (chatMsg*)payload;
    
    if(state == DEADZ_Q) {	// ignore pings received while in other states

      dbg("CarChat","Received ping message from %d - checking for version number\n", rxMsg->sourceAddr);

      #ifdef SIM_MODE
      rssi_value = (int16_t)(call TossimPacket.strength(msg));
      #else
      rssi_value = 43;
      #endif

      if(rxMsg->vNum == 0 && !mote_busy) {             // ping signal received, print RSSI reading and log (if logging is on)
        call Leds.led1Toggle();

        #ifndef SIM_MODE
        rssi_value = (uint16_t)(call CC2420Packet.getRssi(msg));
	#endif

        dbg("CarChat","Message has version 0, counting PING with RSSI %d\n", rssi_value);
   
        number_pings = number_pings + 1;
  
        #ifdef LOGGER_ON
        log_line.no_pings[log_idx] = rxMsg->no_ping;
        log_line.sourceAddr[log_idx] = rxMsg->sourceAddr;
        log_line.sig_val[log_idx] = rssi_value;
	log_idx = log_idx + 1;

        if(log_idx == 10 && mote_busy == FALSE) {
          mote_busy = TRUE;
          if (call LogWrite.append(&log_line, sizeof(logLine)) != SUCCESS) {
          }
          log_idx = 0;
          mote_busy = FALSE;
        }
        #endif

      }
      else {
        call Leds.led2Toggle();
      }
    }
    else {
      dbg("CarChat","Received ping while in %d state, ignoring\n",state);
    }

    return msg;
  }
  

  event message_t* ReceiveInfr.receive(message_t* msg, void* payload, uint8_t len) {
    //dataMsg* rxMsg = (dataMsg*)payload;
 
    call Leds.led1Toggle();

    dbg("CarChat","Received infrastructure message, immediately going to active zone mode\n");
    if(state != LIVEZ) { // if entering live zone, light up red LED
      if(state == DEADZ_Q) {
        call PingTimer.stop();
      }
      state = LIVEZ;    
      call Leds.led0On();
    }
    else {  // if already in live zone, reset timeout timer
      call LiveZoneExitTimer.stop();
    }
    // process infrastructure data, save in appropriate place

    // start timeout timer, to check if still in live zone
    call LiveZoneExitTimer.startOneShot(INFR_TIMEOUT*1024);

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

}
