
#include <Timer.h> 
#include "CarChat.h"
#include "printf.h"


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
    interface CC2420Packet;

#ifdef LOGGER_ON
    interface LogWrite;
    interface LogRead;    
#endif
  }
}

implementation {
  message_t PingPkt;
  nx_uint8_t no_ping;	// counts pings sent by this node
  nx_uint8_t number_pings; // counts pings received in general
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
      // set periodic PING timer
      // reset PING recording fields
      // set state to DEADZ_Q
    }
  }

  event void Boot.booted() {

    // upon booting, first start radio (in DeadZoneQuiescent mode)

    state = DEADZ_Q;
    no_ping = 0;
//    printf("Turned on, starting radio\n");
//    printfflush();


    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {

    if (err == SUCCESS) {	// radio successfully initiated

      #ifdef LOGGER_ON
//      printf("Radio on, starting to read log\n");
//      printfflush();

      log_idx = 0;
      mote_busy = TRUE;

      call Leds.led2On();
  
      if (call LogRead.read(&log_line, sizeof(logLine)) != SUCCESS) { 
	// not critical, so no error handling
      }
      #else
//      printf("Radio on, skipping log\n");
//      printfflush();

      mote_busy = FALSE;

      number_pings = 0;
      call PingTimer.startPeriodic(PING_PER); 

      dbg("CarChat","Started ping timer in Dead Zone Quiescent Mode\n");
      #endif

    }
    else {
      call AMControl.start();
    }

  }

  event void AMControl.stopDone(error_t err) {}

  event void PingTimer.fired()
  {
    chatMsg *pChatMsg = (chatMsg*)(call Packet.getPayload(&PingPkt,(uint8_t)NULL));

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
    state = DEADZ_Q;
    call PingTimer.startPeriodic(PING_PER);    
    call Leds.led0Off();
  }


  event message_t* ReceivePing.receive(message_t* msg, void* payload, uint8_t len) {
    chatMsg* rxMsg = (chatMsg*)payload;
    
    if(state == DEADZ_Q) {

      dbg("CarChat","Received ping message from %d - checking for version number\n", rxMsg->sourceAddr);

      if(rxMsg->vNum == 0 && !mote_busy) {             // ping signal received, do nothing for now ** insert RSSI reading **
        call Leds.led1Toggle();
        dbg("CarChat","Message has version 0, counting PING\n");
   
        number_pings = number_pings + 1;
  
        #ifdef LOGGER_ON
        log_line.no_pings[log_idx] = rxMsg->no_ping;
        log_line.sourceAddr[log_idx] = rxMsg->sourceAddr;
        log_line.sig_val[log_idx] = (uint16_t) call CC2420Packet.getRssi(msg);
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
  event void LogRead.readDone(void* buf, storage_len_t len, error_t err) {

    if ( (len == sizeof(logLine)) && (buf == &log_line) ) {
      // print out contents of log to screen 
      nx_uint8_t idx;
      for(idx = 0; idx < 10; idx = idx+1) {
        printf("Logged ping #%d, from %d, with strength %d\n", log_line.no_pings[idx], log_line.sourceAddr[idx], log_line.sig_val[idx]);
        printfflush();
      }

      if (call LogRead.read(&log_line, sizeof(logLine)) != SUCCESS) { 
	// not critical, so no error handling
      }
    }
    else { // log was completely read
      if (call LogWrite.erase() != SUCCESS) { 
      // error handling, not critical 
      }

      printf("Log read done, starting Ping timer\n");
      printfflush();

      call Leds.led2Off();

      mote_busy = FALSE;

      number_pings = 0;
      call PingTimer.startPeriodic(PING_PER); 

      dbg("CarChat","Started ping timer in Dead Zone Quiescent Mode\n");

    }

  }

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

  event void LogRead.seekDone(error_t err) {
  }

  event void LogWrite.syncDone(error_t err) {
  }

#endif

}
