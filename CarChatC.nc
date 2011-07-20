
#include <Timer.h> 
#include "CarChat.h"
// #include "printf.h"


module CarChatC {
  uses {
    interface Boot;
    // interfaces for sending, receiving ping packets 
    interface AMSend as SendPingMsg;
    interface Receive as ReceivePing;
    interface Packet as Packet1;

    // interface for receiving infrastructure packets
    interface Receive as ReceiveInfr;

    // interface for sending, receiving advertisement messages
    interface AMSend as SendAdvMsg;
    interface Receive as ReceiveAdv;
    interface Packet as Packet3;

    // interface for sending, receiving request messages
    interface AMSend as SendReqMsg;
    interface Receive as ReceiveReq;
    interface Packet as Packet4;

    // interface for sending, receiving data messages
    interface AMSend as SendDataMsg;
    interface Receive as ReceiveData;
    interface Packet as Packet5;

    // interfaces for infrastructure mode
    interface AMSend as SendInfrMsg;
    interface Packet as Packet2;
    interface Timer<TMilli> as InfrTimer;

    interface SplitControl as AMControl; 
    interface Timer<TMilli> as PingTimer;
    interface Timer<TMilli> as LiveZoneExitTimer;
    interface Leds;

    interface Timer<TMilli> as PingRecTimer; // timer for recording Pings in DEADZ_Q before initiating contact
    interface Timer<TMilli> as AdvBackoffTimer; // timer for random backoff for sending timer (to avoid collisions)
    interface Timer<TMilli> as CommTOTimer; // timer to end communication in DEADZ_A if it times out

    interface Timer<TMilli> as DataBackOffTimer;
#ifdef SIM_MODE
    interface Timer<TMilli> as HeartBeatTimer;   
#endif

    // for ping suppression in case of congestion
#ifdef PING_SUPPR
    interface Timer<TMilli> as PingSupprTimer;
#endif

    interface LocalTime<TMilli>;

    // for RSSI read
#ifdef SIM_MODE
    interface TossimPacket;
#else
    interface CC2420Packet;
#endif

#ifdef LOGGER_ON
    interface LogWrite;
#endif

    interface Random;
  }
}

implementation {

// ********** VARIABLES DECLARATION **********

  // variables used for vehicular nodes
  message_t PingPkt;
  message_t DataPack;
  message_t ReqPkt;
  message_t AdvPkt;

  
  nx_uint8_t number_pings; // counts pings received in general
  nx_int16_t rssi_value; // signal strength reading

  dataItem mem_data;	// as long as one single data type will be used for testing, with ID "TEST_ID"

  infrItem infr_data;

  actComm curr_comm; // data on communication in progress
  pingRcv ping_track[MAX_PING]; // data on most recent pings, to evaluate best comm partner 
  nx_uint16_t new_chat;

#ifdef LOGGER_ON
  logInput log_buf;	// save aggregate log data, which will all be written at once
  logLine log_line;
  nx_uint8_t log_idx;
#endif
  bool mote_busy;

  // TRY to see if global newData works better
  dataMsg newData;
  
  // variables used for infrastructure nodes
  message_t InfrPkt;
  bool radio_busy;

  nx_uint8_t state; // state in protocol description - Live Zone, Dead Zone Quiescent, Dead Zone Active

  // auxiliary variables - counters, temps
  nx_uint16_t total_no;
  nx_uint16_t no_ping;	// counts pings sent by this node
  nx_uint16_t no_adv;
  nx_uint16_t no_data;
  nx_uint16_t no_req;

  nx_uint32_t startTime;

// ********** FUNCTIONS USED BY CARCHAT **********

  void ChangeState(nx_uint8_t new_state);

  void findNextReq() {
    uint8_t end_comm = 0;
    uint8_t vNum_req, pNum_req; 

    reqMsg *pReqMsg;

    // check curr_comm, and mem_data to figure out what is next packet needed
    // if no packets are needed, send void request
    if(curr_comm.rcvAdv.dataID == 0) {
      dbg("CarErr"," ERROR !!!! Received ad for no data, send adv back / void request !!!!\n");
      end_comm = 1;
    }
    else if(mem_data.incomData.dataID != 0 && 
           (curr_comm.rcvAdv.dataID != mem_data.incomData.dataID || curr_comm.rcvAdv.dType != mem_data.incomData.dType)) { 
      // unknown data if I have some data already, but does not match what I'm getting
      dbg("CarErr"," EXCEPTION !!! Received ad for UNKNOWN data %d, send adv back / void request !!! \n", curr_comm.rcvAdv.dataID);
      end_comm = 1;
    }
    else {
      if(curr_comm.rcvAdv.vNum > mem_data.incomData.vNum) { // more updated version of complete data is now available
        dbg("CarChat","REQDET: Starting from scratch since I have ver %d\n", mem_data.incomData.vNum);
        vNum_req = curr_comm.rcvAdv.vNum;
        pNum_req = 0;
      }  
      else if((curr_comm.rcvAdv.vNum == mem_data.incomData.vNum) && (mem_data.incomData.vNum > mem_data.complData.vNum)) { // incomplete ver needs update
        dbg("CarChat","REQDET: Starting where I left off, since I have ver %d, pack %d\n",mem_data.incomData.vNum,mem_data.incomData.pNum);
        vNum_req = curr_comm.rcvAdv.vNum;
        pNum_req = mem_data.incomData.pNum;
      }
      else {
        dbg("CarErr"," EXCEPTION !!! Received ad for unnecessary data %d, send adv back / void request !!! \n", curr_comm.rcvAdv.dataID);
        end_comm = 1;
      }
    }

    if(end_comm == 1) { // if nothing more is to be requested, send NULL request
      pReqMsg = (reqMsg*)(call Packet4.getPayload(&ReqPkt,sizeof(reqMsg)));
 
      pReqMsg->dataID = 0;
      pReqMsg->destAddr = curr_comm.NodeID;
      pReqMsg->sourceAddr = (uint16_t)TOS_NODE_ID;
 
      dbg("CarChat","Sending NULL request ......\n");

      if((call SendReqMsg.send(AM_BROADCAST_ADDR,&ReqPkt,sizeof(reqMsg))) != FAIL) { 
        no_req++;
      }
    }
    else { // since there is something to request, initialize request now
      
      pReqMsg = (reqMsg*)(call Packet4.getPayload(&ReqPkt,sizeof(reqMsg)));

      pReqMsg->dataID = curr_comm.rcvAdv.dataID;
      pReqMsg->dType =curr_comm.rcvAdv.dType;
      pReqMsg->destAddr = curr_comm.NodeID;
      pReqMsg->sourceAddr = (uint16_t)TOS_NODE_ID;
      pReqMsg->vNum = vNum_req;
      pReqMsg->pNum = pNum_req;
 
      dbg("CarChat","Sending USEFUL request for ver %d (ID %d), pck %d from %d\n", vNum_req, curr_comm.rcvAdv.dataID, pNum_req, curr_comm.NodeID);

      if((call SendReqMsg.send(AM_BROADCAST_ADDR,&ReqPkt,sizeof(reqMsg)))!= FAIL) {
        no_req++;
      }

      call CommTOTimer.startOneShot(3*PING_PER);
    }

  }

  // update state of node
  void ChangeState(nx_uint8_t new_state)
  {
    nx_uint8_t i;

    if(new_state == LIVEZ)
    {
      // toggle green LED to indicate message received from Infrastructure
      call Leds.led1Toggle();
      
      if(state != LIVEZ) { // if entering live zone, light up red LED
        dbg("CarChat"," ... entering Live Zone state\n");
        if(state == DEADZ_Q) {
          call PingTimer.stop();
          call PingRecTimer.stop();
          call AdvBackoffTimer.stop();
          // stop PingEval timer as well (in case car was in the process of detecting new cars)
        } else if(state == DEADZ_A) {
          call CommTOTimer.stop();
        }
        // set state to LIVEZ
        state = LIVEZ;    
        // turn on RED LED to signify in live zone
        //call Leds.led0On();
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
      call Leds.led0Off();
      call Leds.led1Off();
      call Leds.led2Off();

      dbg("CarChat","Entered dead zone active, engaging with node %d\n", curr_comm.NodeID);

      call PingTimer.stop();
      call PingRecTimer.stop();
      call AdvBackoffTimer.stop();

      // turn on BLUE LED to signify dead zone active
      call Leds.led2On();

      // if transition happened after sending an advertisement, wait for request
      if(curr_comm.sentAdv == 1){
        dbg("CarChat"," --- starting TIMEOUT count ---\n");
        call CommTOTimer.startOneShot(3*PING_PER);
      }
      // else, send request based on data
      else { 
        findNextReq();
      }

      state = DEADZ_A;
    }
    else if(new_state == DEADZ_Q)
    {
      dbg("CarChat","Entering DEADZ_Q\n");
 
      call CommTOTimer.stop();
    
      // clear out all data about previous communications
      curr_comm.NodeID = 0;
      curr_comm.sentAdv = 0;
      curr_comm.rcvAdv.dataID = 0; // other fields of rcv Adv are irrelevant once dataID is NULL
      curr_comm.amInit = 0;
      curr_comm.rcvReq = 0;

      // clear out all data about previous pings
      for( i = 0; i < MAX_PING; i++) {
        ping_track[i].NodeID = 0;
        ping_track[i].firstPing = 0;
        ping_track[i].lastPing = 0x0FFF;
      }
  
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
       infr_data.complData.dType = 0x2A;
       infr_data.complData.vNum = (uint8_t)(TOS_NODE_ID - MAX_NODES);
       infr_data.complData.sourceAddr = (uint16_t)TOS_NODE_ID;
       infr_data.complData.destAddr = 0xFFFF;	// broadcast address
       infr_data.complData.pNum = 0;       // this will get incremented after each transmission, cycle on values 0..9

    }
  }

#ifdef LOGGER_ON
  // add log entry to log buffer
  task void AddToLog() {
    // update all entries in log array

    log_buf.sourceType[log_idx] = log_line.sourceType;
    log_buf.no_pings[log_idx] = log_line.no_pings;
    log_buf.sourceAddr[log_idx] = log_line.sourceAddr;
    log_buf.sig_val[log_idx] = log_line.sig_val;
    log_buf.vNum[log_idx] = log_line.vNum;
    log_buf.pNum[log_idx] = log_line.pNum;

    log_buf.time[log_idx] = log_line.time;
    log_buf.curr_vNum[log_idx] = log_line.curr_vNum;
    log_buf.curr_pNum[log_idx] = log_line.curr_pNum;
    log_buf.no_packs[log_idx] = log_line.no_packs;

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
  void updateData() { //dataMsg newData) {

        //#ifdef LOGGER_ON
          
        //log_line.no_pings =  0;
        //log_line.sourceAddr = newData.sourceAddr;
        //log_line.sig_val = ((newData.dataID) << 8) + (newData.dType);	
        //log_line.vNum = newData.vNum;
        //log_line.pNum = newData.pNum;
        //log_line.sourceType = 8;

        //log_line.time = (uint32_t)((call LocalTime.get() - startTime)/1024);;
        //log_line.curr_vNum = mem_data.incomData.vNum;
        //log_line.curr_pNum = mem_data.incomData.pNum;
        //log_line.no_packs = no_adv + no_req + no_data;

        //post AddToLog();

        //#endif


    if(mem_data.incomData.dataID == 0) { // started with a blank slate, initialize dataID and dType in memory
      //call Leds.led2Toggle();
      mem_data.incomData.dataID = newData.dataID;
      mem_data.incomData.dType = newData.dType;
      mem_data.complData.dataID = newData.dataID;
      mem_data.complData.dType = newData.dType;
      mem_data.totalPack = newData.tPack;
    }
  
    if(newData.dataID != mem_data.incomData.dataID || newData.dType != mem_data.incomData.dType) {
      // error, stray data type in system
      dbg("CarChat","ERROR, unknown data type %d (ID %d) received, I have %d (ID %d) --- from %d\n", 
          newData.dataID, newData.dType, mem_data.incomData.dataID,mem_data.incomData.dType, newData.sourceAddr);
      //call Leds.led0Toggle();
    }
    else {
      if(newData.vNum > mem_data.incomData.vNum) { // received packet from new data version, reset partial version
        mem_data.incomData.vNum = newData.vNum;
        mem_data.incomData.pNum = 0;  // don't yet have any packets from this version number
      }
   
      if(newData.vNum == mem_data.incomData.vNum && newData.pNum == mem_data.incomData.pNum) { //next packet received
        mem_data.incomData.pNum++;

        if(mem_data.incomData.pNum == mem_data.totalPack) { // incomplete version has in fact become complete, copy to complete version
          mem_data.complData.vNum = mem_data.incomData.vNum;
        }

        dbg("CarData"," --- Most recent data status is version %d in complete and version %d, packet %d in incomplete ---\n", 
            mem_data.complData.vNum, mem_data.incomData.vNum, mem_data.incomData.pNum);

 
        if(mem_data.complData.vNum > 0 && mem_data.incomData.pNum == mem_data.totalPack) {
          // output for file concerned with completion of data updates
          dbg("CarData","COMPLETED RECEPTION of VERSION %d\n",mem_data.complData.vNum);
          call Leds.led0On();
          call Leds.led1On();
          call Leds.led2On();
        }

      }
      else { // packet out of order, discard
      }

    }
  }

  void addPing(nx_uint16_t node_id, nx_int16_t sig_value) {
    nx_uint8_t i;

    for( i = 0; i < MAX_PING; i++) {

      if (ping_track[i].NodeID == node_id) {
        ping_track[i].lastPing = sig_value;
        dbg("CarMisc","Ping from %d recorded, with value %d\n", node_id, sig_value);
        break;
      }

      if (ping_track[i].NodeID == 0) { // made it past the recorded neighbors, didn't find this one
        ping_track[i].NodeID = node_id;
        ping_track[i].firstPing = sig_value;
//        ping_track[i].lastPing = sig_value; // don't do anything to last ping until it's heard second time 
        dbg("CarMisc","First ping from %d recorded, with value %d\n", node_id, sig_value);
        break;
      }
    }
 
    if ( i >= MAX_PING ) { // all positions were full, current PING did not belong
      dbg("CarErr","Ping from %d superfluous, discarding\n", node_id);
    }
    
  }


  void selPing() { // once we're done recording pings, decide which one amongs them is best to communicate with

    nx_uint8_t i;
    nx_uint8_t max_i;
    nx_int16_t max_diff;

    new_chat = 0;
    max_i = MAX_PING + 1;
    max_diff = -1;

    for( i = 0; i < MAX_PING; i++) {
      if(ping_track[i].NodeID == 0) { // reached the end of recorded pings
        dbg("CarMisc","Less than %d neighbors\n", i+1);
        break;
      }

      dbg("CarMisc","For neighbor %d, difference is %d \n", i+1, abs(ping_track[i].firstPing - ping_track[i].lastPing));

      // don't contact node who's only been heard once - may be transient
      if(abs(ping_track[i].lastPing != 0x0FFF && ping_track[i].firstPing - ping_track[i].lastPing) > max_diff) { 
        // found two pings with a larger difference (~higher rel speed)
        max_i = i;
        max_diff = abs(ping_track[i].firstPing - ping_track[i].lastPing);
        dbg("CarMisc","Node %d fastest one so far\n",ping_track[i].NodeID);
      }     
 
    }
    
    if(max_i <= MAX_PING) { // found a candidate
      new_chat = ping_track[max_i].NodeID;
      dbg("CarChat","Attempting to contact %d to enter Dead Zone Active\n", new_chat);
    }  
  }


// ********** EVENT BEHAVIOR - GENERAL **********

  event void Boot.booted() {  
    // global counters of messages transmitted (overflow not important, still helps with global ordering)
    no_ping = 0;
    no_adv = 0;
    no_data = 0;
    no_req = 0;

    #ifdef SIM_MODE
      call HeartBeatTimer.startPeriodic(SIM_UNIT); // for simulation, need event every SIM_UNIT seconds to update position
    #else
      startTime = call LocalTime.get();
    #endif

    

    // upon booting, first start radio 
    call AMControl.start();
  }

  event void AMControl.startDone(error_t err) {

    if (err == SUCCESS) {	// radio successfully initiated

      if((uint16_t)TOS_NODE_ID < MAX_NODES) // this node is a vehicular node
      {

       mem_data.totalPack = 0;
       mem_data.complData.dataID = 0;
       mem_data.complData.dType = 0;
       mem_data.complData.vNum = 0;
       mem_data.complData.sourceAddr = 0;
       mem_data.complData.destAddr = 0;	
       mem_data.complData.pNum = 0; 

       mem_data.incomData.dataID = 0;
       mem_data.incomData.dType = 0;
       mem_data.incomData.vNum = 0;
       mem_data.incomData.sourceAddr = 0;
       mem_data.incomData.destAddr = 0;	
       mem_data.incomData.pNum = 0; 

        #ifdef LOGGER_ON
          log_idx = 0;
        #endif

        // initiate DeadZoneQuiescent mode
        mote_busy = TRUE;
        atomic {
          ChangeState(DEADZ_Q);
        }
        mote_busy = FALSE;

        dbg("CarChat","Started mote %d in Dead Zone Quiescent Mode\n",TOS_NODE_ID);
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

  #ifdef SIM_MODE
  event void HeartBeatTimer.fired() {
    // do nothing, will count as event for TOSSIM simulation environment (should trigger update)
    if(no_adv + no_req + no_data != total_no) { // some messages have been sent since last update
      total_no = no_adv + no_req + no_data;
      if((total_no) % 20 == 0) { // only display count every 20 messsages
        dbg("CarData","Have sent %d adv, %d req, %d data packets, total %d\n", no_adv, no_req, no_data, total_no);
      }
    }
  }
  #endif


// ********** EVENT BEHAVIOR - Dead Zone Quiescent **********

event void AdvBackoffTimer.fired() {
    advMsg *pAdvMsg = (advMsg*)(call Packet3.getPayload(&AdvPkt,sizeof(advMsg)));
  
    pAdvMsg->dataID = mem_data.complData.dataID; // replace with actual Data ID from memory!!!
    pAdvMsg->dType = mem_data.complData.dType;
    pAdvMsg->vNum = mem_data.complData.vNum;
    pAdvMsg->sourceAddr = (uint16_t)TOS_NODE_ID;
    pAdvMsg->destAddr = new_chat;

    // random backoff timer for sending advertisement went off, check if communication is already taking place, else send adv
    dbg("CarMisc","    ******** Sending advertisement message to %d ********\n", new_chat);

    if(call SendAdvMsg.send(AM_BROADCAST_ADDR,&AdvPkt,sizeof(advMsg)) == FAIL) { 
      dbg("CarErr","Failed sending adv message \n");
      ChangeState(DEADZ_Q);
    }
    else { 
      dbg("CarMisc","Pending Dead Zone Active ---- IGNORING ALL MESSAGES!!\n");
      state = DEADZ_A_pend;
      no_adv++;
    }
  }

event void PingRecTimer.fired() {
    nx_uint16_t rTime;

    // done keeping record of pings, evaluate what's in the records and decide who to contact
    dbg("CarMisc","DONE keeping track of Pings !!!!!\n");

    call PingTimer.stop();

    selPing();

    if(new_chat > 0) {
      dbg("CarMisc","   ***** Initiating conversation with %d *****\n", new_chat);

      if( (TOS_NODE_ID % 3) == (new_chat % 3) && TOS_NODE_ID > new_chat) { // choose larger backoff for larger ID value
        rTime = 4 * BACKOFF_PER;
      } else {
        rTime = ((TOS_NODE_ID % 3) + 1) * BACKOFF_PER;
      } 

      dbg("CarMisc","       ***** Waiting %d ms before sending adv *****\n", rTime);
      call AdvBackoffTimer.startOneShot(rTime);
    }
    else {
      dbg("CarMisc","   ***** No one to talk to :( \n");
      ChangeState(DEADZ_Q);
    }
    
  }

#ifdef PING_SUPPR
  event void PingSupprTimer.fired()
  {
    if(number_pings >= 10) { // then still too congested, continue to be quiet
      call PingSupprTimer.startOneShot(PING_PER);
    }
    else { // network has cleared up some, restart PING timer
      call PingTimer.startPeriodic(PING_PER);
    }

    number_pings = 0;
  }
#endif

  event void PingTimer.fired()
  {
    chatMsg *pChatMsg = (chatMsg*)(call Packet1.getPayload(&PingPkt,sizeof(chatMsg)));

    number_pings = 0;

    dbg("CarChat","Timer went off, sending PING message %d\n", no_ping); 
    
    call Leds.led0Toggle();

    pChatMsg->vNum = 0;  // ping message simply contains a ver. 0 advertisement
    pChatMsg->sourceAddr = (uint16_t)TOS_NODE_ID;
    pChatMsg->no_ping = no_ping;

    // send Ping including source ID for car identification
    if(call SendPingMsg.send(AM_BROADCAST_ADDR,&PingPkt,sizeof(chatMsg))==FAIL){ 
      dbg("CarErr","Failed sending ping message %d\n", no_ping);
    }
    else { 
      no_ping++;
    }
  }

  event message_t* ReceivePing.receive(message_t* msg, void* payload, uint8_t len) {

    if( (uint16_t)TOS_NODE_ID < MAX_NODES ) {

      if(state == DEADZ_Q) {	// ignore pings received while in other states

        // safer way to obtain message payload
        chatMsg *rxMsg = (chatMsg*)(call Packet1.getPayload(msg,sizeof(chatMsg)));

        if( call AdvBackoffTimer.isRunning() ) { 
          dbg("CarMisc","Pending advertisement sending, ignoring ping!!!\n");
          return msg;
        }


        dbg("CarMisc","Received ping message %d from %d - checking for version number\n", rxMsg->no_ping, rxMsg->sourceAddr);
 
        #ifdef SIM_MODE
        rssi_value = (int16_t)(call TossimPacket.strength(msg));
        #else
        rssi_value = (uint16_t)(call CC2420Packet.getRssi(msg));
        #endif

        if(rxMsg->vNum == 0 && !mote_busy) {             // ping signal received, print RSSI reading and log (if logging is on)
          
          call Leds.led1Toggle();

          dbg("CarChat","Counting PING with RSSI %d from node %d\n", rssi_value, rxMsg->sourceAddr);

          if( !(call PingRecTimer.isRunning())) {  // if this is the first ping since entering DEADZ_Q
            call PingRecTimer.startOneShot(2*PING_PER+1);
            dbg("CarMisc","Started recording pings\n");
          }
  
          #ifdef LOGGER_ON
          
          log_line.no_pings = rxMsg->no_ping;
          log_line.sourceAddr = rxMsg->sourceAddr;
          log_line.sig_val = rssi_value;
          log_line.vNum = 0;
          log_line.pNum = 0;
          log_line.sourceType = 1;

          log_line.time = (uint32_t)((call LocalTime.get() - startTime)/1024);;
          log_line.curr_vNum = mem_data.incomData.vNum;
          log_line.curr_pNum = mem_data.incomData.pNum;
          log_line.no_packs = no_adv + no_req + no_data;


	  post AddToLog();

          #endif

          // here, record new ping, and start recording timer if appropriate (new function?)
          number_pings = number_pings + 1;

          #ifdef PING_SUPPR
        
          if(number_pings >= 10 && !(call PingSupprTimer.isRunning()))  // if received too many pings and haven't suppressed yet
          {
            dbg("CarData"," ***** Too many PINGS, starting quiet period for congestion control *****\n");
            call PingTimer.stop();
            call PingSupprTimer.startOneShot(PING_PER);
          }

          #endif

          addPing(rxMsg->sourceAddr, rssi_value);


        } // end if(rxMsg->vNum == 0 && !mote_busy)

     } // end if(state == DEADZ_Q)
     else {
       dbg("CarChat","Received ping while in %d state, ignoring\n",state);
     } // other states, vehicular node

    } // end if( (uint16_t)TOS_NODE_ID < MAX_NODES ) 
    return msg;
  }

  event void SendPingMsg.sendDone(message_t* msg, error_t error) {
     call Leds.led0Toggle();
  }


// ********** EVENT BEHAVIOR - Dead Zone Active **********

// may be used if I decide to introduce a delay between receiving a request and servicing it
  event void DataBackOffTimer.fired() {
    
  }

  event void CommTOTimer.fired() {
    // Dead Zone Active communication timed out, just go back to Dead Zone Quiescent mode
    dbg("CarErr","Dead Zone Active Communication timed out!\n");
    ChangeState(DEADZ_Q);
  }

  event message_t* ReceiveAdv.receive(message_t* msg, void* payload, uint8_t len) {

    if( (uint16_t)TOS_NODE_ID < MAX_NODES ) {  
      // if in LIVE ZONE, ignore
      if(state == LIVEZ) {
        return msg;
      }
      else { // packet may be of interest, get payload
        advMsg *rxMsg = (advMsg*)(call Packet3.getPayload(msg,sizeof(advMsg))); 
      
        dbg("CarChat","!!** Received ADV packet from %d to %d for data %d, ver %d**!! while in state %d\n",
            rxMsg->sourceAddr, rxMsg->destAddr, rxMsg->dataID, rxMsg->vNum, state);

        if(rxMsg -> destAddr != (uint16_t)TOS_NODE_ID) { // if not for me, ignore
          return msg;
        }
        else if(state == DEADZ_Q) { 
        // if in DEADZ_Q, take this guy as comm partner (since it's an adv for me)
        // also take the other as comm partner initiator if its NodeID is a lower number
                // save information on infrastructure data received

          
          atomic {
            dbg("CarChat","Entering DEADZ_A as secondary with node %d\n", rxMsg->sourceAddr);

            curr_comm.NodeID = rxMsg->sourceAddr;
            curr_comm.sentAdv = 0;
            curr_comm.rcvAdv = *rxMsg;
            curr_comm.amInit = 0;
            curr_comm.rcvReq = 0;
 
            call AdvBackoffTimer.stop();

            ChangeState(DEADZ_A);
          }

          #ifdef LOGGER_ON
          
          log_line.no_pings =  0;
          log_line.sourceAddr = rxMsg->sourceAddr;
          log_line.sig_val = ((rxMsg->dataID) << 8) + (rxMsg->dType);	
          log_line.vNum = rxMsg-> vNum;
          log_line.pNum = 0;
          log_line.sourceType = 2;

          log_line.time = (uint32_t)((call LocalTime.get() - startTime)/1024);;
          log_line.curr_vNum = mem_data.incomData.vNum;
          log_line.curr_pNum = mem_data.incomData.pNum;
          log_line.no_packs = no_adv + no_req + no_data;


          post AddToLog();

          #endif

        }
        else if(state==DEADZ_A) { // if in live zone, figure out if advertisement should be considered
          dbg("CarChat","Received ADV while in dead zone active, considering whether to accept it\n");
          if(curr_comm.NodeID == rxMsg->sourceAddr && rxMsg->destAddr == TOS_NODE_ID && curr_comm.sentAdv == 1 && curr_comm.rcvReq == 1) { 

            
          // this is halfway point in communication exchange
            call CommTOTimer.stop();
            dbg("CarChat"," -- ADV is meant for me! --\n");
            curr_comm.rcvAdv = *rxMsg;

            #ifdef LOGGER_ON
          
            log_line.no_pings =  0;
            log_line.sourceAddr = rxMsg->sourceAddr;
            log_line.sig_val = ((rxMsg->dataID) << 8) + (rxMsg->dType);	
            log_line.vNum = rxMsg-> vNum;
            log_line.pNum = 0;
            log_line.sourceType = 2;

            log_line.time = (uint32_t)((call LocalTime.get() - startTime)/1024);;
            log_line.curr_vNum = mem_data.incomData.vNum;
            log_line.curr_pNum = mem_data.incomData.pNum;
            log_line.no_packs = no_adv + no_req + no_data;

            post AddToLog();

            #endif

            findNextReq();
          }
        }
    
      }

    }

    return msg;
  }

  event void SendAdvMsg.sendDone(message_t* msg, error_t error) {

    if(error == SUCCESS && state == DEADZ_A_pend) {
      atomic {  // this is the case when we enter Dead Zone Active as initiators
        dbg("CarChat","DONE sending an advertisement message!\n");
        dbg("CarChat","Entering DEADZ_A as initiator with node %d\n", new_chat);
        curr_comm.NodeID = new_chat;
        curr_comm.sentAdv = 1;
        curr_comm.amInit = 1;
        curr_comm.rcvReq = 0;

        ChangeState(DEADZ_A);
      }
    }

    if(error == SUCCESS && state == DEADZ_A && curr_comm.sentAdv == 0) { // probably sent advertisement halfway thru comm
      dbg("CarChat","Just sent ADV in my turn to other node\n");
      curr_comm.sentAdv = 1;
    }
  }

  event message_t* ReceiveReq.receive(message_t* msg, void* payload, uint8_t len) {
    // if in LIVE ZONE, ignore
    // if not for me, ignore
    // if in DEADZ_A with another, ignore
    	// immediately send data as requested

    if(state == DEADZ_A) { 
      reqMsg *rxMsg = (reqMsg*)(call Packet4.getPayload(msg,sizeof(reqMsg))); 
  
      if(rxMsg->sourceAddr == curr_comm.NodeID && rxMsg->destAddr == TOS_NODE_ID) { // request part of current comm
      
        call CommTOTimer.stop();
 
        #ifdef LOGGER_ON
          
        log_line.no_pings =  0;
        log_line.sourceAddr = rxMsg->sourceAddr;
        log_line.sig_val = ((rxMsg->dataID) << 8) + (rxMsg->dType);	
        log_line.vNum = rxMsg-> vNum;
        log_line.pNum = rxMsg-> pNum;
        log_line.sourceType = 3;

        log_line.time = (uint32_t)((call LocalTime.get() - startTime)/1024);;
        log_line.curr_vNum = mem_data.incomData.vNum;
        log_line.curr_pNum = mem_data.incomData.pNum;
        log_line.no_packs = no_adv + no_req + no_data;

        post AddToLog();

        #endif


        if(rxMsg->dataID == 0) { // null request, return to DEADZ_Q if not initiator
          curr_comm.rcvReq = 1;
          if(curr_comm.amInit == 0) {
            dbg("CarChat","Received a NULL request, going back to DEADZ_Q\n");
            ChangeState(DEADZ_Q);
          }
          else {
            dbg("CarChat","Received a NULL request, waiting for other to send ADV\n");
            call CommTOTimer.startOneShot(3*PING_PER);
          }
        }
        else { // real request, respond with data
          dbg("CarMisc","Attempting to provide version %d, packet %d to %d\n");
          if(rxMsg->dataID != mem_data.complData.dataID || rxMsg->dType != mem_data.complData.dType) { // request for invalid data
            dbg("CarMisc","Invalid data type requested!\n");
            return msg;
          }
          // should have data if request is sent, but just in case, send back NULL request/adv if data is invalid
          if(mem_data.complData.vNum == rxMsg->vNum || (mem_data.incomData.vNum == rxMsg->vNum && mem_data.incomData.pNum > rxMsg->pNum)) {
            dataMsg *pdataMsg = (dataMsg*)(call Packet5.getPayload(&DataPack,sizeof(dataMsg)));

            pdataMsg->dataID = rxMsg->dataID; 
            pdataMsg->dType = rxMsg->dType;
            pdataMsg->vNum = rxMsg->vNum;
            pdataMsg->sourceAddr = (uint16_t)TOS_NODE_ID;
            pdataMsg->pNum = rxMsg->pNum;
            pdataMsg->tPack = mem_data.totalPack;

            dbg("CarData","Sending data ID %d, type %d, from %d to %d!!!\n", pdataMsg->dataID, pdataMsg->dType, pdataMsg->sourceAddr, rxMsg->sourceAddr);

            if(call SendDataMsg.send(AM_BROADCAST_ADDR, &DataPack, sizeof(dataMsg)) == SUCCESS) {
              call CommTOTimer.startOneShot(3*PING_PER);
              no_data++;
            }
          }
        }
      }
    }

    return msg;
  }

  event void SendReqMsg.sendDone(message_t* msg, error_t error) {
    // if NULL request was sent and node is NOT initiator, follow up with own advertisement
    reqMsg* sentMsg = (reqMsg*)(call Packet4.getPayload(msg,sizeof(reqMsg)));

    if(sentMsg->dataID == 0) { // null request was sent, think about going back to DEADZ_Q
      dbg("CarChat","Null request was sent...\n");
      if(curr_comm.amInit == 0) { // non-initiator, then send own advertisement
        advMsg* pAdvMsg = (advMsg*)(call Packet3.getPayload(&AdvPkt,sizeof(advMsg))); 
        dbg("CarChat","... sending follow-up adv\n");
  
        pAdvMsg->dataID = mem_data.complData.dataID; 
        pAdvMsg->dType = mem_data.complData.dType;
        pAdvMsg->vNum = mem_data.complData.vNum;
        pAdvMsg->sourceAddr = (uint16_t)TOS_NODE_ID;
        pAdvMsg->destAddr = curr_comm.NodeID;

        if(call SendAdvMsg.send(AM_BROADCAST_ADDR,&AdvPkt,sizeof(advMsg)) == FAIL) { 
          dbg("CarErr","Failed sending adv message \n");
          ChangeState(DEADZ_Q);
        } 
        else {
          dbg("CarChat","Sending own adv, pending in DEADZ_A while waiting for requests\n");
          no_adv++;
          call CommTOTimer.startOneShot(3*PING_PER);
        } // end if(call SendAdvMsg. ...
      }
      else { // initiator, can go back to DEADZ_Q
         dbg("CarChat","... going back to DEADZ_Q\n");
         ChangeState(DEADZ_Q);
      } // end if(curr_comm.amInit == 0
    } // end if(sentMsg-> ...
  }

  event message_t* ReceiveData.receive(message_t* msg, void* payload, uint8_t len) {
    if(TOS_NODE_ID < MAX_NODES && state != LIVEZ) {
      // if data is relevant, save, else drop packet
      dataMsg* rxMsg = (dataMsg*)(call Packet5.getPayload(msg,sizeof(dataMsg)));
      //dataMsg newData;

      #ifdef LOGGER_ON
      
      log_line.no_pings = 0;
      log_line.sourceAddr = rxMsg->sourceAddr;
      log_line.sig_val = ((rxMsg->dataID) << 8) + (rxMsg->dType);	
      log_line.vNum = rxMsg-> vNum;
      log_line.pNum = rxMsg-> pNum;
      log_line.sourceType = 4;

      log_line.time = (uint32_t)((call LocalTime.get() - startTime)/1024);;
      log_line.curr_vNum = mem_data.incomData.vNum;
      log_line.curr_pNum = mem_data.incomData.pNum;
      log_line.no_packs = no_adv + no_req + no_data;

      post AddToLog();

      #endif

      // ***** upload data *****
      atomic {
        newData.dataID = rxMsg->dataID;
        newData.dType = rxMsg->dType;
        newData.vNum = rxMsg->vNum; 
        newData.pNum = rxMsg->pNum;
        newData.sourceAddr = rxMsg->sourceAddr;
        newData.tPack = rxMsg->tPack;
        updateData(); //newData);
      }

      // if data is from communication partner, send next request back
      if(state == DEADZ_Q && curr_comm.NodeID == rxMsg->sourceAddr) { 
        findNextReq();
      }
    }

    return msg;
  }

  event void SendDataMsg.sendDone(message_t* msg, error_t error) {
  }



  
// ********** EVENT BEHAVIOR - Live Zone **********

  // timeout from Live Zone
  event void LiveZoneExitTimer.fired() {

    // when live zone times out, time to switch back to Dead Zone Quiescent state
    dbg("CarChat","Timed out on live zone, starting to send PING messages\n"); 
    ChangeState(DEADZ_Q);

  }

  // received infrastructure message
  event message_t* ReceiveInfr.receive(message_t* msg, void* payload, uint8_t len) {
    
    if( (uint16_t)TOS_NODE_ID < MAX_NODES ) {  // only process infrastructure message if node is vehicular
       
      // safer way to obtain message payload
      dataMsg *rxMsg = (dataMsg*)(call Packet2.getPayload(msg,sizeof(dataMsg)));
      //dataMsg newData;

      dbg("CarChat","Received infrastructure message, data ID %d (type %d), version number %d, packet %d of %d\n", 
          rxMsg->dataID, rxMsg->dType, rxMsg->vNum, rxMsg->pNum, rxMsg->tPack);


      atomic {
        newData.dataID = rxMsg->dataID;
        newData.dType = rxMsg->dType;
        newData.vNum = rxMsg->vNum; 
        newData.pNum = rxMsg->pNum;
        newData.sourceAddr = rxMsg->sourceAddr;
        newData.tPack = rxMsg->tPack;
        updateData(); //newData);
      } 
   
      atomic {

      // save information on infrastructure data received
      #ifdef LOGGER_ON
          
        log_line.sourceType = 0;
        log_line.no_pings = 0;
        log_line.sourceAddr = rxMsg->sourceAddr;
        log_line.sig_val = ((rxMsg->dataID) << 8) + (rxMsg->dType);	
        log_line.vNum = rxMsg-> vNum;
        log_line.pNum = rxMsg-> pNum;

        log_line.time = (uint32_t)((call LocalTime.get() - startTime)/1024);
        log_line.curr_vNum = mem_data.incomData.vNum;
        log_line.curr_pNum = mem_data.incomData.pNum;
        log_line.no_packs = no_adv + no_req + no_data;


        post AddToLog();

      #endif

      }
      
      atomic {
          ChangeState(LIVEZ);
        }

    }
 
    return msg;
  }



// ********** EVENT BEHAVIOR - Logging **********  

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
    pInfrMsg->dType = infr_data.complData.dType;
    pInfrMsg->vNum = infr_data.complData.vNum;
    pInfrMsg->sourceAddr = infr_data.complData.sourceAddr;
    pInfrMsg->pNum = infr_data.complData.pNum;
    pInfrMsg->tPack = infr_data.totalPack;

    // send first data packet with source ID = 0 (infrastructure)
    if(call SendInfrMsg.send(AM_BROADCAST_ADDR,&InfrPkt,sizeof(dataMsg))==FAIL){ 
      dbg("CarChat","Sending Failed\n");
    }
    else { 
      // after successfully broadcasting one packet, move on to next
      infr_data.complData.pNum = (infr_data.complData.pNum + 1) % infr_data.totalPack;
      //dbg("CarData","Infrastructure sent packet %d (ver %d) of message with ID %d (type %d), with source address %d\n",
      //    infr_data.complData.pNum, infr_data.complData.vNum, infr_data.complData.dataID, infr_data.complData.dType, infr_data.complData.sourceAddr);
    }
  }

  event void SendInfrMsg.sendDone(message_t* msg, error_t error) {
     call Leds.led2Off();
     dbg("InfrChat", "Sending complete, starting timer again\n");
     call InfrTimer.startOneShot(INFRMSG_PER); 
  }


}
