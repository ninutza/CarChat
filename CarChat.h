
#ifndef CARCHAT_H
#define CARCHAT_H

#define LOGGER_ON       // uncomment for testbed compilation/installation
//#define SIM_MODE      // uncomment for simulation mode compilation

//#define PING_SUPPR	// suppress pinging for PING_PER interval if more than SUPPR_NO pings are heard in any one interval 	

enum {
  AM_PINGMSG = 167,	// AM type for ping(beacon) messages
  AM_INFRMSG = 173,     // AM type for infrastructure messages 
  AM_ADVMSG = 179,      // AM type for advertisement messages 
  AM_REQMSG = 185,      // AM type for request messages 
  AM_DATAMSG = 191,     // AM type for data messages 

  PING_PER = 1000,      // ping interval in ms
  INFR_TIMEOUT = 10,    //time in seconds until live zone times out
  CAR_TIMEOUT = 3,      // time in seconds until dead zone active times out
  INFRMSG_PER = 1024,   // time between transmissions, in ms (for infrastructure nodes)
  BACKOFF_PER = 525,    // time for random backoff in ms for sending advertisement

  DATASIZE = 30,        // amount of data transmitted per packet

  LIVEZ = 1,		// state encoding
  DEADZ_Q = 2,
  DEADZ_A = 3,
  DEADZ_A_pend = 4,
  INFR_NODE = 0xFF,	// special state for nodes operating in infrastructure mode

  MAX_NODES = 300,	// this is how many nodes can operate in vehicular mode, anything with higher ID will be infrastructure
  TEST_ID = 94,          // in simple tests, only 1 data item will circulate from infrastructure
  MAX_DATA = 1,
  MAX_PING = 5,
  LOG_MAX = 10,
  SIM_UNIT = 308,	// in ms, smallest unit of simulation time update (for heartbeat timer)

  SUPPR_NO = 10
};

// dissemination packet
typedef nx_struct dataMsg {
  nx_uint8_t dataID; // unique data identifier
  nx_uint8_t dType; //data type (i.e. 1 = weather, 2 = traffic, 3 = worksites)
  nx_uint8_t vNum; //version number
  nx_uint8_t pNum; //packet number
  nx_uint16_t sourceAddr; // always 0 for infrastructure nodes
  nx_uint8_t tPack; // total number of packets
  nx_uint8_t data[DATASIZE]; // data contents
} dataMsg;

// request packet
typedef nx_struct reqMsg {
  nx_uint8_t dataID;
  nx_uint8_t dType;
  nx_uint8_t vNum;
  nx_uint8_t pNum; // packet number will be 0 if complete data is requested. else, number of first missing packet is transmitted
  nx_uint16_t sourceAddr;
  nx_uint16_t destAddr;
} reqMsg;

// advertisement message packet
typedef nx_struct advMsg {
  nx_uint8_t dataID; // this will be an array if multiple data versions are stored (it may advertise partial versions too)
  nx_uint8_t dType;
  nx_uint8_t vNum;
  nx_uint16_t sourceAddr;
  nx_uint16_t destAddr;
} advMsg;

// PING packet
typedef nx_struct chatMsg {
  nx_uint8_t vNum; //version number
  nx_uint16_t sourceAddr;
  nx_uint16_t no_ping;
} chatMsg;


// ********** data structures for memory storage **********

// to save memory on TelosB motes, we won't actually save data received; instead, we will send default value of data[DATASIZE] array
// and only save the version number (vNum) and received packet numbers (pNum) at the receiver - same structure as request packets

// structure of data items 
typedef nx_struct dataItem {
  nx_uint8_t totalPack;	// total number of packets that this data is divided in
  reqMsg complData;	// this can be an array if multiple complete versions are stored
  reqMsg incomData;	// this stores the incomplete version of data
} dataItem;

typedef nx_struct infrItem {
  nx_uint8_t totalPack;
  reqMsg complData;
} infrItem;

// maintain infromation on current communication partner in Dead Zone Active state
typedef nx_struct actComm {
  nx_uint16_t NodeID; // 0 for no current communication in progress (reset when going to DEADZ_Q)
  nx_uint8_t sentAdv; // 0 for not yet, 1 for yes
   nx_uint8_t amInit; // 1 if node was initiator - meaning it won't accept adv until it's received a req; ** redundant wrt sentAdv **
  nx_uint8_t rcvReq; // 1 if null request was sent by NodeID ;  ** may be redundant **
  advMsg rcvAdv; // store advertisement received to determine what packet we need next
} actComm;

typedef nx_struct pingRcv {
  nx_uint16_t NodeID;
  nx_int16_t firstPing;
  nx_int16_t lastPing;
} pingRcv;

// ********** data structures for Logging node activities **********

// log data set - every 10 messages record what was received
typedef nx_struct logLine {
    nx_uint8_t sourceType;     // 0 = infr, 1 = ping, 2 = adv, 3 = req, 4 = data
    nx_uint8_t no_pings;       // order number (local to the sending node)
    nx_uint16_t sourceAddr;
    nx_int16_t sig_val;    // this will contain a concatenated value of data type and data ID if not ping msg
    nx_uint8_t vNum;
    nx_uint8_t pNum;

    nx_uint32_t time;	// current time when logging
    nx_uint8_t curr_vNum; // current version in incomplete
    nx_uint8_t curr_pNum; // current packet number in incomplete
    nx_uint16_t no_packs; // number of packets sent while running
  } logLine;

typedef nx_struct logInput {
    nx_uint8_t sourceType[LOG_MAX];
    nx_uint8_t no_pings[LOG_MAX];	// order number (local to the sending node)
    nx_uint16_t sourceAddr[LOG_MAX];
    nx_int16_t sig_val[LOG_MAX];    // this will contain a concatenated value of data type and data ID if not ping msg
    nx_uint8_t vNum[LOG_MAX];
    nx_uint8_t pNum[LOG_MAX];


    nx_uint32_t time[LOG_MAX];	// current time when logging
    nx_uint8_t curr_vNum[LOG_MAX]; // current version in incomplete
    nx_uint8_t curr_pNum[LOG_MAX]; // current packet number in incomplete
    nx_uint16_t no_packs[LOG_MAX]; // number of packets sent while running

  } logInput;


#endif
