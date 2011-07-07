
#ifndef CARCHAT_H
#define CARCHAT_H

//#define LOGGER_ON
#define SIM_MODE
enum {
  AM_PINGMSG = 167,	// AM type for ping(beacon) messages
  AM_INFRMSG = 168,     // AM type for infrastructure messages 
  PING_PER = 1000,      // ping interval in ms
  INFR_TIMEOUT = 10,    //time in seconds until live zone times out
  INFRMSG_PER = 1024,   // time between transmissions, in ms (for infrastructure nodes)
  DATASIZE = 30,        // amount of data transmitted per packet
  LIVEZ = 1,		// state encoding
  DEADZ_Q = 2,
  DEADZ_A = 3,
  INFR_NODE = 0xFF,	// special state for nodes operating in infrastructure mode
  MAX_NODES = 100,	// this is how many nodes can operate in vehicular mode, anything with higher ID will be infrastructure
  TEST_ID = 94,          // in simple tests, only 1 data item will circulate from infrastructure
  MAX_DATA = 1
};

// dissemination packet
typedef nx_struct dataMsg {
  nx_uint8_t dataID; // unique data identifier
  nx_uint8_t vNum; //version number
  nx_uint16_t sourceAddr; // always 0 for infrastructure nodes
  nx_uint8_t dType; //data type (i.e. 1 = weather, 2 = traffic, 3 = worksites)
  nx_uint8_t pNum; //packet number
  nx_uint8_t data[DATASIZE]; // data contents
} dataMsg;

// request packet
typedef nx_struct reqMsg {
  nx_uint8_t dataID;
  nx_uint8_t vNum;
  nx_uint16_t sourceAddr;
  nx_uint16_t destAddr;
  nx_uint8_t pNum; // packet number will be 0 if complete data is requested. else, numnber of first missing packet is transmitted
} reqMsg;

// to save memory on TelosB motes, we won't actually save data received; instead, we will send default value of data[DATASIZE] array
// and only save the version number (vNum) and received packet numbers (pNum) at the receiver - same structure as request packets

// structure of data items 
typedef nx_struct dataItem {
  nx_uint8_t totalPack;	// total number of packets that this data is divided in
  reqMsg complData;	// this can be an array if multiple complete versions are stored
  reqMsg incomData;	// this stores the incomplete version of data
} dataItem;


// PING packet
typedef nx_struct chatMsg {
  nx_uint8_t vNum; //version number
  nx_uint16_t sourceAddr;
  nx_uint16_t no_ping;
} chatMsg;

// log data set - every 10 messages record what was received
typedef nx_struct logLine {
    nx_uint8_t no_pings[10];	// order number (local to the sending node)
    nx_uint16_t sourceAddr[10];
    nx_uint16_t sig_val[10];    // this will contain a concatenated value of data type and data ID if not ping msg
    nx_uint8_t vNum[10];
    nx_uint8_t pNum[10];
  } logLine;

#endif
