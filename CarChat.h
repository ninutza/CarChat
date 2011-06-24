
#ifndef CARCHAT_H
#define CARCHAT_H

enum {
  AM_PINGMSG = 167,
  AM_INFRMSG = 168,
  PING_PER = 1000,
  INFR_TIMEOUT = 10, //time in seconds
  DATASIZE = 30,
  LIVEZ = 1,
  DEADZ_Q = 2,
  DEADZ_A = 3
};

typedef nx_struct dataMsg {
  nx_uint16_t sourceAddr; // always 0 for infrastructure nodes
  nx_uint8_t dType; //data type (i.e. 1 = weather, 2 = traffic, 3 = worksites)
  nx_uint8_t vNum; //version number
  nx_uint8_t pNum; //packet number
  nx_uint8_t data[DATASIZE]; // data contents
} dataMsg;

typedef nx_struct chatMsg {
  nx_uint16_t sourceAddr;
  nx_uint16_t dest;
  nx_uint8_t vNum; //version number
} chatMsg;


typedef nx_struct logLine {
    nx_uint8_t no_pings;
    nx_uint16_t sourceAddr;
  } logLine;

#endif
