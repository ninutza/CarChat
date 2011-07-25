
#ifndef READ_LOG_H
#define READ_LOG_H

enum {
  AM_READ_LOG_MSG = 0x89,
  LOG_MAX = 10
};

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

  } read_log_msg_t;

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
