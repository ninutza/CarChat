
#include "ReadLog.h"
#include "StorageVolumes.h"

configuration ReadLogAppC {
}

implementation {
  components ReadLogC;
  components MainC;
  components SerialActiveMessageC as SerialAM;
  components LedsC;
  components new TimerMilliC() as ReadTimer;

  components new LogStorageC(VOLUME_LOGTEST, TRUE);

  components UserButtonC;

  ReadLogC.Boot -> MainC.Boot;
  ReadLogC.AMControl -> SerialAM;
  ReadLogC.AMSend -> SerialAM.AMSend[AM_READ_LOG_MSG];
  ReadLogC.Packet -> SerialAM;
  ReadLogC.Leds -> LedsC;
  ReadLogC.ReadTimer -> ReadTimer;

  ReadLogC.LogRead -> LogStorageC;
  ReadLogC.LogWrite -> LogStorageC;

  ReadLogC.Notify -> UserButtonC;
}


