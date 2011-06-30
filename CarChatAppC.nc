#include "CarChat.h"
#ifdef LOGGER_ON
  #include "StorageVolumes.h"
#endif

configuration CarChatAppC{ 
}

implementation {
  components MainC;
  components CarChatC;
  components new TimerMilliC() as PingTimer;
  components new TimerMilliC() as LiveTimer;
  components  ActiveMessageC; 
  components new AMSenderC(AM_PINGMSG) as AMSender1;
  components new AMReceiverC(AM_PINGMSG) as AMReceiver1;
  components new AMReceiverC(AM_INFRMSG) as AMReceiver2;
  components LedsC;

#ifdef SIM_MODE
  components TossimActiveMessageC as TossimMsgC;
#else
  components CC2420ActiveMessageC as CC2420MsgC;
#endif

#ifdef LOGGER_ON
  components new LogStorageC(VOLUME_LOGTEST, TRUE);
#endif
  
  CarChatC -> MainC.Boot;
  CarChatC.Leds -> LedsC;
  CarChatC.PingTimer -> PingTimer;
  CarChatC.LiveZoneExitTimer -> LiveTimer;
  CarChatC.Packet -> AMSender1; 
  CarChatC.AMControl -> ActiveMessageC;
  CarChatC.ReceivePing -> AMReceiver1;
  CarChatC.ReceiveInfr -> AMReceiver2;
  CarChatC.SendPingMsg -> AMSender1.AMSend;

#ifdef SIM_MODE
  CarChatC -> TossimMsgC.TossimPacket;
#else
  CarChatC -> CC2420MsgC.CC2420Packet;
#endif


#ifdef LOGGER_ON
  CarChatC.LogWrite -> LogStorageC;
#endif
}

