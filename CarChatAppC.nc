#include "CarChat.h"
#ifdef LOGGER_ON
  #include "StorageVolumes.h"
#endif

configuration CarChatAppC{ 
}

implementation {

// basic components for functionality 
  components CarChatC;
  components MainC;
  components LedsC;
  components  ActiveMessageC; 


  CarChatC -> MainC.Boot;
  CarChatC.Leds -> LedsC;
  CarChatC.AMControl -> ActiveMessageC;


// components used if node acts as vehicle
  components new TimerMilliC() as PingTimer;
  components new TimerMilliC() as LiveTimer;
  components new AMSenderC(AM_PINGMSG) as AMSender1;
  components new AMReceiverC(AM_PINGMSG) as AMReceiver1;
  components new AMReceiverC(AM_INFRMSG) as AMReceiver2;

#ifdef SIM_MODE
  components TossimActiveMessageC as TossimMsgC;
#else
  components CC2420ActiveMessageC as CC2420MsgC;
#endif

#ifdef LOGGER_ON
  components new LogStorageC(VOLUME_LOGTEST, TRUE);
#endif
  
  CarChatC.PingTimer -> PingTimer;
  CarChatC.LiveZoneExitTimer -> LiveTimer;
  CarChatC.Packet1 -> AMSender1; 
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

// components used if node acts as infrastructure node
  components new AMSenderC(AM_INFRMSG) as AMSender2;
  components new TimerMilliC() as InfrTimer;

  CarChatC.Packet2 -> AMSender2;
  CarChatC.InfrTimer -> InfrTimer;
  CarChatC.SendInfrMsg -> AMSender2.AMSend;
}

