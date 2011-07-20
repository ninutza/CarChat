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
  components new AMSenderC(AM_ADVMSG) as AMSender3;
  components new AMSenderC(AM_REQMSG) as AMSender4;
  components new AMSenderC(AM_DATAMSG) as AMSender5;
  components new AMReceiverC(AM_PINGMSG) as AMReceiver1;
  components new AMReceiverC(AM_INFRMSG) as AMReceiver2;
  components new AMReceiverC(AM_ADVMSG) as AMReceiver3;
  components new AMReceiverC(AM_REQMSG) as AMReceiver4;
  components new AMReceiverC(AM_DATAMSG) as AMReceiver5;

  components new TimerMilliC() as PingRecTimer; // timer for recording Pings in DEADZ_Q before initiating contact
  components new TimerMilliC() as AdvBackoffTimer; // timer for random backoff for sending timer (to avoid collisions)
  components new TimerMilliC() as CommTOTimer; // timer to end communication in DEADZ_A if it times out

  components new TimerMilliC() as DataBackOffTimer; // timer to end communication in DEADZ_A if it times out

  components HilTimerMilliC;
  CarChatC.LocalTime -> HilTimerMilliC;

  components  RandomLfsrC;

#ifdef PING_SUPPR
  components new TimerMilliC() as PingSupprTimer;
  CarChatC.PingSupprTimer -> PingSupprTimer;
#endif

#ifdef SIM_MODE
  components TossimActiveMessageC as TossimMsgC;
  components new TimerMilliC() as HeartBeatTimer;
#else
  components CC2420ActiveMessageC as CC2420MsgC;
#endif

#ifdef LOGGER_ON
  components new LogStorageC(VOLUME_LOGTEST, TRUE);
#endif
  
  CarChatC.PingTimer -> PingTimer;
  CarChatC.LiveZoneExitTimer -> LiveTimer;
  CarChatC.PingRecTimer -> PingRecTimer;
  CarChatC.AdvBackoffTimer -> AdvBackoffTimer;
  CarChatC.CommTOTimer -> CommTOTimer;
  CarChatC.DataBackOffTimer -> DataBackOffTimer;

  CarChatC.Packet1 -> AMSender1; 
  CarChatC.ReceivePing -> AMReceiver1;
  CarChatC.ReceiveInfr -> AMReceiver2;
  CarChatC.SendPingMsg -> AMSender1.AMSend;

  CarChatC.ReceiveAdv -> AMReceiver3;
  CarChatC.Packet3 -> AMSender3;
  CarChatC.SendAdvMsg -> AMSender3.AMSend;

  CarChatC.ReceiveReq -> AMReceiver4;
  CarChatC.Packet4 -> AMSender4;
  CarChatC.SendReqMsg -> AMSender4.AMSend;

  CarChatC.ReceiveData -> AMReceiver5;
  CarChatC.Packet5 -> AMSender5;
  CarChatC.SendDataMsg -> AMSender5.AMSend;

  CarChatC.Random ->  RandomLfsrC;
 
#ifdef SIM_MODE
  CarChatC -> TossimMsgC.TossimPacket;
  CarChatC.HeartBeatTimer ->  HeartBeatTimer;
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

