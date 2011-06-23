#include "CarChat.h"
#include "StorageVolumes.h"

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

  components new LogStorageC(VOLUME_LOGTEST, TRUE);
  
  CarChatC -> MainC.Boot;
  CarChatC.Leds -> LedsC;
  CarChatC.PingTimer -> PingTimer;
  CarChatC.LiveZoneExitTimer -> LiveTimer;
  CarChatC.Packet -> AMSender1; 
  CarChatC.AMControl -> ActiveMessageC;
  CarChatC.ReceivePing -> AMReceiver1;
  CarChatC.ReceiveInfr -> AMReceiver2;
  CarChatC.SendPingMsg -> AMSender1.AMSend;

  CarChatC.LogWrite -> LogStorageC;
  CarChatC.LogRead -> LogStorageC;
}

