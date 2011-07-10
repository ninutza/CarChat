COMPONENT=CarChatAppC
CFLAGS += -I$(TOSDIR)/lib/printf
PFLAGS += -DCC2420_DEF_CHANNEL=26
CFLAGS += -DTOSH_DATA_LENGTH=50		# 50 bytes of data per packet - this leaves <= 45B for traffic data
PFLAGS += -DCC2420_DEF_RFPOWER=30	#range of 0 to 31


include $(MAKERULES)
