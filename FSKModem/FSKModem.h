#include "Arduino.h"
#ifndef FSKModem_h
#define FSKModem_h


class FSKModem
{
  public:
	void begin(int baud, int sender, int receiver);
    byte read();
    void write( byte out);
	virtual int available();
	void getIntOvf();
	void receive();
	void send();
	void send2();
	int status(int param);
  private:
//	static void ISR_FSK_Receiver();
    volatile int _pin;
	volatile long lowFreq,hiFreq;
	volatile int lowFreqTolerance, hiFreqTolerance;
	volatile boolean _inReceive;
	volatile byte _bitReceive,_byteReceive,_bitSend,_byteSend;
	volatile byte _recvBuffer[16];
	volatile byte _sendBuffer[16];
	volatile byte _recvStart,_recvEnd;
	volatile byte _sendStart, _sendEnd;
};

extern FSKModem modem;
#endif
