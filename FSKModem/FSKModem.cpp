/* Library to simplex communication on a voice channel with FSK modulation
 * The library uses Timer2 
 * suggestion. pin2,pin3 for input 
 * 
 */

#include "Arduino.h"
#include "FSKModem.h"
//#include "avr/interrupt.h"


static void ISR_FSK_Receiver(void) //incoming signal receiver interrupt service routine
{
	sei();
  modem.receive();
}

FSKModem modem;

ISR(TIMER2_COMPA_vect) // timer compareA interrupt service routine
{
	sei();
  modem.send();
}

ISR(TIMER2_COMPB_vect) // timer compareB interrupt service routine
{
	sei();
  modem.send2();
}

ISR(TIMER2_OVF_vect) // timer overflow interrupt service routine
{
	sei();
  modem.getIntOvf();
}


//FSKModem::FSKModem();


void FSKModem::begin(int baud, int sender, int receiver)
{
	long presc;
	
	_recvStart = 0;
	_recvEnd = 0;
	_sendStart = 0;
	_sendEnd = 0;
	_inReceive = false;
	_bitReceive = 0;
	_pin=sender;
	_bitSend=50;
	_byteSend=0;

	if (sender!=0) {
		pinMode(sender,OUTPUT);
//		pinMode(sender+1,OUTPUT);
	}
	
	if (receiver!=0) {
		pinMode(receiver, INPUT);
		attachInterrupt(digitalPinToInterrupt(receiver),ISR_FSK_Receiver , RISING );
		//pinMode(9,OUTPUT);
		//		pinMode(13,OUTPUT);
		//digitalWrite(9,LOW );
		//		digitalWrite(13,LOW );
	}
	
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2  = 0;//initialize counter value to 0
	presc = 62500 / (baud *0.9); //calculate best prescaler. 62500 = 16000000/256, 10% tolerance calculated
	if (presc<=8){
		TCCR2B |= (1 << CS21); 
		lowFreq = 2000000 /  baud ; // 16000000/8
		hiFreq = 1081081 / baud ; // hi freq = 1.85 * lo freq
		}
	else if (presc<=32) {
		TCCR2B |= (1 << CS20)|(1 << CS21); 
		lowFreq = 500000 /  baud ; // 16000000/32
		hiFreq = 270270 / baud ; // hi freq = 1.85 * lo freq
		}		
	else if (presc<=64) {
		TCCR2B |= (1 << CS22); 
		lowFreq = 250000 /  baud ; // 16000000/64
		hiFreq = 135135 / baud ; // hi freq = 1.85 * lo freq
		}
	else if (presc<=128) {
		TCCR2B |= (1 << CS20)|(1 << CS22); 
		lowFreq = 125000 /  baud ; // 16000000/128
		hiFreq = 67567 / baud ; // hi freq = 1.85 * lo freq
		}		
	else if (presc<=256) {
		TCCR2B |= (1 << CS21)|(1 << CS22);
		lowFreq = 62500 /  baud ; // 16000000/256
		hiFreq = 33784 / baud ; // hi freq = 1.85 * lo freq
		}		
	else if (presc<=1024) {
	  TCCR2B |= (1 << CS22) |(1 << CS21)| (1 << CS20);  
		lowFreq = 15625 /  baud ; // 16000000/1024
		hiFreq = 8446 / baud  ; // hi freq = 1.85 * lo freq
		}

		
	lowFreqTolerance=lowFreq * 0.1;
	hiFreqTolerance = hiFreq * 0.1;
	
//	TCCR2A |= (1 << WGM21); // CTC mode
	TIMSK2 = (1 << TOIE2); // enable interrupt


}

void FSKModem::receive()
{
	byte act, nextBufptr;
	int elapsedCount;
	boolean validBit;
	//digitalWrite(13,digitalRead(13)^1 );
	//	digitalWrite(9,LOW );
		//digitalWrite(13,LOW );
	if (_inReceive)
	{
		//digitalWrite(10,LOW );
		elapsedCount = TCNT2;
		validBit = true;
		if (elapsedCount > lowFreq + lowFreqTolerance) { // signal lost, reset receiving
			validBit= false;
			_bitReceive = 0;
			TCNT2 = 0;			
			//digitalWrite(13,HIGH );
		} else if (abs(elapsedCount-lowFreq)<lowFreqTolerance) { // logical 0 received
			act = 0;
		//	digitalWrite(9,LOW );
			TCNT2 = 0;
			//digitalWrite(13,HIGH );
		} else if (abs(elapsedCount-hiFreq)<hiFreqTolerance) { // logical 1 received
			act = 1;
			//digitalWrite(9,HIGH );
			//		digitalWrite(13,LOW );
			if (_bitReceive==0)
			{
				validBit=false;
			}
			TCNT2 = 0;
		} else {
			validBit= false;	//noise
			//TCNT2 = 0;
			//digitalWrite(9,HIGH );
		}
		if (validBit)
		{
			//digitalWrite(13,HIGH);
			//digitalWrite(13,_bitReceive & 1 );
			if (_bitReceive==0 && act ==0 ) // start bit
			{
				_byteReceive = 0;
				_bitReceive++;
				//digitalWrite(9,HIGH );
			} else if (_bitReceive==9) // stop bit
			{
				if (act == 1) // real stopbit
				{
					//digitalWrite(9,LOW );
					nextBufptr=(_recvEnd+1) & 15;
					if (_recvStart!=nextBufptr)
					{
						_recvBuffer[_recvEnd]=_byteReceive;
						_recvEnd = nextBufptr;		
					}
				}
				_bitReceive = 0;
				//digitalWrite(13,LOW );
			}
			
			else {
				_byteReceive = (_byteReceive << 1) | act;
				_bitReceive++;	
				//digitalWrite(9,HIGH );
			}
		}		
	} else {
		//digitalWrite(13,LOW );
		_inReceive = true;
		_bitReceive = 0;
		TCNT2 = 0;
	}
}

byte FSKModem::read()
{
	byte act;
	if (_recvStart!=_recvEnd)
	{
		act=_recvBuffer[_recvStart];
		_recvStart = (_recvStart+1) & 15;
		return act;
	}
}

void FSKModem::getIntOvf()
{
	_inReceive = false;  
	if (_pin!=0)
	{
		if (_sendStart!=_sendEnd)
		{
			_byteSend=_sendBuffer[_sendStart];
			_sendStart= (_sendStart + 1) & 15;
			_bitSend=40;
			OCR2A = hiFreq;
			OCR2B = hiFreq/2;
			TCCR2A |= (1 << WGM21); 
			TIMSK2 = (1 << OCIE2B )|( 1 << OCIE2A);
//			TCCR2A |= (1 << WGM21); 
			digitalWrite(_pin,HIGH );
//			digitalWrite(_pin+1,digitalRead(_pin+1)^1 ); 
		}
	}

}

void FSKModem::send()
{
	//TCNT2  = 0;
	if (_bitSend==250)
	{
		TIMSK2 = (1 << TOIE2);
		TCCR2A &= ~(1 << WGM21);
	} else 
	{
	if (_bitSend>10)
	{
	//		digitalWrite(_pin+1,digitalRead(_pin+1)^1 ); 
			digitalWrite(_pin,HIGH );
			if (_bitSend!=20){
			OCR2A = hiFreq;
			OCR2B = hiFreq/2;	
			} else {
			OCR2A = lowFreq;
			OCR2B = lowFreq/2;	
			}
//			digitalWrite(_pin,HIGH );
			_bitSend-=10;		
	} else
	{
		if (_bitSend<8) 
		{
			digitalWrite(_pin,HIGH );
			if ((_byteSend & 128) == 128)
			{
//				digitalWrite(_pin+1,digitalRead(_pin+1)^1 ); // toggle LED pin
				OCR2A = hiFreq;
				OCR2B = hiFreq/2;	
			} else 
			{
				OCR2A = lowFreq;
				OCR2B = lowFreq/2;				
			}	
			_byteSend=_byteSend << 1;
			_bitSend++;
		} else
		{
			if (_bitSend==8)
			{
				digitalWrite(_pin,HIGH );
				OCR2A = hiFreq;
				OCR2B = hiFreq/2;
				_bitSend++;			
			} else
			{
				
				if (_sendStart!=_sendEnd)
				{
					_byteSend=_sendBuffer[_sendStart];
					_sendStart= (_sendStart + 1) & 15;
					_bitSend=0;
					digitalWrite(_pin,HIGH );
					OCR2A = lowFreq;
					OCR2B = lowFreq/2;	
				} else
				{
					digitalWrite(_pin,HIGH );
					OCR2A = hiFreq;
					OCR2B = hiFreq/2;	
					_bitSend=250;
				}
			}
		}
	}
	}
}

void FSKModem::send2()
{
//	if (TCNT2!=0)
//	{
	digitalWrite(_pin,LOW);
//	digitalWrite(_pin+1,digitalRead(_pin+1)^1 ); 
//	}
}

void FSKModem::write(byte out)
{
	byte stepup;
	stepup= (_sendEnd + 1) & 15;
	while ( stepup == _sendStart ) delay(100);
	_sendBuffer[_sendEnd]=out;
	_sendEnd=stepup;
}

int FSKModem::status(int param)
{
		return(_recvBuffer[param]);
}

int FSKModem::available()
{
		if (_recvStart>_recvEnd)
		{
			return _recvEnd+16-_recvStart;
		} else
		{
			return _recvEnd-_recvStart;
		}
}