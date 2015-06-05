/*
 * 
 * FIRMWARE SIMULATION FOR ARDUINO TO TEST THE INTERACTION WITH THE REMOTE XTH
 * 
 * mmg data
 *
 */

volatile int cur_time = 0;
uint16_t battery = 800;

// <<<================ serial protocol ==============

// escape : 92; delimiter : 10; cr : 13
#define ESC_CHAR '\\'
#define DEL_CHAR '\n'
#define CR '\r'

#define S_NO_MSG '0'
#define S_MMG 'M'
#define S_MPU 'm'

#define S_SER 'S'
#define S_ADDR 'A'
#define S_CONF 'C'

#define S_LED 'L'

#define S_LOOP 'l'

uint8_t responseState = S_NO_MSG;

// incoming message
uint8_t byte_index=0;
uint8_t escaping = 0;
uint8_t message[50];
char msg_type = S_NO_MSG;

void sendSerial(char type, uint8_t *p, uint8_t size) {
	Serial.write(ESC_CHAR);
	Serial.write(type);
	for(uint8_t i = 0;i < size;i++) slipSerial(p[i]);
	Serial.write(DEL_CHAR);
}

void sendSerial(char type, char *p, uint8_t size) {
	Serial.write(ESC_CHAR);
	Serial.write(type);
	for(uint8_t i = 0;i < size;i++) slipSerial(p[i]);
	Serial.write(DEL_CHAR);
}

void slipSerial(char c) {
	if((c == ESC_CHAR) || (c == DEL_CHAR) || (c == CR))
	    Serial.write(ESC_CHAR);
	Serial.write(c);
}

void readSerial(void){
  uint16_t bytestoread;
  bytestoread = Serial.available();
  if ( bytestoread > 0 ){
    for ( uint8_t i = 0; i < bytestoread; i++ ){
      readSerialByte();      
    }
  }
}

void readSerialByte() {
	char incoming = Serial.read();
	if(escaping) {	//escape set
		if((incoming == ESC_CHAR)  || (incoming == DEL_CHAR) || (incoming == CR)) {	//escape to integer
			if ( msg_type != S_NO_MSG ){ // only add if message type set
			  message[byte_index] = incoming;
			  byte_index++;
			}
		} else {	//escape to char
			msg_type = incoming;
		}
		escaping = false;
	} else {	//escape not set
		if(incoming == ESC_CHAR) {
			escaping = true;
		} else if(incoming == DEL_CHAR) {	//end of msg
			message[byte_index] = '\0'; // null-termination
			routeSerialMsg(msg_type, message, byte_index);	//route completed message
			msg_type = S_NO_MSG;
			byte_index = 0;	//reset buffer index
		} else {
			if ( msg_type != S_NO_MSG ){ // only add if message type set
			  message[byte_index] = incoming; 
			  byte_index++;
			}
		}
	}
	if ( byte_index > 50 ){
	    msg_type = S_NO_MSG;
	    byte_index = 0;
	}
}

void routeSerialMsg(char type, uint8_t *msg, uint8_t size) {
// 	uint8_t len;
	uint8_t * ser;
	uint8_t * addr;
	
	uint16_t target;

	digitalWrite( 13, 1 );
	
	sendSerial( S_LOOP, msg, size );
	
// 	switch(type) {
// // 		case S_LED:
// // 		      if ( size >= 5 ){
// // 			// set led value
// // 			target = msg[0]*256 + msg[1];
// // 			send_led_value( &(msg[2]), target );
// // 		      }
// // 		      break;
// 		case S_SER:
// 			// request serial number
// 			fake_send_serialnumber();
// 			break;
// 		case S_ADDR:
// 			// set address for serial number
// 			if ( size >= 10 ){
// 			  ser = msg;
// 			  addr = &(msg[8]);
// 			  send_address( ser, addr );
// 			}
// 			break;
// 		case S_CONF:
// 			// set configuration
// 			if ( size >= 5 ){
// 			  target = msg[0]*256 + msg[1];
// 			  send_configuration( msg[2], msg[3], msg[4], target );
// 			}
// 			break;
// 	}
}

// ================ serial protocol ==============>>>

#define RADIO_ADDRESS 13
#define RADIO_ADDRESS_DEST 7

uint16_t radioAddress = RADIO_ADDRESS;
uint16_t newRadioAddress;



// <<<================ radio ========================


typedef struct NWK_DataInd_t
{
  uint16_t     srcAddr;
  uint16_t     dstAddr;
  uint8_t      srcEndpoint;
  uint8_t      dstEndpoint;
  uint8_t      options;
  uint8_t      *data;
  uint8_t      size;
  uint8_t      lqi;
  int8_t       rssi;
} NWK_DataInd_t;

bool mmgCallback(NWK_DataInd_t *ind);


// packed size of 80 is 64 samples
#define MMGPACKEDSIZE 80
#define MMGSAMPLES 64


uint8_t msgid_mmg=0;
uint8_t msg_mmg[88];

uint8_t msg_mmg_out[91]; // 87 plus lqi (1), rssi (1), srcAddr (2)

volatile uint16_t mmgs[MMGSAMPLES];
volatile uint8_t mmg_index = 0;
uint8_t prev_mmg_index = 0;

volatile uint8_t mmgs_packed[MMGPACKEDSIZE];


/// --- radio networking -------

bool mmgCallback(NWK_DataInd_t *ind) { // called when receiving data
  digitalWrite( 13, 1 );

  msg_mmg_out[0] = ind->srcAddr / 256;
  msg_mmg_out[1] = ind->srcAddr%256;
  msg_mmg_out[2] = ind->lqi;
  msg_mmg_out[3] = ind->rssi;
  
  uint8_t j = 4;
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_mmg_out[j] = ind->data[i];
    j++;
  }  
  sendSerial( S_MMG, msg_mmg_out, ind->size + 4 );
//   for ( uint8_t i = 0; i < MMGPACKEDSIZE; i++ ){  
//     msg_mmg_out[j] = mmgs_packed[i];
//     j++;
//   }  
//   sendSerial( S_MMG, msg_mmg_out, MMGPACKEDSIZE + 4 );

  return true;
}

void fake_send_data( uint8_t endpoint, uint8_t size, uint8_t * data ){
  NWK_DataInd_t ind;
  ind.srcAddr = radioAddress;
  ind.dstAddr = RADIO_ADDRESS_DEST;
  ind.srcEndpoint = endpoint;
  ind.dstEndpoint = endpoint;
  ind.options = 0; // CHECK
  ind.lqi = 10;
  ind.rssi = 16;
  
  ind.size = size;
  ind.data = data;
  
  switch( endpoint ){
    case 1:
      mmgCallback( &ind );
      break;
//     case 2:
//       mpuCallback( &ind );
//       break;
//     case 4:
//       serialCallback( &ind );
//       break;
//     case 5:
//       addressCallback( &ind );
//       break;
//     case 6:
//       confCallback( &ind );
//       break;
  }
}
// ================== fake data ==================>>>



volatile uint8_t mmg_block_lowindex = 0; // which block we are
volatile uint8_t mmg_block_highindex = 4; // which block we are
// volatile uint8_t mmg_block_subindex = 0; // 0, 1, 2, 3
volatile uint8_t mmg_block_subshift = 0; // 0, 2, 4, 6

// quintuples of:
// lowbyte 0 , lowbyte 1, lowbyte 2, lowbyte 3, {highbyte 3, highbyte 2, highbyte 1, highbyte 0}
volatile uint8_t lowbyte;
volatile uint8_t highbyte;
inline void mmg_pack( uint16_t newvalue ){
    lowbyte = (uint8_t) newvalue%256;
    highbyte = (uint8_t) (newvalue>>8);
    highbyte = highbyte & (0b00000011);
    mmgs_packed[ mmg_block_lowindex ] = lowbyte; // lowbyte; index + sub_index
    mmgs_packed[ mmg_block_highindex ] |= (highbyte << mmg_block_subshift);
    mmg_block_lowindex++; // plus one
    mmg_block_subshift += 2;
    if ( mmg_block_subshift > 6 ){
      mmg_block_lowindex++; // additional plus one
      mmg_block_highindex += 5;
      if ( mmg_block_highindex > MMGPACKEDSIZE ){
	mmg_block_lowindex = 0;
	mmg_block_highindex = 4;
      }
      mmgs_packed[ mmg_block_highindex ] = 0;
//       mmg_block_subindex = 0;
      mmg_block_subshift = 0;
    }
}




union bits2byte
{
  struct{
    byte lo4: 2;
    byte lo3: 2;
    byte lo2: 2;
    byte lo1: 2;
  } bits2;
  byte lobyte;
};

union inttobytes
{
  struct{
    byte lo;
    byte hi;
  } parts;
  uint16_t myint;
};


uint8_t index = 0;
uint8_t lo_index = 4;
uint8_t lo_shift = 0;

bits2byte currentlowbyte;

// [ highbyte 1, hb 2, hb 3, hb4 , [lo1, lo2, lo3, l4]
void pack( uint16_t value ){
  inttobytes unionvalue;
  unionvalue.myint = (value << 6); // bitshift 6
  mmgs_packed[index] = unionvalue.parts.hi; // high byte;
  currentlowbyte.lobyte |= ( (unionvalue.parts.lo & 0xC0) >> lo_shift);
  mmgs_packed[lo_index] = currentlowbyte.lobyte;

  index++;
  lo_shift += 2;
  if ( lo_shift > 6 ){
      index++; // jump over the low byte
      lo_index += 5;
      lo_index = lo_index%MMGPACKEDSIZE;
      lo_shift = 0;
      currentlowbyte.lobyte = 0; // this means that it will be reset
  }
  index = index % MMGPACKEDSIZE;
}






void send_mmg_data(){
    msgid_mmg++; // increase message counter
    msg_mmg[0] = msgid_mmg;
    msg_mmg[1] = (cur_time/256);
    msg_mmg[2] = (cur_time%256);
    msg_mmg[3] = (battery/256);
    msg_mmg[4] = (battery%256);
    msg_mmg[5] = prev_mmg_index;
    msg_mmg[6] = mmg_index;


// TEST FILL OF PACKED BUFFER. THESE REPRESENT BYTES, NOT SAMPLES
//    for (uint8_t j=0; j<MMGPACKEDSIZE; j++ ){
//	mmgs_packed[j] = j;
//    }
    


    for (uint8_t j=0; j<MMGPACKEDSIZE; j++ ){
	msg_mmg[j+7] = mmgs_packed[j];
    }
    prev_mmg_index = mmg_index;
    
    fake_send_data( 1, MMGPACKEDSIZE + 7, msg_mmg );
//  sendSerial( S_MMG, msg_mmg, 7+MMGPACKEDSIZE );
//     radio_send_data( 7+MMGPACKEDSIZE, msg_mmg, 1 );
}


// FIXME: put in a nice waveform here in stead of a counting up to simulate a sound source:
void setup_mmgs(){
  for (uint8_t j=0; j<MMGSAMPLES; j++ ){
    //mmgs[j] = j;
    // a single cosine cycle in 10 bit resolution
    mmgs[j] = (((cos(3*2*PI*((float) j/MMGSAMPLES))) + 1) * 0.5)*1023;
  }
}


void setup_mmg_packed(){
  for (uint8_t j=0; j<MMGSAMPLES; j++ ){
//    mmg_pack( mmgs[j] );
    pack( mmgs[j] );

  }
}


/// ---------- MAIN SETUP -------


void setup() {
  Serial.begin( 115200, SERIAL_8N1 );
  pinMode( 13, OUTPUT );
  setup_mmgs();
  setup_mmg_packed();
}


void loop() {
  cur_time++;
  
  digitalWrite( 13, 0 );
  readSerial();

  send_mmg_data();
  
  delay(32);
}
