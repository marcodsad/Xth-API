/*
 * 
 * FIRMWARE SIMULATION FOR ARDUINO TO TEST THE INTERACTION WITH THE REMOTE XTH
 *  
 */


#include <avr/interrupt.h>
#include <avr/io.h>

#define XTH_STARTING 0
#define XTH_WAITING_ADDRESS 1
#define XTH_WAITING_CONFIG 2
#define XTH_SENDING 3

uint8_t status = XTH_STARTING;

uint8_t mmg_on = 0;
uint8_t mpu_on = 0;
uint8_t temp_on = 0;

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
	
	switch(type) {
// 		case S_LED:
// 		      if ( size >= 5 ){
// 			// set led value
// 			target = msg[0]*256 + msg[1];
// 			send_led_value( &(msg[2]), target );
// 		      }
// 		      break;
		case S_SER:
			// request serial number
			send_serial_request();
			break;
		case S_ADDR:
			// set address for serial number
			if ( size >= 10 ){
			  ser = msg;
			  addr = &(msg[8]);
			  send_address( ser, addr );
			}
			break;
		case S_CONF:
			// set configuration
			if ( size >= 5 ){
			  target = msg[0]*256 + msg[1];
			  send_configuration( msg[2], msg[3], msg[4], target );
			}
			break;
	}
}


// ================ serial protocol ==============>>>
#define RADIO_ADDRESS 13
#define RADIO_ADDRESS_DEST 7

uint16_t radioAddress = RADIO_ADDRESS;
uint16_t newRadioAddress;



#define RADIO_PANID 0x3456
#define RADIO_CHANNEL 20


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
bool mpuCallback(NWK_DataInd_t *ind);

bool serialCallback(NWK_DataInd_t *ind);
bool addressCallback(NWK_DataInd_t *ind);
bool confCallback(NWK_DataInd_t *ind);

/// --- radio networking -------

uint8_t msg_mpu_out[90];
uint8_t msg_mmg_out[91]; // 87 plus lqi (1), rssi (1), srcAddr (2)

uint8_t msg_serial_out[12]; // 4 + 8
uint8_t msg_address_out[12];
uint8_t msg_conf_out[7];


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

  return true;
}

bool mpuCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 13, 1 );

  msg_mpu_out[0] = ind->srcAddr / 256;
  msg_mpu_out[1] = ind->srcAddr%256;
  msg_mpu_out[2] = ind->lqi;
  msg_mpu_out[3] = ind->rssi;
  
  uint8_t j = 4;
   
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_mpu_out[j] = ind->data[i];
    j++;
  }
  sendSerial( S_MPU, msg_mpu_out, ind->size + 4 );

  return true;
}


bool serialCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 13, 1 );

  msg_serial_out[0] = ind->srcAddr / 256;
  msg_serial_out[1] = ind->srcAddr%256;
  msg_serial_out[2] = ind->lqi;
  msg_serial_out[3] = ind->rssi;

//  msg_serial_out[0] = 100;
//  msg_serial_out[1] = 101;
//  msg_serial_out[2] = 102;
//  msg_serial_out[3] = 103;
  
  uint8_t j = 4;

// CHECK HERE
   for ( uint8_t i = 0; i < 8; i++ ){  
     ind->data[i] = 16+i;
   }  
   
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_serial_out[j] = ind->data[i];
    j++;
  }
  sendSerial( S_SER, msg_serial_out, ind->size + 4 );

  return true;
}

bool addressCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 13, 1 );

  msg_address_out[0] = ind->srcAddr / 256;
  msg_address_out[1] = ind->srcAddr%256;
  msg_address_out[2] = ind->lqi;
  msg_address_out[3] = ind->rssi;
  
  uint8_t j = 4;
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_address_out[j] = ind->data[i];
    j++;
  }
  sendSerial( S_ADDR, msg_address_out, ind->size + 4 );

  return true;
}

bool confCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 13, 1 );

  msg_conf_out[0] = ind->srcAddr / 256;
  msg_conf_out[1] = ind->srcAddr%256;
  msg_conf_out[2] = ind->lqi;
  msg_conf_out[3] = ind->rssi;
  
  uint8_t j = 4;
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_conf_out[j] = ind->data[i];
    j++;
  }
  sendSerial( S_CONF, msg_conf_out, ind->size + 4 );

  return true;
}
// <<<=============== fake data =====================

// volatile uint8_t adclow = 0;
// volatile uint8_t adchigh = 0;
volatile uint16_t newadc = 0;
// volatile uint8_t hasnewADC = 0;
// volatile uint8_t curADC = 0;
// volatile uint8_t restart_adc = 0;

volatile uint16_t temp_time = 10; // slight offset to avoid clashes
volatile uint16_t mpu_time = 10; // slight offset to avoid clashes
volatile uint16_t send_time = 0;
// volatile uint16_t adc_time = 0;

// volatile uint16_t cur_time = 0;
// volatile uint8_t cur_time = 0;
volatile int cur_time = 0;

volatile uint16_t mmg_adc = 0;
volatile uint16_t batt_adc = 0;

// packed size of 80 is 64 samples
#define MMGPACKEDSIZE 80
#define MMGSAMPLES 64

volatile uint8_t hasnewMMG = 0;
volatile uint16_t mmgs[MMGSAMPLES];
volatile uint8_t mmg_index = 0;
uint8_t prev_mmg_index = 0;

volatile uint8_t mmgs_packed[MMGPACKEDSIZE];
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


// ISR( TIMER1_COMPA_vect ){
void update_mmg(){
  newadc++; // just increase counter to fake data
  newadc = newadc%256;
  
  cur_time++; // running time counter, will wrap around
  cur_time = cur_time%20000;
  
  send_time++; // sending time interval
  
  // update time counters of each digital sensor
  temp_time++;
  mpu_time++;
    
//       hasnewMMG = 1;
//       mmg_adc = newadc;
//       mmgs[ mmg_index ] = mmg_adc;
//       mmg_index++;
//       mmg_index = mmg_index%MMGSAMPLES;
//       mmg_pack( mmg_adc );
}

void setup_timer1(){
  TCCR1A |= 0;  
  // not required since WGM11:0, both are zero (0)
 
//   TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);  
  // Mode = CTC, Prescaler = 8
  TCCR1B |= (1 << WGM12)|(1 << CS11);
  OCR1A = 1000;   // timer compare value // 0.5 ms -> ADC runs at 1000 Hz
  
  // enable compare interrupt
  TIMSK1 |= (1 << OCIE1A);
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
    case 2:
      mpuCallback( &ind );
      break;
    case 4:
      serialCallback( &ind );
      break;
    case 5:
      addressCallback( &ind );
      break;
    case 6:
      confCallback( &ind );
      break;
  }
}
// ================== fake data ==================>>>

uint8_t msgid_mpu=0;
// uint8_t msgid_mmg=0;

uint8_t msgid_mmg=0;
uint8_t msg_mmg[88];
uint8_t msg_mpu[86];


uint8_t msg_serial[9];

/// ---------- OUTPUT MESSAGES -------

uint8_t msgid_led = 0;
uint8_t msgid_serial = 0;
uint8_t msgid_conf = 0;
uint8_t msgid_address = 0;

uint8_t msg_led[7];
uint8_t msg_conf[4];
uint8_t msg_address[10];

void send_led_value( uint8_t * inmsg, uint16_t target ){
    msgid_led++; // increase message counter
    msg_led[0] = msgid_led;
    for ( uint8_t i=0; i<6; i++ ){
      msg_led[1+i] = inmsg[i];
    }
//     radio_send_data( 7, msg_led, 3, target );
}

void send_serial_request(){
    msgid_serial++; // increase message counter
    // fake response of serial number:
    fake_send_serialnumber();
//     radio_send_data( 1, &msgid_serial, 4 );
}

void send_address( uint8_t * serial, uint8_t * address ){
    msgid_address++; // increase message counter
    msg_address[0] = msgid_address;
    for ( uint8_t i=0; i<8; i++ ){
	msg_address[i+1] = serial[i];
    }
//     msg_address[9] = address[0];
//     msg_address[10] = address[1];
        
    fake_send_address();
//     radio_send_data( 11, msg_address, 5 );
}

void send_configuration( uint8_t enable_mmg, uint8_t enable_mpu, uint8_t enable_temperature, uint16_t target ){
    msgid_conf++; // increase message counter
    msg_conf[0] = msgid_conf;
    msg_conf[1] = enable_mmg;
    msg_conf[2] = enable_mpu;
    msg_conf[3] = enable_temperature;
    
    mmg_on = enable_mmg;
    mpu_on = enable_mpu;
    temp_on = enable_temperature;
    fake_send_configuration();
//     radio_send_data( 4, msg_conf, 6, target );
}


uint16_t SEND_INTERVAL = 100; // every 50 ms
uint16_t TEMP_INTERVAL = 2000; // every 1000 ms
// uint16_t MPU_INTERVAL = 50; // 50: every 25 ms // = 40 Hz // 25: every 12.5 ms // = 80 Hz
uint16_t MPU_INTERVAL = 100; // 100: every 50 ms // = 20 Hz
// uint16_t ADC_INTERVAL = 2; // 


uint16_t battery = 800;

uint16_t nowtime;

uint8_t ds_valbytes[] = { 3, 40 };



uint8_t mpu_count;

uint8_t mpu_bytes[70];

void long_to_mpubytes( long value, uint8_t offset ){  
    long myvalue = value;
    for ( uint8_t i=0; i<4; i++ ){
      mpu_bytes[offset] = myvalue%256;
      myvalue = myvalue >> 8;
      offset++;
    }
}

void short_to_mpubytes( short value, uint8_t offset ){  
    short myvalue = value;
    mpu_bytes[offset] = myvalue%256;
    offset++;
    mpu_bytes[offset] = myvalue>>8;
}

void setup_mpu_fake(){
    mpu_count++;
    
    long_to_mpubytes( 1, 0 );
    long_to_mpubytes( 2, 4 );
    long_to_mpubytes( 3, 8 );
    long_to_mpubytes( 4, 12 );
    short_to_mpubytes( 5, 16 );
    short_to_mpubytes( 6, 18 );
    short_to_mpubytes( 7, 20 );
    short_to_mpubytes( 8, 22 );
    short_to_mpubytes( 9, 24 );
    short_to_mpubytes( 10, 26 );
    short_to_mpubytes( 11, 28 );
    short_to_mpubytes( 12, 30 );
    short_to_mpubytes( 13, 32 );
    
    mpu_bytes[34] = mpu_count;
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
    for (uint8_t j=0; j<MMGPACKEDSIZE; j++ ){
	msg_mmg[j+7] = mmgs_packed[j];
    }
    prev_mmg_index = mmg_index;
    
    fake_send_data( 1, MMGPACKEDSIZE + 7, msg_mmg );
//  sendSerial( S_MMG, msg_mmg, 7+MMGPACKEDSIZE );
//     radio_send_data( 7+MMGPACKEDSIZE, msg_mmg, 1 );
}

void send_mpu_data(){
    mpu_bytes[34] = mpu_count;
  
    msgid_mpu++; // increase message counter
    msg_mpu[0] = msgid_mpu;
    msg_mpu[1] = (cur_time/256);
    msg_mpu[2] = (cur_time%256);
    msg_mpu[3] = (battery/256);
    msg_mpu[4] = (battery%256);;
    msg_mpu[5] = ds_valbytes[1];
    msg_mpu[6] = ds_valbytes[0];
    for (uint8_t j=0; j<35; j++ ){
	msg_mpu[j+7] = mpu_bytes[j];
    }
    fake_send_data( 2, 42, msg_mpu );
//       sendSerial( S_MPU, msg_mpu, 42 );
//     radio_send_data( 42, msg_mpu, 2 );  
}

void fake_send_serialnumber(){
//   readMacAddress64( 0, mymac64 );  //FIXME
  msgid_serial++;
  msg_serial[0] = msgid_serial;
//   for ( uint8_t i=0; i<8; i++ ){
//     msg_serial[i+1] = mymac64[i];
//   }

//   msg_serial[1] = 0x0F;
//   msg_serial[2] = 0x1E;
//   msg_serial[3] = 0x2D;
//   msg_serial[4] = 0x3C;
//   msg_serial[5] = 0x4B;
//   msg_serial[6] = 0x5A;
//   msg_serial[7] = 0x69;
//   msg_serial[8] = 0x78;
  
  msg_address[1] = 1;
  msg_address[2] = 2;
  msg_address[3] = 3;
  msg_address[4] = 4;
  msg_address[5] = 5;
  msg_address[6] = 6;
  msg_address[7] = 7;
  msg_address[8] = 8;

  
  fake_send_data( 4, 9, msg_serial );
  
//   radio_send_data( 9, msg_serial, 4 );
  responseState = S_NO_MSG;
  status = XTH_WAITING_ADDRESS;
}

void fake_send_address(){
//   readMacAddress64( mymac64 );  
  msgid_address++;
  msg_address[0] = msgid_address;
//   for ( uint8_t i=0; i<8; i++ ){
//     msg_address[i+1] = mymac64[i];
//   }
  
  msg_address[1] = 1;
  msg_address[2] = 2;
  msg_address[3] = 3;
  msg_address[4] = 4;
  msg_address[5] = 5;
  msg_address[6] = 6;
  msg_address[7] = 7;
  msg_address[8] = 8;
  
  fake_send_data( 5, 9, msg_address );

//   radio_send_data( 9, msg_address, 5 );
  responseState = S_NO_MSG;
  status = XTH_WAITING_CONFIG;
}

void fake_send_configuration(){
  msgid_conf++;
  msg_conf[0] = msgid_conf;
  msg_conf[1] = mmg_on;
  msg_conf[2] = mpu_on;
  msg_conf[3] = temp_on;
  
  fake_send_data( 6, 4, msg_conf );

//   radio_send_data( 4, msg_conf, 6 );
  responseState = S_NO_MSG;
  status = XTH_SENDING;
}

void setup_mmg_packed(){
  for (uint8_t j=0; j<MMGPACKEDSIZE; j++ ){
    mmgs_packed[j] = j;
  }
}

/// ---------- MAIN SETUP -------

void setup() {
//   SYS_Init();
  Serial.begin( 115200, SERIAL_8N1 );
  pinMode( 13, OUTPUT );  
//   radio_setup();
  setup_mpu_fake();
  setup_mmg_packed();
//   setup_timer1();
  
  // enable global interrupts
//   sei();
}

void loop() {    
  update_mmg();
  
  if ( send_time%4 == 0 ){
    readSerial();
    digitalWrite( 13, 0 );
  }
//   if ( send_time%50 == 0 ){ // every 25 ms
//     check_power();
//   }
  
  if( status == XTH_SENDING ){
    if ( mmg_on && (send_time > SEND_INTERVAL) ){
      send_time = 0;
//       battery = batt_adc;
      send_mmg_data();
    }

    if ( mpu_on && (mpu_time > MPU_INTERVAL) ){
	mpu_count++;
	mpu_time = 0;
	send_mpu_data();
    }
    
  } else {
      switch( status ){
	case XTH_STARTING:
	case XTH_WAITING_ADDRESS:  
	  // send serial number
	  if (send_time > (SEND_INTERVAL*100) ){
	    send_time = 0;
	    fake_send_serialnumber();
	  }
	  break;
	case XTH_WAITING_CONFIG:
	  if (send_time > (SEND_INTERVAL*100) ){
	    send_time = 0;
	    fake_send_address();
	  }
	  break;
      }
  }
  
  if ( responseState != S_NO_MSG ){
    switch ( responseState ){
      case S_SER:
	// send serial number	
	fake_send_serialnumber();
	break;
      case S_ADDR:
	// send serial number from new address
	radioAddress = newRadioAddress;
// 	meshSetRadioAddress( radioAddress );
// 	meshSetRadio( radioAddress, RADIO_PANID, RADIO_CHANNEL );
	fake_send_address();
	break;
      case S_CONF:
	// send configuration confirmation
	fake_send_configuration();
	break;
    }
  }
  
  delay(1);
}
