/*
 * 
 * FIRMWARE SIMULATION FOR ARDUINO TO TEST THE INTERACTION WITH THE REMOTE XTH
 *  
 */


// #include <avr/interrupt.h>
// #include <avr/io.h>

 #include <TimerOne.h>

// uint8_t sintable[256] PROGMEM = { 127, 130, 133, 136, 139, 143, 146, 149, 152, 155, 158, 161, 164, 167, 170, 173, 176, 179, 182, 184, 187, 190, 193, 195, 198, 200, 203, 205, 208, 210, 213, 215, 217, 219, 221, 224, 226, 228, 229, 231, 233, 235, 236, 238, 239, 241, 242, 244, 245, 246, 247, 248, 249, 250, 251, 251, 252, 253, 253, 254, 254, 254, 254, 254, 255, 254, 254, 254, 254, 254, 253, 253, 252, 251, 251, 250, 249, 248, 247, 246, 245, 244, 242, 241, 239, 238, 236, 235, 233, 231, 229, 228, 226, 224, 221, 219, 217, 215, 213, 210, 208, 205, 203, 200, 198, 195, 193, 190, 187, 184, 182, 179, 176, 173, 170, 167, 164, 161, 158, 155, 152, 149, 146, 143, 139, 136, 133, 130, 127, 124, 121, 118, 115, 111, 108, 105, 102, 99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 70, 67, 64, 61, 59, 56, 54, 51, 49, 46, 44, 41, 39, 37, 35, 33, 30, 28, 26, 25, 23, 21, 19, 18, 16, 15, 13, 12, 10, 9, 8, 7, 6, 5, 4, 3, 3, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 15, 16, 18, 19, 21, 23, 25, 26, 28, 30, 33, 35, 37, 39, 41, 44, 46, 49, 51, 54, 56, 59, 61, 64, 67, 70, 72, 75, 78, 81, 84, 87, 90, 93, 96, 99, 102, 105, 108, 111, 115, 118, 121, 124 };

const uint16_t sintable[256] PROGMEM = { 512, 525, 537, 550, 562, 575, 587, 600, 612, 624, 636, 649, 661, 673, 684, 696, 708, 719, 731, 742, 753, 764, 775, 786, 796, 807, 817, 827, 837, 846, 856, 865, 874, 883, 891, 900, 908, 916, 923, 931, 938, 945, 951, 957, 964, 969, 975, 980, 985, 990, 994, 998, 1002, 1005, 1009, 1012, 1014, 1016, 1018, 1020, 1022, 1023, 1023, 1024, 1024, 1024, 1023, 1023, 1022, 1020, 1018, 1016, 1014, 1012, 1009, 1005, 1002, 998, 994, 990, 985, 980, 975, 969, 964, 957, 951, 945, 938, 931, 923, 916, 908, 900, 891, 883, 874, 865, 856, 846, 837, 827, 817, 807, 796, 786, 775, 764, 753, 742, 731, 719, 708, 696, 684, 673, 661, 649, 636, 624, 612, 600, 587, 575, 562, 550, 537, 525, 512, 499, 487, 474, 462, 449, 437, 424, 412, 400, 388, 375, 363, 351, 340, 328, 316, 305, 293, 282, 271, 260, 249, 238, 228, 217, 207, 197, 187, 178, 168, 159, 150, 141, 133, 124, 116, 108, 101, 93, 86, 79, 73, 67, 60, 55, 49, 44, 39, 34, 30, 26, 22, 19, 15, 12, 10, 8, 6, 4, 2, 1, 1, 0, 0, 0, 1, 1, 2, 4, 6, 8, 10, 12, 15, 19, 22, 26, 30, 34, 39, 44, 49, 55, 60, 67, 73, 79, 86, 93, 101, 108, 116, 124, 133, 141, 150, 159, 168, 178, 187, 197, 207, 217, 228, 238, 249, 260, 271, 282, 293, 305, 316, 328, 340, 351, 363, 375, 388, 400, 412, 424, 437, 449, 462, 474, 487, 499 };


#define XTH_STARTING 0
#define XTH_WAITING_ADDRESS 1
#define XTH_WAITING_CONFIG 2
#define XTH_SENDING 3

uint8_t status = XTH_STARTING;

uint8_t mmg_on = 0;
uint8_t mpu_on = 0;
uint8_t temp_on = 0;

uint16_t temperature = 808;

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
  
  uint8_t j = 4;
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

uint8_t samplecnt = 0;

// ISR( TIMER1_COMPA_vect ){
void update_mmg(){
  samplecnt++;
  newadc = sintable[ samplecnt ];  // read from sine table
  
  cur_time++; // running time counter, will wrap around
  cur_time = cur_time%20000;
  
  send_time++; // sending time interval
  
  // update time counters of each digital sensor
  temp_time++;
  mpu_time++;
    
//       hasnewMMG = 1;
//       mmg_adc = newadc;
//       mmgs[ mmg_index ] = mmg_adc;
  mmg_index++;
  mmg_index = mmg_index%MMGSAMPLES;
  mmg_pack( mmg_adc );
}

// void setup_timer1(){
//   TCCR1A |= 0;  
//   // not required since WGM11:0, both are zero (0)
//  
// //   TCCR1B |= (1 << WGM12)|(1 << CS11)|(1 << CS10);  
//   // Mode = CTC, Prescaler = 8
//   TCCR1B |= (1 << WGM12)|(1 << CS11);
//   OCR1A = 1000;   // timer compare value // 0.5 ms -> ADC runs at 1000 Hz
//   
//   // enable compare interrupt
//   TIMSK1 |= (1 << OCIE1A);
// }

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

uint8_t msgid_battery=0;
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

uint8_t msg_battery[7];

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

// uint8_t ds_valbytes[] = { 3, 40 };



uint8_t mpu_count;

uint8_t mpu_bytes[19];

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

void make_mpu_fake(){
    mpu_count++;
    
//     long_to_mpubytes( 1, 0 );
//     long_to_mpubytes( 2, 4 );
//     long_to_mpubytes( 3, 8 );
//     long_to_mpubytes( 4, 12 );
//     short_to_mpubytes( 5, 16 );
//     short_to_mpubytes( 6, 18 );
//     short_to_mpubytes( 7, 20 );
//     short_to_mpubytes( 8, 22 );
//     short_to_mpubytes( 9, 24 );
//     short_to_mpubytes( 10, 26 );
//     short_to_mpubytes( 11, 28 );
//     short_to_mpubytes( 12, 30 );
//     short_to_mpubytes( 13, 32 );

    short_to_mpubytes( 5, 1 ); //x
    short_to_mpubytes( 6, 3 ); //y
    short_to_mpubytes( 7, 5 ); //z
    short_to_mpubytes( 8, 7 );  //x 
    short_to_mpubytes( 9, 9 );  //y
    short_to_mpubytes( 10, 11 );  //z
    short_to_mpubytes( 11, 13 );  //x
    short_to_mpubytes( 12, 15 );  //y
    short_to_mpubytes( 13, 17 );  //z
    
//     mpu_bytes[34] = mpu_count;
}

///-------- sendBattery (battery and temperature)

void send_battery_data(){
    msgid_battery++; // increase message counter
    msg_battery[0] = msgid_battery;
    msg_battery[1] = (cur_time/256);
    msg_battery[2] = (cur_time%256);
    msg_battery[3] = (battery/256);
    msg_battery[4] = (battery%256);
    if ( temp_on ){
      msg_battery[5] = (temperature/256);
      msg_battery[6] = (temperature%256);
      fake_send_data( 7, 7, msg_battery );
    } else {
      fake_send_data( 7, 5, msg_battery );
    }    
}
  
void send_mmg_data(){
    msgid_mmg++; // increase message counter
    msg_mmg[0] = msgid_mmg;
    msg_mmg[1] = (cur_time/256);
    msg_mmg[2] = (cur_time%256);
//     msg_mmg[3] = (battery/256);
//     msg_mmg[4] = (battery%256);
    msg_mmg[5] = prev_mmg_index;
    msg_mmg[6] = mmg_index;
    prev_mmg_index = mmg_index;
    for (uint8_t j=0; j<MMGPACKEDSIZE; j++ ){
	msg_mmg[j+7] = mmgs_packed[j];
    }
    
    fake_send_data( 1, MMGPACKEDSIZE + 7, msg_mmg );
//  sendSerial( S_MMG, msg_mmg, 7+MMGPACKEDSIZE );
//     radio_send_data( 7+MMGPACKEDSIZE, msg_mmg, 1 );
}

void send_mpu_data(){
//     mpu_bytes[34] = mpu_count;
    mpu_bytes[0] = mpu_count;
    msgid_mpu++; // increase message counter
    msg_mpu[0] = msgid_mpu;
    msg_mpu[1] = (cur_time/256);
    msg_mpu[2] = (cur_time%256);
//     msg_mpu[3] = (battery/256);
//     msg_mpu[4] = (battery%256);;
//     msg_mpu[5] = ds_valbytes[1];
//     msg_mpu[6] = ds_valbytes[0];
    for (uint8_t j=0; j<19; j++ ){
	msg_mpu[j+3] = mpu_bytes[j];
    }
    fake_send_data( 2, 22, msg_mpu );
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
  make_mpu_fake();
//   setup_mmg_packed();
//   setup_timer1();
  
  // enable global interrupts
//   sei();
  Timer1.initialize( 1000 ); // in microseconds
  Timer1.attachInterrupt( update_mmg );
}

void loop() {    
//   update_mmg();
  
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
// 	make_mpu_fake();
	send_mpu_data();
    }
    
    if ( temp_time > TEMP_INTERVAL ){
      temp_time = 0;
      send_battery_data();
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
