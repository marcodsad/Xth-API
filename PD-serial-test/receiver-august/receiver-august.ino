// #include <OneWire.h>

// #define BLINK_INTERVAL 1000
// #define MMG_INTERVAL 5
// #define MPU_INTERVAL 100
// #define TEMPERATURE_INTERVAL 250
// #define ONBTEMPERATURE_INTERVAL 1000

#include <avr/interrupt.h>
#include <avr/io.h>

// <<<================ serial protocol ==============

// escape : 92; delimiter : 10; cr : 13
#define ESC_CHAR '\\'
#define DEL_CHAR '\n'
#define CR '\r'

#define S_NO_MSG '0'
#define S_MMG 'M'
#define S_MPU 'm'
#define S_BATTERY 'b'

#define S_SER 'S'
#define S_ADDR 'A'
#define S_CONF 'C'

#define S_LED 'L'

#define S_LOOP 'l'

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
	uint8_t len;
	uint8_t * ser;
	uint8_t * addr;
	
	uint16_t target;

	digitalWrite( 21, 1 );
	
	sendSerial( S_LOOP, msg, size );
	
	switch(type) {
		case S_LED:
		      if ( size >= 5 ){
			// set led value
			target = msg[0]*256 + msg[1];
			send_led_value( &(msg[2]), target );
		      }
		      break;
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

// <<<================ radio ========================


#include <lwm.h>
#include <lwm/phy/phy.h>
#include <lwm/hal/hal.h>
#include <lwm/sys/sys.h>
#include <lwm/nwk/nwk.h>

#include <lwm/nwk/nwkTx.h>
#include <lwm/sys/sysTimer.h> // for system timers

#include <lwm/phy/atmegarfr2.h>


#define RADIO_ADDRESS 13
#define RADIO_ADDRESS_DEST 7

#define RADIO_PANID 0x3456
#define RADIO_CHANNEL 20


// #define RADIO_TX_INTERVAL 50
#define RADIO_NR_DATA_REQS 100

static bool mmgCallback(NWK_DataInd_t *ind);
static bool mpuCallback(NWK_DataInd_t *ind);
static bool batteryCallback(NWK_DataInd_t *ind);

static bool serialCallback(NWK_DataInd_t *ind);
static bool addressCallback(NWK_DataInd_t *ind);
static bool confCallback(NWK_DataInd_t *ind);

static uint8_t txRetries = 0;
static void meshTxConfirm( NWK_DataReq_t *req );

static NWK_DataReq_t radioDataReqs[ RADIO_NR_DATA_REQS ];
uint8_t currentRadioReq = 0;

/// --- radio networking -------

void meshSetSecurityKey(const uint8_t *key) {
  NWK_SetSecurityKey((uint8_t *)key); // 16 bytes

//   for (int i=0; i<16; i++) {
//     eeprom_update_byte((uint8_t *)8162+i, key[i]);
//   }
}

void meshResetSecurityKey(void) {
  const uint8_t buf[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  meshSetSecurityKey(buf);
}

// endpoint is just an arbitrary number to look up which function to call / max is 16 / see nwk.h/.c
void meshListen(uint8_t endpoint, bool (*handler)(NWK_DataInd_t *ind)) {
  NWK_OpenEndpoint(endpoint, handler);
}

void meshSetRadio(const uint16_t theAddress, const uint16_t thePanId, const uint8_t theChannel) {
  NWK_SetAddr(theAddress);
//   address = theAddress;
  NWK_SetPanId(thePanId);
//   panId = thePanId;
  PHY_SetChannel(theChannel);
//   channel = theChannel;
  PHY_SetRxState(true);
  
  PHY_SetTxPower(0);
    /* Page 116 of the 256RFR2 datasheet
    0   3.5 dBm
    1   3.3 dBm
    2   2.8 dBm
    3   2.3 dBm
    4   1.8 dBm
    5   1.2 dBm
    6   0.5 dBm
    7  -0.5 dBm
    8  -1.5 dBm
    9  -2.5 dBm
    10 -3.5 dBm
    11 -4.5 dBm
    12 -6.5 dBm
    13 -8.5 dBm
    14 -11.5 dBm
    15 -16.5 dBm
  */
  TRX_CTRL_2_REG_s.oqpskDataRate = 0;
  /* Page 123 of the 256RFR2 datasheet
    0   250 kb/s  | -100 dBm
    1   500 kb/s  |  -96 dBm
    2   1000 kb/s |  -94 dBm
    3   2000 kb/s |  -86 dBm
  */
}


  // struct of ind (nwkRx.h)
//   typedef struct NWK_DataInd_t
// {
//   uint16_t     srcAddr;
//   uint16_t     dstAddr;
//   uint8_t      srcEndpoint;
//   uint8_t      dstEndpoint;
//   uint8_t      options;
//   uint8_t      *data;
//   uint8_t      size;
//   uint8_t      lqi;     /// probability of delivery: linearize with NWK_LinearizeLqi
//   int8_t       rssi;
// } NWK_DataInd_t;

char msg_mpu[90];
uint8_t msg_mmg[91]; // 87 plus lqi (1), rssi (1), srcAddr (2)
uint8_t msg_batt[12]; // 87 plus lqi (1), rssi (1), srcAddr (2)

uint8_t msg_serial_out[12]; // 4 + 8
uint8_t msg_address_out[12];
uint8_t msg_conf_out[7];


static bool mmgCallback(NWK_DataInd_t *ind) { // called when receiving data
  digitalWrite( 21, 1 );

  msg_mmg[0] = ind->srcAddr / 256;
  msg_mmg[1] = ind->srcAddr%256;
  msg_mmg[2] = ind->lqi;
  msg_mmg[3] = ind->rssi;
  
  uint8_t j = 4;
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_mmg[j] = ind->data[i];
    j++;
  }
  sendSerial( S_MMG, msg_mmg, ind->size + 4 );

  return true;
}

static bool mpuCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 21, 1 );

  msg_mpu[0] = ind->srcAddr / 256;
  msg_mpu[1] = ind->srcAddr%256;
  msg_mpu[2] = ind->lqi;
  msg_mpu[3] = ind->rssi;
  
  uint8_t j = 4;
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_mpu[j] = ind->data[i];
    j++;
  }
  sendSerial( S_MPU, msg_mpu, ind->size + 4 );

  return true;
}

static bool batteryCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 21, 1 );

  msg_batt[0] = ind->srcAddr / 256;
  msg_batt[1] = ind->srcAddr%256;
  msg_batt[2] = ind->lqi;
  msg_batt[3] = ind->rssi;
  
  uint8_t j = 4;
  for ( uint8_t i = 0; i < ind->size; i++ ){
    msg_batt[j] = ind->data[i];
    j++;
  }
  sendSerial( S_BATTERY, msg_batt, ind->size + 4 );

  return true;
}

static bool serialCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 21, 1 );

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

static bool addressCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 21, 1 );

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

static bool confCallback(NWK_DataInd_t *ind) { // called when receiving data
  
  digitalWrite( 21, 1 );

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

void radio_send_data( uint8_t len, uint8_t * data, uint8_t endpoint, uint16_t destination=RADIO_ADDRESS_DEST ){ 
  txRetries = 0;
  NWK_DataReq_t * radioDataReq = &(radioDataReqs[ currentRadioReq ]);
  
  radioDataReq->dstAddr = destination;
  radioDataReq->dstEndpoint = endpoint;
  radioDataReq->srcEndpoint = endpoint;
  radioDataReq->options = 0;
//   radioDataReq->options = NWK_OPT_ACK_REQUEST;
//   radioDataReq.options = NWK_OPT_ENABLE_SECURITY;
  radioDataReq->data = data; // set data!
  radioDataReq->size = len;
  
#ifdef DEBUG_WIRELESS
  radioDataReq->confirm = meshTxConfirm; // DEBUG only
#endif

  NWK_DataReq( radioDataReq ); // adds the request to the queue
  
  currentRadioReq++;
  currentRadioReq = currentRadioReq%RADIO_NR_DATA_REQS;
}

void radio_setup(){
  meshSetRadio( RADIO_ADDRESS , RADIO_PANID, RADIO_CHANNEL ); // address, pan id, channel
  meshListen( 1, mmgCallback );
  meshListen( 2, mpuCallback );

  meshListen( 4, serialCallback ); // serial number
  meshListen( 5, addressCallback ); // address confirmation
  meshListen( 6, confCallback ); // confirmation of config
  
  meshListen( 7, batteryCallback ); // battery/temperature data
  
  currentRadioReq = 0;
}


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
    radio_send_data( 7, msg_led, 3, target );
}

void send_serial_request(){
    msgid_serial++; // increase message counter
    radio_send_data( 1, &msgid_serial, 4 );
}

void send_address( uint8_t * serial, uint8_t * address ){
    msgid_address++; // increase message counter
    msg_address[0] = msgid_address;
    for ( uint8_t i=0; i<8; i++ ){
	msg_address[i+1] = serial[i];
    }
    msg_address[9] = address[0];
    msg_address[10] = address[1];
    radio_send_data( 11, msg_address, 5 );
}

void send_configuration( uint8_t enable_mmg, uint8_t enable_mpu, uint8_t enable_temperature, uint16_t target ){
    msgid_conf++; // increase message counter
    msg_conf[0] = msgid_conf;
    msg_conf[1] = enable_mmg;
    msg_conf[2] = enable_mpu;
    msg_conf[3] = enable_temperature;
    radio_send_data( 4, msg_conf, 6, target );
}

/// ---------- MAIN SETUP -------

void setup() {
  SYS_Init();
  Serial.begin( 115200, SERIAL_8N1 );
  pinMode( 21, OUTPUT );  
  radio_setup();
}

void loop() {    
  SYS_TaskHandler(); // this does all the tasks set for scheduling
  readSerial();
  digitalWrite( 21, 0 );
  delay(1);
}
