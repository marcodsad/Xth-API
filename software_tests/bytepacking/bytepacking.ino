
/*
// quintuples of: lowbyte 0 , lowbyte 1, lowbyte 2, lowbyte 3, {highbyte 3, highbyte 2, highbyte 1, highbyte 0}
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
      if ( mmg_block_highindex > XTH_MMGPACKEDSIZE ){
	mmg_block_lowindex = 0;
	mmg_block_highindex = 4;
      }
      mmgs_packed[ mmg_block_highindex ] = 0;
//       mmg_block_subindex = 0;
      mmg_block_subshift = 0;
    }
}*/

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

void setup(){
  Serial.begin( 57600 );
}

uint8_t index = 0;
uint8_t lo_index = 4;
uint8_t lo_shift = 0;

uint8_t packed[80];

bits2byte currentlowbyte;

// [ highbyte 1, hb 2, hb 3, hb4 , [lo1, lo2, lo3, l4]
void pack( uint16_t value ){
  inttobytes unionvalue;
  unionvalue.myint = (value << 6); // bitshift 6
  packed[index] = unionvalue.parts.hi; // high byte;
  currentlowbyte.lobyte |= ( (unionvalue.parts.lo & 0xC0) >> lo_shift);
  packed[lo_index] = currentlowbyte.lobyte;

  
//  Serial.print( value );
//  Serial.print( " " );
//  Serial.print( index );
//  Serial.print( " " );
//  Serial.print( lo_index );
//  Serial.print( " " );
//  Serial.print( lo_shift );
//  Serial.print( " " );
//  Serial.print( packed[index] );
//  Serial.print( " " );
//  Serial.print( unionvalue.parts.hi );
//  Serial.print( " " );
//  Serial.print( unionvalue.parts.lo );
//  Serial.print( " " );
//  Serial.print( currentlowbyte.lobyte );
//  Serial.print( " " );
//  Serial.print( currentlowbyte.bits2.lo1 );
//  Serial.print( " " );  
//  Serial.print( currentlowbyte.bits2.lo2 );
//  Serial.print( " " );  
//  Serial.print( currentlowbyte.bits2.lo3 );
//  Serial.print( " " );  
//  Serial.println( currentlowbyte.bits2.lo4 );

  index++;
  lo_shift += 2;
  if ( lo_shift > 6 ){
      index++; // jump over the low byte
      lo_index += 5;
      lo_index = lo_index%80;
      lo_shift = 0;
      currentlowbyte.lobyte = 0; // this means that it will be reset
  }
  index = index % 80;
}



void loop(){
  for ( uint16_t i=0; i<64; i++ ){
    pack( 63-i );
  }
  
  for (uint8_t i=0; i<80; i++){
    Serial.println(packed[i]);
  }

  Serial.println("FINIS");
  
  delay(1000);
  
}
