#include <Servo.h>
#include <SPI.h>
#include <Wire.h>

/*
 * Magellan Machine 
 * Cocky Coconut Main Arduino Code
 *
 * Arduino's purpose is to read sensors and control actuators.
 * It communicates with a Raspberry Pi over a serial port.
 */
#define RUDDER_I2C_ADDRESS 2

#define MAX_SENSORS 64

/*
 * IMU stuff
 */
typedef enum { 
  UM6_STATUS = 0x55, UM6_GYRO_RAW_XY = 0x56, UM6_GYRO_RAW_Z = 0x57,
  UM6_ACCEL_RAW_XY = 0x58, UM6_ACCEL_RAW_Z = 0x59, 
  UM6_MAG_RAW_XY = 0x5A, UM6_MAG_RAW_Z = 0x5B,
  UM6_GYRO_PROC_XY = 0x5C,  UM6_GYRO_PROC_Z = 0x5D,
  UM6_ACCEL_PROC_XY = 0x5E, UM6_ACCEL_PROC_Z = 0x5F,
  UM6_MAG_PROC_XY = 0x60, UM6_MAG_PROC_Z = 0x61,
  UM6_EULER_PHI_THETA = 0x62, UM6_EULER_PSI = 0x63, 
  UM6_TEMPERATURE = 0x76, UM6_GPS_LONGITUDE = 0x77,
  UM6_GPS_LATITUDE = 0x78,
  UM6_GPS_ALTITUDE = 0x79,
  UM6_GPS_POSITION_N = 0x7A,
  UM6_GPS_POSITION_E = 0x7B,
  UM6_GPS_POSITION_H = 0x7C,
  UM6_GPS_COURSE_SPEED = 0x7D,
  UM6_GPS_SAT_SUMMARY = 0x7E,
  UM6_GPS_SAT_0_1 = 0x7F,
  UM6_GPS_SAT_2_3 = 0x80,
  UM6_GPS_SAT_4_5 = 0x81,
  UM6_GPS_SAT_6_7 = 0x82,
  UM6_GPS_SAT_8_9 = 0x83,
  UM6_GPS_SAT_10_11 = 0x84,
  UM6_ZERO_GYROS = 0xA6, UM6_GET_FW_VERSION = 0xAA
} IMUAddress;

typedef enum {
  IMU_PARSED_NOTHING, IMU_PARSED_S, IMU_PARSED_N, IMU_PARSED_P, 
  IMU_PARSED_TYPE, IMU_PARSED_ADDRESS, IMU_PARSED_DATA, IMU_PARSED_CHECKSUM_0
} IMUParseState;

/*
 * type definitions
 */
typedef enum { GOT_COMMAND, GOT_DEVICE, READING_VALUE, GOT_NEWLINE, 
               PARSE_ERROR } ParseState;

typedef struct sCommandParseState
{
  ParseState state;  
  char command;
  int deviceCount;
  char devices[MAX_SENSORS];
  int valueLength;
  char value[256];
  String errorMessage;
} sCommandParseState;

typedef struct sIMUPacket
{
  boolean hasData;
  boolean isBatch;
  int batchLength;
  int commandFailed;
  int dataLength;
  
  byte address;
  
  byte data[ 16 * 4 ];
  
  byte checksum[2];
} sIMUPacket;

// GetValueFunc returns error message or NULL if successful
typedef const char * (*GetValueFunc)( char name, char *resultBuffer );

// SetValueFunc returns error message or NULL if successful
typedef const char * (*SetValueFunc)( char name, const char *value );
typedef void (*PollFunc)( char name );

typedef struct sSensor
{
  char name;
  GetValueFunc get;
  PollFunc poll;
} sSensor, *Sensor;

typedef struct sActuator
{
  char name;
  SetValueFunc set;
} sActuator, *Actuator;

/*
 * Static function 
 */
static void sendGetError( char device, const char *errorMessage );
void imu_init();
void imu_poll();
static void imu_sendPacket( struct sIMUPacket *packet );

/*
 *  Global variables
 */
// constants won't change. Used here to 
// set pin numbers:
const int ledPin =  13, imuSSPin = 10, rudderPin = 3, sailPin = 5, 
  leftEndStopPin = 4, rightEndStopPin = 6;      // the number of the LED pin

Servo rudderServo, sailServo;

sCommandParseState commandParseState;
String globalError;

struct
{
  IMUParseState state;
  int dataIndex, dataLength;
  uint16_t checksum;
  sIMUPacket packet;
} imuParseInfo;

struct 
{
  // multiply by 360 / 32768 to get degrees
  int16_t eulerPsi; // yaw
  int16_t eulerTheta; // pitch
  int16_t eulerPhi; // roll
  
  struct
  {
    float longitude, latitude;
    int16_t cog; // divide by 100 to get value in degrees
    int16_t speed; // divide by 100 to get speed in m/s
  
    int mode;
    int satCount;
    int hdop;
    int vdop;
  } gps;
  
} imuData;

Sensor sensors[ MAX_SENSORS ];
Actuator actuators[ MAX_SENSORS ];

sActuator LEDActuator = { 'L', setIO };
sActuator rudderActuator = { 'R', setRudder };
sActuator sailActuator = { 'S', setServo };

sSensor latitudeSensor = { 'A', getIMU, NULL };
sSensor longitudeSensor = { 'O', getIMU, NULL };
sSensor gpsSatCountSensor = { 'S', getIMU, NULL };

// Mast NIY
// sSensor mastSensor = { 'M', getRotation, NULL };
sSensor pitchSensor = { 'P', getIMU, NULL };
sSensor rollSensor = { 'R', getIMU, NULL };
sSensor yawSensor = { 'Y', getIMU, NULL };
// sSensor imuFirmwareSensor = { 'F', getIMU, NULL };
// sSensor imuStatusSensor = { 'S', getIMU, NULL };

void setup()
{
   int i;

  Serial.begin( 115200 );

  // thing below is from an Arduino example
  while (!Serial)
    ; // wait for serial port to connect. Needed for Leonardo only

  Serial.println( "setup: Entry" );
  
   for( i = 0; i < MAX_SENSORS; i++ )
   {
     sensors[ i ] = NULL;
     actuators[i] = NULL;  
   }

  pinMode(ledPin, OUTPUT);      
  pinMode( rightEndStopPin, INPUT );
  pinMode( leftEndStopPin, INPUT );
  
  rudderServo.attach( rudderPin );
  sailServo.attach( sailPin );
  
  actuators[ 'L' - 65 ] = &LEDActuator;
  actuators[ 'R' - 65 ] = &rudderActuator;
  actuators[ 'S' - 65 ] = &sailActuator;
  
  sensors[ 'A' - 65 ] = &latitudeSensor;
  sensors[ 'O' - 65 ] = &longitudeSensor;
  sensors[ 'R' - 65 ] = &rollSensor;
  sensors[ 'P' - 65 ] = &pitchSensor;
  sensors[ 'S' - 65 ] = &gpsSatCountSensor;
  sensors[ 'Y' - 65 ] = &yawSensor;
  
  Wire.begin(); // for i2c communication with other Arduino
#if 0
  // SPI Communication with IMU
  pinMode( imuSSPin, OUTPUT);      
  
  digitalWrite( imuSSPin, 1 ); // active Low
  
  SPI.begin(); // SPI is for communication with the IMU
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0); // I think it is mode 2, pololu forum says mode 0?
  // Leonardo clock speed is 16 MHz
  // IMU maximum clock rate is 400 kHz
  // ideal divider is 40. Closes higher is 64
  // we will be using 250 kHz
  SPI.setClockDivider( SPI_CLOCK_DIV64 );
#endif
 
  // ready to receive a new command
  commandParseState.state = GOT_NEWLINE;

  imu_init();
    
  imu_zeroGyros();

  Serial.println( "Main Arduino: setup done." );
}

const char *setIO( char name, const char *value )
{
  int intVal;
  
  intVal = atoi( value );
  
  if( name == 'L' )
  {
    digitalWrite( ledPin, intVal );
    return NULL;
  }
  else
    return "No such IO";
}

const char *getIO( char name, char *resultBuffer )
{
  int pinNumber, value;
  
  switch( name )
  {
    case 'L':
      pinNumber = leftEndStopPin;
      break;
      
    case 'R':
      pinNumber = rightEndStopPin;
      break;
  }
  
  sprintf( resultBuffer, "%c", digitalRead( pinNumber ) ? '0' : '1' );
  
  return NULL;
}
#if 0
uint16_t readIMURegisterWord( int address )
{
   byte B2, B3;
  
   // read a register from the IMU
   digitalWrite( imuSSPin, LOW );
   delayMicroseconds(1);
   SPI.transfer( 0 );
   SPI.transfer( address );
   
   B3 = SPI.transfer( 0 );
   B2 = SPI.transfer( 0 );
   SPI.transfer( 0 );
   SPI.transfer( 0 );
   
   digitalWrite( imuSSPin, HIGH );
  
   return B3 | (B2 << 8);
}

uint32_t readIMURegisterLong( byte address )
{
   byte b[4];
  
   // read a register from the IMU
   digitalWrite( imuSSPin, LOW );
   delayMicroseconds(1);
   // delay(1000);
   SPI.transfer( 0 );
   SPI.transfer( address );
   
   b[3] = SPI.transfer( 0 );
   b[2] = SPI.transfer( 0 );
   b[1] = SPI.transfer( 0 );
   b[0] = SPI.transfer( 0 );
   
   digitalWrite( imuSSPin, HIGH );
  
   return b[3] | (b[2] << 8) | (b[1] << 16) | (b[0] << 24);
}
#endif
#if 0
  sSensor latitudeSensor = { 'A', getIMU, NULL };
sSensor latitudeSensor = { 'O', getIMU, NULL };
// Mast NIY
// sSensor mastSensor = { 'M', getRotation, NULL };
sSensor pitchSensor = { 'P', getIMU, NULL };
sSensor rollSensor = { 'R', getIMU, NULL };
sSensor yawSensor = { 'Y', getIMU, NULL };
#endif

const char *getIMU( char name, char *resultBuffer )
{
  switch( name )
  {
    case 'A':
      sprintf( resultBuffer, "%f", imuData.gps.latitude );
      return NULL;
      
    case 'O':
      sprintf( resultBuffer, "%f", imuData.gps.longitude );
      return NULL;
    
    case 'P':
      sprintf( resultBuffer, "%d", (int) (imuData.eulerTheta * 0.0109863) );
      return NULL;
      
    case 'Y':
      sprintf( resultBuffer, "%d", (int) (imuData.eulerPsi * 0.0109863) );
      return NULL;

    case 'R':
      sprintf( resultBuffer, "%d", (int) (imuData.eulerPhi * 0.0109863) );
      return NULL;
      
    case 'S':
      sprintf( resultBuffer, "%d", imuData.gps.satCount );
      return NULL;
      
    default:
      return "getIMU: Invalid IMU command";
  }
}

#if 0
const char *getIMU( char name, char *resultBuffer )
{
  uint32_t registerValue;
  float angle;
  
  switch( name )
  {
    case 'F':
      registerValue = readIMURegisterLong( UM6_GET_FW_VERSION );
      sprintf( resultBuffer, "%lx", registerValue );
      #if 0
      resultBuffer[0] = registerValue & 0xff;
      resultBuffer[1] = (registerValue >> 8) & 0xff;
      resultBuffer[2] = (registerValue >> 16) & 0xff;
      resultBuffer[3] = (registerValue >> 24) & 0xff;
      resultBuffer[4] = '\0';
      #endif
      return NULL;

   case 'S':
      registerValue = readIMURegisterLong( UM6_GET_FW_VERSION );
      sprintf( resultBuffer, "%lx", registerValue );
      return NULL;

    case 'Y':
      registerValue = readIMURegisterWord( UM6_EULER_PSI );
      angle = registerValue * 0.0109863;
      sprintf( resultBuffer, "%d", registerValue );
      return NULL;
   
   default:
      return "Invalid IMU register";   
  }
}
#endif

const char *setServo( char name, const char *value )
{
  int angle = atoi( value );
  const char *error = NULL;
  
  switch( name )
  {
    case 'R':
      rudderServo.write( angle );
      break;
      
    case 'S':
      sailServo.write( angle );
      break;

    default:
      error = "Invalid servo specivied";
      break;
  }
  
  return error;
}

const char *setRudder( char name, const char *value )
{
  int i, err;
  
  i = atoi( value );
  
  Wire.beginTransmission( RUDDER_I2C_ADDRESS );
  Wire.write( name );
  Wire.write( i >> 8 );
  Wire.write( i & 0xff );

  err = Wire.endTransmission();
  
  if( err == 0 )
    return NULL;
  else
    return i2cErrorToString( err );
}

const char *i2cErrorToString( int err )
{
  static const char *messages[] = 
  { "success",
    "data too long to fit in transmit buffer",
    "received NACK on transmit of address",
    "received NACK on transmit of data",
    "other error" };
    
  if( err < 5 && err >= 0 )
    return messages[ err ];
  else
    return "unknown error";
}

void loop()
{
  int i;
  static int counter = 0;
  
  counter++;
  if( counter == 1000 )
  {
    // Serial.print( "-" );
    counter = 0;
  }
#if 1
  // loop, reading sensors
 
  for( i = 0; i < MAX_SENSORS; i++ )
  {
     Sensor sensor;

     sensor = sensors[i];
     if( sensor != NULL && (sensor->poll != NULL) )
       sensor->poll( sensor->name );
  }
#endif  
  imu_poll();
  
  serialEvent();
}

void setGlobalError( String message )
{
  globalError = message;
}

void doGetCommand( int deviceCount, char *devices )
{
  int i;
  Sensor sensor;
  
  for( i = 0; i < deviceCount; i++ )
  {
    if( devices[ i ] < 65 || devices[ i ] > 122 )
      sensor = NULL;
    else
      sensor = sensors[ devices[ i ] - 65 ];
    
    if( sensor != NULL )
    {
      const char *errorMessage;
      char valueBuffer[ 256 ];
      
      errorMessage = sensor->get( sensor->name, valueBuffer );
      if( errorMessage == NULL ) // was successful
        sendGetResponse( sensor->name, valueBuffer );
      else
        sendGetError( sensor->name, errorMessage );
    }
    else
    {
      char errorMessage[256 ];
      
      sprintf( errorMessage, "Sensor '%c' does not exist", devices[ i ] );
      sendGetError( devices[i], errorMessage );
    }
  }
}

void sendGetResponse( char sensorName, const char *value )
{
  Serial.write( 'G' );
  Serial.write( sensorName );
  Serial.print( value );
  Serial.write( '\n' );  
}

void doSetCommand( char device, const char *value )
{
  Actuator actuator;
  
  if( (device < 65) || (device > 122) )
    actuator = NULL;
  else
    actuator = actuators[ device - 65 ];
  
  if( actuator != NULL )
  {
    const char *errorMessage;
    char buffer[256];
    
    errorMessage = actuator->set( device, value );
    
    if( errorMessage == NULL )
    {
      // Send Set OK message
      Serial.write( 'S' );
      Serial.write( device );
      Serial.write( '\n' );
    }   
    else
      sendSetError( device, errorMessage );
  }
  else
    sendSetError( device, "Actuator does not exist" );
}

void sendSetError( char device, const char *errorMessage )
{
  Serial.write( 'S' );
  Serial.print( device );
  Serial.print( errorMessage );
  Serial.write( '\n' );
} 

static void sendGetError( char device, const char *errorMessage )
{
  Serial.write( 'E' );
  Serial.print( device );
  Serial.print( errorMessage );
  Serial.write( '\n' );
} 

void serialEvent()
{
   // Serial.println( "serialEvent!" );

  while( Serial.available() )
  {
    byte ch;
    
    ch = Serial.read();
    
    switch( commandParseState.state )
    {
      case GOT_NEWLINE: // ready for a new command
        if( ch == '\n' )
          continue; // ignore multiple newlines after another
          
        commandParseState.command = ch;
        if( commandParseState.command == 'S' || 
            commandParseState.command == 'G' )
        {
          commandParseState.state = GOT_COMMAND;
          commandParseState.deviceCount = 0;
        }
        else
        {
          commandParseState.state = PARSE_ERROR;
          commandParseState.errorMessage = "Command was not 'G' or 'S'";
        }
        break;
        
      case GOT_COMMAND:
        if( ch == '\n' )
        {
          setGlobalError( "Got command without device" );
          commandParseState.state = GOT_NEWLINE;
          continue;
        }
        
        commandParseState.devices[ commandParseState.deviceCount ] = ch;
        commandParseState.deviceCount++;
        commandParseState.state = GOT_DEVICE;
        break;
        
      case GOT_DEVICE:
        if( ch == '\n' )
        {
          commandParseState.state = GOT_NEWLINE;
          if( commandParseState.command == 'G' )
            // got a complete get command. execute it
            doGetCommand( commandParseState.deviceCount, commandParseState.devices ); // note, will block serial reception until return
          else
            // return an error that it was a set command without values
            sendSetError( commandParseState.devices[0], 
                            "Set command without value" );
          continue;
        }
        
        switch( commandParseState.command )
        {
           // if it is a set command, start reading the value
           case 'S':
             commandParseState.valueLength = 1;
             commandParseState.value[0] = ch;
             commandParseState.state = READING_VALUE;
             break;
             
           // if it is a get command, read another device
           case 'G':
             if( commandParseState.deviceCount == 256 )
               setGlobalError( "More than 256 devices specified in get command" );
               // ignore this device
             else
             {
               commandParseState.devices[ commandParseState.deviceCount ] = ch;
               commandParseState.deviceCount++;
             }
             break;
        }
        break;
        
      case READING_VALUE:
        if( ch == '\n' )
        {
          commandParseState.state = GOT_NEWLINE;
          commandParseState.value[ commandParseState.valueLength ] = '\0'; // zero-terminate string
          doSetCommand( commandParseState.devices[0], commandParseState.value );
        }
        else
        {
          if( commandParseState.valueLength == 256 )
            sendSetError( commandParseState.devices[0], "Value was more than 255 characters long" );
          else
          {
            commandParseState.value[ commandParseState.valueLength ] = ch;
            commandParseState.valueLength++;
          }
        }
        break;
        
      case PARSE_ERROR:
        if( ch == '\n' )
        {
          setGlobalError( commandParseState.errorMessage );
          commandParseState.state = GOT_NEWLINE;
        }
        else
          continue; // ignore characters until newline
    }
  }
}

void imu_init()
{
  Serial1.begin( 115200 );
}

void imu_poll()
{
  static int packetCounter = 0;
  
  while( Serial1.available() )
  {
    byte b;
    
    b = Serial1.read();

    if( imuParseInfo.state <  IMU_PARSED_DATA )
      imuParseInfo.checksum += b;
#if 0
    Serial.print( "b=" );
    Serial.print( b, HEX );
    Serial.print( ", chksum=" );
    Serial.println( imuParseInfo.checksum, HEX );
#endif    
    switch( imuParseInfo.state )
    {
      case IMU_PARSED_NOTHING:
        if( b == 's' )
        {
          imuParseInfo.checksum = b;
          imuParseInfo.state = IMU_PARSED_S;
        }
        break;
        
      case IMU_PARSED_S:
        if( b == 'n' )
          imuParseInfo.state = IMU_PARSED_N;
        else
          imuParseInfo.state = IMU_PARSED_NOTHING;
        break;
        
      case IMU_PARSED_N:
        if( b == 'p' )
          imuParseInfo.state = IMU_PARSED_P;
        else
          imuParseInfo.state = IMU_PARSED_NOTHING;
        break;
      
      case IMU_PARSED_P:
        // imuParseInfo.packet.type = b;
        
        imuParseInfo.packet.hasData = (b & 0x80);
        imuParseInfo.packet.isBatch = (b & 0x40);
        imuParseInfo.packet.batchLength = (b >> 2) & 0x0f;
        imuParseInfo.packet.commandFailed = b & 0x1;

        if( imuParseInfo.packet.hasData )
        {
          if( imuParseInfo.packet.isBatch )
            imuParseInfo.dataLength = 4 * imuParseInfo.packet.batchLength;
          else
            imuParseInfo.dataLength = 4;
        }
        else
          imuParseInfo.dataLength = 0;
          
        imuParseInfo.packet.dataLength = imuParseInfo.dataLength;
        
        // Serial.print( "DL:" );
        // Serial.println( imuParseInfo.dataLength );
        
        imuParseInfo.state = IMU_PARSED_TYPE;
        break;
        
      case IMU_PARSED_TYPE:
        imuParseInfo.packet.address = b;
        if( imuParseInfo.dataLength == 0 )
          imuParseInfo.state = IMU_PARSED_DATA;
        else
          imuParseInfo.state = IMU_PARSED_ADDRESS;
        
        imuParseInfo.dataIndex = 0;
        break;
        
      case IMU_PARSED_ADDRESS:
        // read Data
        imuParseInfo.packet.data[ imuParseInfo.dataIndex ] = b;
        imuParseInfo.dataIndex++;
        imuParseInfo.dataLength--;
        if( imuParseInfo.dataLength == 0 )
          imuParseInfo.state = IMU_PARSED_DATA;
        break;
        
      case IMU_PARSED_DATA:
        imuParseInfo.packet.checksum[0] = b;
        imuParseInfo.state = IMU_PARSED_CHECKSUM_0;
        break;
        
      case IMU_PARSED_CHECKSUM_0:
        imuParseInfo.packet.checksum[1] = b;
        
        // Verify checksum
        if( (imuParseInfo.checksum >> 8) == imuParseInfo.packet.checksum[0] &&
            (imuParseInfo.checksum & 0xff) == b )
        {
          imu_parsePacket( &(imuParseInfo.packet) );
          // Serial.print( "." );
        }
        else
          Serial.println( "Bad checksum" );
          
        packetCounter++;
#if 0
        if( packetCounter == 20 )
        {
          Serial.println( "Looping." );
        
          while( 1 )
            delay( 100 );
        }
#endif
        imuParseInfo.state = IMU_PARSED_NOTHING;
        break;
    } // end of switch( parse state );
    
    // Serial.print( imuParseInfo.state );
  } // end of while( there is serial data )
}

static void imu_sendPacket( struct sIMUPacket *packet )
{
  int checksum = 's' + 'n' + 'p';
  byte packetType;
  
  Serial1.print( "snp" );
  
  packetType = (packet->hasData ? 0x80 : 0) | 
                 (packet->isBatch ? 0x40 : 0) |
                 (packet->batchLength << 2);
  Serial1.write( packetType );
  checksum += packetType;
  
  Serial1.write( packet->address );
  checksum += packet->address;
  
  // write data
  if( packet->hasData )
    // Serial1.write( packet->
    ;
  
  Serial1.write( checksum >> 8 );
  Serial1.write( checksum & 0xff );
}

void imu_zeroGyros()
{
  struct sIMUPacket packet;
  
  packet.hasData = 0;
  packet.isBatch = 0;
  packet.batchLength = 0;
  packet.address = UM6_ZERO_GYROS;
  
  imu_sendPacket( &packet );
}

static int16_t decodeTwosComplementWord( byte low, byte high )
{
  int x;

  x = low + (high << 8);
  if( x >= (1 << 15) )
     x = x - (1 << 16);

  return x;
}

void imu_parsePacket( const struct sIMUPacket *packet )
{
  int address, addressPastEnd, dataIndex;
  
  addressPastEnd = packet->address + packet->dataLength / 4;
  
  for( address = packet->address, dataIndex = 0; address < addressPastEnd; 
       dataIndex += 4, address++)
  {
    // Serial.print( "A:" );
    // Serial.println( address, HEX );
    
    switch( address )
    {
      // case UM6_MAG_RAW_XY:
      // case UM6_MAG_RAW_Z:
      // case MU6_ACCEL_RAW_Z:
      // case UM6_ACCEL_RAW_XY:
      case UM6_EULER_PHI_THETA:
        imuData.eulerTheta = decodeTwosComplementWord( packet->data[dataIndex + 1], 
                                                       packet->data[dataIndex] );
        imuData.eulerPhi = decodeTwosComplementWord( packet->data[dataIndex + 3], 
                                                       packet->data[dataIndex] + 2);
        break;
        
      case UM6_EULER_PSI:
        imuData.eulerPsi = decodeTwosComplementWord( packet->data[dataIndex + 3], 
                                                       packet->data[dataIndex] + 2);
        break;
        
      case UM6_GPS_LONGITUDE:
        // do we need endianness conversion?
        Serial.print( "Longitude: " );
        Serial.print( *( ((uint32_t *) &(packet->data[ dataIndex ]))), HEX);
        Serial.print( " = " );
        Serial.println( *( (float *) &(packet->data[ dataIndex ] )));
        imuData.gps.longitude = *( (float *) &(packet->data[ dataIndex ] ));
        break;
        
      case UM6_GPS_LATITUDE:
        // do we need endianness conversion?
        float f;
        byte *f2;
        f2 = (byte *) &f;
        f2[0] = packet->data[ dataIndex + 3 ];
        f2[1] = packet->data[ dataIndex + 2 ];
        f2[2] =  packet->data[ dataIndex + 1 ];
        f2[3] = packet->data[ dataIndex ];
        imuData.gps.latitude = f; // *( (float *) &(packet->data[ dataIndex ] ));
        break;
        
      case UM6_GPS_COURSE_SPEED:
        imuData.gps.cog = decodeTwosComplementWord( packet->data[dataIndex + 3], 
                                                packet->data[dataIndex] + 2);
        imuData.gps.speed = decodeTwosComplementWord( packet->data[dataIndex + 1], 
                                                       packet->data[dataIndex] );
        break;
 
      case UM6_GPS_SAT_SUMMARY:
        Serial.print( "GPS Sat Summary=" );
        Serial.println( (uintpacket->data
        imuData.gps.mode = packet->data[ dataIndex + 3 ] >> 6;
        imuData.gps.satCount = (packet->data[ dataIndex + 3 ] >> 2) & 0x0f;
        imuData.gps.hdop =  ((int) (packet->data[ dataIndex + 3 ] & 0x3)) << 8 | packet->data[ dataIndex + 2 ];
        imuData.gps.vdop = ((int) packet->data[ dataIndex + 1 ]) << 2 | ( packet->data[ dataIndex ] >> 6 );
        break;
      
      case UM6_GYRO_RAW_XY:
      case UM6_GYRO_RAW_Z:
      case UM6_GYRO_PROC_XY:
      case UM6_GYRO_PROC_Z:
      case UM6_ACCEL_RAW_XY:
      case UM6_ACCEL_RAW_Z:
      case UM6_MAG_RAW_XY:
      case UM6_MAG_RAW_Z:
      case UM6_MAG_PROC_XY:
      case UM6_MAG_PROC_Z:
      case UM6_ACCEL_PROC_XY:
      case UM6_ACCEL_PROC_Z:
      case UM6_TEMPERATURE:
      case UM6_GPS_ALTITUDE:
      case UM6_GPS_POSITION_N:
      case UM6_GPS_POSITION_E:
      case UM6_GPS_POSITION_H:
      case UM6_GPS_SAT_0_1:
      case UM6_GPS_SAT_2_3:
      case UM6_GPS_SAT_4_5:
      case UM6_GPS_SAT_6_7:
      case UM6_GPS_SAT_8_9:
      case UM6_GPS_SAT_10_11:
        // ignore for now
        break;
        
      default:
        Serial.print( "Unimplemented IMU address: " );
        Serial.println( address, HEX );
        break;
    } // end of switch
  } // end of loop over data length
}
