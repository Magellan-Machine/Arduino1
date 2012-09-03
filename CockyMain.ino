/*
 * Magellan Machine 
 * Cocky Coconut Main Arduino Code
 *
 * Arduino's purpose is to read sensors and control actuators.
 * It communicates with a Raspberry Pi over a serial port.
 */

#define MAX_SENSORS 64

typedef enum { GOT_COMMAND, GOT_DEVICE, READING_VALUE, GOT_NEWLINE, PARSE_ERROR } ParseState;

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

/*
 *  Global variables
 */
// constants won't change. Used here to 
// set pin numbers:
const int ledPin =  13;      // the number of the LED pin

sCommandParseState commandParseState;
String globalError;

Sensor sensors[ MAX_SENSORS ];
Actuator actuators[ MAX_SENSORS ];

sActuator LEDActuator = { 'L', setIO };


 void setup()
 {
   int i;
   
   for( i = 0; i < MAX_SENSORS; i++ )
   {
     sensors[ i ] = NULL;
     actuators[i] = NULL;  
   }

  pinMode(ledPin, OUTPUT);      
 
  actuators[ 'L' - 65 ] = &LEDActuator;
  
  Serial.begin( 115200 );

  // thing below is from an Arduino example
  while (!Serial)
    ; // wait for serial port to connect. Needed for Leonardo only
 
   Serial.println( "Hej!" );
   
  // ready to receive a new command
  commandParseState.state = GOT_NEWLINE;
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

void loop()
{
  int i;
   
  // loop, reading sensors
 
   for( i = 0; i < MAX_SENSORS; i++ )
   {
     Sensor sensor;

     sensor = sensors[i];
     if( sensor != NULL )
       sensor->poll( sensor->name );
   }
  delay( 1000 );
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
   Serial.println( "serialEvent!" );

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
