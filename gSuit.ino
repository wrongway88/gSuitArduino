#include "Arduino.h"
#include "stdint.h"
#include "SCMD.h"
#include "SCMD_config.h"
#include "Wire.h"
#include "Adafruit_BMP280.h"

// Pins
#define TOGGLE_INFL 5
#define TOGGLE_DEFL 4

#define SELECT 10
#define RESET 11

// Motor driver stuff
#define PUMP_SPEED 200 // minimum verified is 50, 25 didn't run the pumps at all
#define PUMP_REG_LIMIT 350.0f // minimum pressure diff for full speed

// Serial com. protocol
#define DELIMITER ';'

#define CHALLENGE "PUMPITUP"
#define RESPONSE "HITTHEJAM"

#define CMD_INFL "INFL"
#define CMD_DEFL "DEFL"
#define CMD_STOP "STOP"
#define CMD_REBOOT "REBOOT"

#define CMD_SET_PRESSURE "SET_PRESSURE"

#define UPDATE_DELAY 1000/10

#define RECALIBRATION_TIMEOUT 60000 // 60 seconds

SCMD _motorDriver;
Adafruit_BMP280 _bmp;

String _msgBuffer;

bool _doInflate;
bool _doDeflate;

bool _doInflateTest;
bool _doDeflateTest;

// to prevent redundant messages to motor drivers...
bool _wasInflating;
bool _wasDeflating;

bool _sensorIsCalibrated;
bool _sensorIsAvailable;
float _referencePressure;

float _targetPressure;
bool _targetSet;

float _inactiveTime;

void PrintLine(String line)
{
  line = line + DELIMITER;
  Serial.println(line);
}

void setupMotorDriver()
{
  _motorDriver.settings.commInterface = SPI_MODE;
  _motorDriver.settings.I2CAddress = 0x5A;
  _motorDriver.settings.chipSelectPin = SELECT;

  delay(2500);

  uint8_t tempReturnValue = _motorDriver.begin();
  while ( tempReturnValue != 0xA9 )
  {
    PrintLine( "ID mismatch, read as 0x" + String(tempReturnValue, HEX));
    delay(2500);

    tempReturnValue = _motorDriver.begin();
  }
  
  PrintLine( "ID matches 0xA9" );

  PrintLine("Waiting for enumeration...");
  while ( _motorDriver.ready() == false );
  PrintLine("Done.");

  uint8_t tempAddr = _motorDriver.readRegister(SCMD_SLV_TOP_ADDR);
  if ( tempAddr >= START_SLAVE_ADDR )
  {
    PrintLine("Detected " + String(tempAddr - START_SLAVE_ADDR + 1) + " peripherals.");
  }
  else
  {
    PrintLine("No peripherals detected");
  }

  _motorDriver.bridgingMode( 0, 0 );
  _motorDriver.bridgingMode( 1, 0 );
  _motorDriver.bridgingMode( 2, 0 );

  _motorDriver.writeRegister(SCMD_MST_E_IN_FN, 0x1 & 0x2 & 0x4); // reboot and re-init user and expansion port

  _motorDriver.enable();
}

void setupPressureSensor()
{
  unsigned status = _bmp.begin(0x76);

  while(!status)
  {
    PrintLine(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    PrintLine("SensorID was: " + String(_bmp.sensorID()));
    PrintLine("   ID of 0xFF probably means a bad address, a BMP 180 or BMP 085");
    PrintLine("   ID of 0x56-0x58 represents a BMP 280,");
    PrintLine("   ID of 0x60 represents a BME 280.");
    PrintLine("   ID of 0x61 represents a BME 680.");

    delay(1000);

    status = _bmp.begin(0x76);
  }

  PrintLine("bmp begin status: " + String(status));

  _sensorIsAvailable = (status == 1);

  PrintLine("BMP280 initialized");

  _bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_250);

  PrintLine("BMP280 sampling parameters set");
}

void setup()
{
  Serial.setTimeout(0);
  Serial.begin(9600);

  _msgBuffer = "";
  _doInflate = false;
  _doDeflate = false;

  _wasInflating = false;
  _wasDeflating = false;

  _sensorIsCalibrated = false;
  _sensorIsAvailable = false;
  _referencePressure = 0.0f;

  _targetPressure = 0.0f;
  _targetSet = false;

  _inactiveTime = 0.0f;

  // Setup test switch
  pinMode(TOGGLE_INFL, INPUT_PULLUP);
  pinMode(TOGGLE_DEFL, INPUT_PULLUP);

  // Setup motor driver reset pin
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, LOW);


  // Setup motor driver
  setupMotorDriver();

  delay(200);
 
  /**/
  setupPressureSensor();
  /**/
}

void setInflationSpeed(int speed)
{
  PrintLine("setInflationSpeed");
  // _motorDriver.setDrive(2, 0, speed);
  /*
  _motorDriver.setDrive(3, 0, speed);
  _motorDriver.setDrive(4, 0, speed);
  _motorDriver.setDrive(5, 0, speed);
  /**/

  _motorDriver.setDrive(1, 0, speed);
  _motorDriver.setDrive(3, 0, speed);
  _motorDriver.setDrive(5, 0, speed);
} 

void setDeflationSpeed(int speed)
{
  PrintLine("setDeflationSpeed");
  /*
  _motorDriver.setDrive(0, 0, speed);
  _motorDriver.setDrive(1, 0, speed);
  _motorDriver.setDrive(2, 0, speed);
  /**/

  _motorDriver.setDrive(0, 0, speed);
  _motorDriver.setDrive(2, 0, speed);
  _motorDriver.setDrive(4, 0, speed);
}

void handleTestSwitch()
{
  if(digitalRead(TOGGLE_INFL) == false)
  {
    _doInflateTest = true;
  }
  else
  {
    _doInflateTest = false;
  }

  if(digitalRead(TOGGLE_DEFL) == false)
  {
    _doDeflateTest = true;
  }
  else
  {
    _doDeflateTest = false;
  }
}

void setDoInflate(bool inflate)
{
  _doInflate = inflate;
  
  if(_doInflate)
  {
    _doDeflate = false;
  }
}

void setDoDeflate(bool deflate)
{
  _doDeflate = deflate;
  
  if(_doDeflate)
  {
    _doInflate = false;
  }
}

void stopPumps()
{
  _doInflate = false;
  _doDeflate = false;
}

void readSerialInput()
{
  if (Serial.available() > 0)
  {
    String msg = Serial.readString();

    _msgBuffer += msg;

    int msgEnd = _msgBuffer.indexOf(DELIMITER);

    if(msgEnd > -1)
    {
      String m = _msgBuffer.substring(0, msgEnd);
      _msgBuffer = _msgBuffer.substring(msgEnd+1);
      msgEnd = _msgBuffer.indexOf(DELIMITER);

      while(msgEnd > -1)
      {
        m = _msgBuffer.substring(0, msgEnd);
        _msgBuffer = _msgBuffer.substring(msgEnd+1);
        msgEnd = _msgBuffer.indexOf(DELIMITER);
      }

      if(m == CHALLENGE)
      {
        PrintLine(RESPONSE);
      }
      else if(m == CMD_INFL)
      {
        PrintLine("inflate command received");
        setDoInflate(true);
      }
      else if(m == CMD_DEFL)
      {
        PrintLine("deflate command received");
        setDoDeflate(true);
      }
      else if(m == CMD_STOP)
      {
        PrintLine("stop command received");

        _targetPressure = 0.0f;
        _targetSet = false;

        stopPumps();
        setInflationSpeed(0.0f);
        setDeflationSpeed(0.0f);
      }
      else if(m == CMD_REBOOT)
      {
        PrintLine("reboot command received");
        digitalWrite(RESET, HIGH);
        delay(1000);
        digitalWrite(RESET, LOW);
        setupMotorDriver();
      }
      else if(m.indexOf(CMD_SET_PRESSURE) == 0)
      {
        String sVal = m.substring(12);
        _targetPressure = sVal.toFloat();
        _targetSet = true;
        PrintLine("set pressure command received: " + String(_targetPressure));
      }
    }
  }
}

void updatePumps()
{
  bool inflate = _doInflate || _doInflateTest;
  bool deflate = _doDeflate || _doDeflateTest;

  // dont run both at the same time, that's just silly...
  if(inflate & deflate)
  {
    inflate = false;
    deflate = false;
  }

  if(inflate != _wasInflating)
  {
    if(inflate)
    {
      setInflationSpeed(PUMP_SPEED);
    }
    else
    {
      setInflationSpeed(0);
    }
    _wasInflating = inflate;
  }
  
  if(deflate != _wasDeflating)
  {
    if(deflate)
    {
      setDeflationSpeed(PUMP_SPEED);
    }
    else
    {
      setDeflationSpeed(0);
    }
    _wasDeflating = deflate;
  }
}

void calibrate()
{
  /*
  setDeflationSpeed(PUMP_SPEED);

  delay(1000);

  setDeflationSpeed(0);

  delay(10000);
  /**/

  if(_sensorIsAvailable == true)
  {
    _referencePressure = _bmp.readPressure();

    PrintLine("Calibrated Reference Pressure = " + String(_referencePressure));
  }
  else
  {
    PrintLine("Pressure Sensor is not available");
  }
  
  _sensorIsCalibrated = true;
}

void resetPressureSensor()
{
  Serial.println("Pressure sensor lost");

  _targetPressure = 0.0f;
  _targetSet = false;
  stopPumps();
  setInflationSpeed(0.0f);
  setDeflationSpeed(0.0f);

  _bmp.reset();
  setupPressureSensor();
  _sensorIsCalibrated = false;
}

void handleTargetPressure()
{
  float pressure = _bmp.readPressure() - _referencePressure;

  if(_targetSet)
  {
    float diff = _targetPressure - pressure;
    float absDiff = abs(diff);

    // PrintLine(String(_targetPressure) + " - " + String(pressure) + " = " + String(diff));

    if(absDiff > 20.0f) // get positive, diff > 50
    {
      float pf = 1.0f;

      if(absDiff < PUMP_REG_LIMIT)
      {
        pf = constrain(absDiff / PUMP_REG_LIMIT, 0.2f, 1.0f);
      }

      if(diff > 0.0f)
      {
        setDeflationSpeed(0.0f);
        setInflationSpeed(PUMP_SPEED * pf);
      }
      else
      {
        setInflationSpeed(0.0f);
        setDeflationSpeed(PUMP_SPEED * pf);
      }
    }
    else
    {
      setInflationSpeed(0.0f);
      setDeflationSpeed(0.0f);
      if(_targetPressure <= 0.0f)
      {
        _targetSet = false;
      }
    }
  }
}

void loop()
{
  if(_sensorIsCalibrated == false)
  {
    calibrate();
  }

  readSerialInput();

  handleTestSwitch();

  updatePumps();

  // delay to give serial com. some space
  delay(UPDATE_DELAY);

  // check if motor driver is still up and running, or reset if not (e.g. power supply disconnected or something...)
  SCMDDiagnostics diagObject;
  _motorDriver.getDiagnostics(diagObject);
  if(diagObject.MST_E_ERR != 0x0)
  {
    Serial.println("Motor driver lost");
    setupMotorDriver();
  }

  // check sensor is up and running and reset/restart until it is (e.g. unplugged...)
  uint8_t status = _bmp.getStatus();

  if(_sensorIsAvailable == false
    || status == 0xF3)
  {
    resetPressureSensor();
  }
  else
  {
    // normal operations
    handleTargetPressure();
  }

  // recalibrate periodically to handle changing environment conditions
  if(_targetSet == false)
  {
    _inactiveTime += UPDATE_DELAY * 2; // x2 because we delay after reading serial input

    if(_inactiveTime > RECALIBRATION_TIMEOUT)
    {
      _referencePressure = _bmp.readPressure();

      PrintLine("Calibration updated to " + String(_referencePressure));

      _inactiveTime = 0.0f;
    }
  }
  else
  {
    _inactiveTime = 0.0f;
  }

  delay(UPDATE_DELAY);
}
