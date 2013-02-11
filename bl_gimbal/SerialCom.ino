void setSerialProtocol()
{
  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("WE", writeEEPROM);   
  sCmd.addCommand("RE", readEEPROM); 
  sCmd.addCommand("TC", transmitActiveConfig);      
  sCmd.addCommand("SD", setDefaultParameters);   
  sCmd.addCommand("SP", setPitchPID);
  sCmd.addCommand("SR", setRollPID);
  sCmd.addCommand("SA", setAccelWeight);
  sCmd.addCommand("SF", setMotorParams);
  sCmd.addCommand("SE", setMotorPWM);
  sCmd.addCommand("SM", setMotorDirNo);
  sCmd.addCommand("GC", gyroRecalibrate);
  sCmd.addCommand("TRC", transmitRCConfig);
  sCmd.addCommand("SRC", setRCConfig);
  sCmd.addCommand("HE", helpMe);
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
}


void transmitRCConfig()
{
  Serial.println(config.minRCPitch);
  Serial.println(config.maxRCPitch);
  Serial.println(config.minRCRoll);
  Serial.println(config.maxRCRoll);
}

void setRCConfig()
{
  config.minRCPitch = atoi(sCmd.next());
  config.maxRCPitch = atoi(sCmd.next());
  config.minRCRoll = atoi(sCmd.next());
  config.maxRCRoll = atoi(sCmd.next());
}



void writeEEPROM()
{
  EEPROM_writeAnything(0, config); 
}

void readEEPROM()
{
  EEPROM_readAnything(0, config);  
}

void transmitActiveConfig()
{
  Serial.println(config.vers);
  Serial.println(config.gyroPitchKp);
  Serial.println(config.gyroPitchKi);
  Serial.println(config.gyroPitchKd);
  Serial.println(config.gyroRollKp);
  Serial.println(config.gyroRollKi);
  Serial.println(config.gyroRollKd);
  Serial.println(config.accelWeight);
  Serial.println(config.nPolesMotorPitch);
  Serial.println(config.nPolesMotorRoll);
  Serial.println(config.dirMotorPitch);
  Serial.println(config.dirMotorRoll);
  Serial.println(config.motorNumberPitch);
  Serial.println(config.motorNumberRoll);
  Serial.println(config.maxPWMmotorPitch);
  Serial.println(config.maxPWMmotorRoll);
}


void setPitchPID()
{
  config.gyroPitchKp = atoi(sCmd.next());
  config.gyroPitchKi = atoi(sCmd.next());
  config.gyroPitchKd = atoi(sCmd.next());
}

void setRollPID()
{
  config.gyroRollKp = atoi(sCmd.next());
  config.gyroRollKi = atoi(sCmd.next());
  config.gyroRollKd = atoi(sCmd.next());
}

void setAccelWeight()
{
  config.accelWeight = atoi(sCmd.next());
}

void setMotorPWM()
{
  cli();
  config.maxPWMmotorPitch = atoi(sCmd.next());
  config.maxPWMmotorRoll = atoi(sCmd.next());
  calcSinusArray(config.maxPWMmotorPitch,pwmSinMotorPitch);
  calcSinusArray(config.maxPWMmotorRoll,pwmSinMotorRoll);
  sei();
}

void setMotorParams()
{
  config.nPolesMotorPitch = atoi(sCmd.next());
  config.nPolesMotorRoll = atoi(sCmd.next());
  // Initialize Motor Movement
  maxDegPerSecondPitch = MOTORUPDATE_FREQ * 1000.0 / N_SIN / (config.nPolesMotorPitch/2) * 360.0;
  maxDegPerSecondRoll = MOTORUPDATE_FREQ * 1000.0 / N_SIN / (config.nPolesMotorRoll/2) * 360.0;
}

void gyroRecalibrate()
{
  gyroOffsetCalibration();
}

void setMotorDirNo()
{
  config.dirMotorPitch = atoi(sCmd.next());
  config.dirMotorRoll = atoi(sCmd.next());
  config.motorNumberPitch = atoi(sCmd.next());
  config.motorNumberRoll = atoi(sCmd.next());
}

void helpMe()
{
  Serial.println(F("This gives you a list of all commands with usage:"));
  Serial.println(F("Explanation in brackets(), use Integers only !"));
  Serial.println(F(""));
  Serial.println(F("WE    (Writes active config to eeprom)"));
  Serial.println(F("RE    (Restores values from eeprom to active config)"));  
  Serial.println(F("TC    (transmits all config values in eeprom save order)"));     
  Serial.println(F("SD    (Set Defaults)"));
  Serial.println(F("SP gyroPitchKp gyroPitchKi gyroPitchKd    (Set PID for Pitch)"));
  Serial.println(F("SR gyroRollKp gyroRollKi gyroRollKd    (Set PID for Roll)"));
  Serial.println(F("SA accelWeight    (Set Weight in accelWeight/1000)"));
  Serial.println(F("SF nPolesMotorPitch nPolesMotorRoll"));
  Serial.println(F("SE maxPWMmotorPitch maxPWMmotorRoll     (Used for Power limitiation on each motor 255=high, 1=low)"));
  Serial.println(F("SM dirMotorPitch dirMotorRoll motorNumberPitch motorNumberRoll"));
  Serial.println(F("GC    (Recalibrates the Gyro Offsets)"));
  Serial.println(F("TRC   (transmitts RC Config)"));
  Serial.println(F("SRC minRCPitch maxRCPitch minRCRoll maxRCRoll (angles -90..90)"));
  Serial.println(F("HE    (This output)"));
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println(F("What? type in HE for Help ..."));
}

