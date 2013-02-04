void setSerialProtocol()
{
  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("WE", we);   
  sCmd.addCommand("RE", re); 
  sCmd.addCommand("TC", ta);      
  sCmd.addCommand("SD", setDefaultParameters);   
  sCmd.addCommand("SP", setPitchPID);
  sCmd.addCommand("SR", setRollPID);
  sCmd.addCommand("SA", setAccelWeight);
  sCmd.addCommand("SF", setMotorParams);
  sCmd.addCommand("SE", setMotorPWM);
  sCmd.addCommand("SM", setMotorDirNo);
  sCmd.addCommand("GC", gyroRecalibrate);
  sCmd.addCommand("HE", helpMe);
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
}

void we()
{
  EEPROM_writeAnything(0, config); 
}

void re()
{
  EEPROM_readAnything(0, config);  
}

void ta()
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
  config.maxPWMmotorPitch = atoi(sCmd.next());
  config.maxPWMmotorRoll = atoi(sCmd.next());
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
  Serial.println("This gives you a list of all commands with usage:");
  Serial.println("Explanation in brackets(), use Integers only !");
  Serial.println("");
  Serial.println("WE    (Writes active config to eeprom)");   
  Serial.println("RE    (Restores values from eeprom to active config)");      
  Serial.println("TC    (transmits all config values in eeprom save order)");      
  Serial.println("SD    (Set Defaults)");
  Serial.println("SP gyroPitchKp gyroPitchKi gyroPitchKd    (Set PID for Pitch)");
  Serial.println("SR gyroRollKp gyroRollKi gyroRollKd    (Set PID for Roll)");
  Serial.println("SA accelWeight    (Set Weight in accelWeight/1000)");
  Serial.println("SF nPolesMotorPitch nPolesMotorRoll");
  Serial.println("SE maxPWMmotorPitch maxPWMmotorRoll     (Used for Power limitiation on each motor 5=high, 1=low)");
  Serial.println("SM dirMotorPitch dirMotorRoll motorNumberPitch motorNumberRoll");
  Serial.println("GC    (Recalibrates the Gyro Offsets)");
  Serial.println("HE    (This output)"); 
}


// This gets set as the default handler, and gets called when no other command matches.
void unrecognized(const char *command) {
  Serial.println("What? type in HE for Help ...");
}

