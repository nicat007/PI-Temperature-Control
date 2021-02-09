
double Sensor = A0;
double output = A1;

//PID constants for the FAN
double kpF = 10;
double kiF = 0;
double kdF = 0;

//PID constants for the HEATER
double kpH = 2;
double kiH = 5;
double kdH = 1;

//PID constants 
double Kp;
double Ki;
double Kd;

//Variables to be used
unsigned long currTime, prevTime;
double elapsedTime;
double err;
double prevErr=0.0;
double desiredTemp;
double errCum, errDiff;
double ControllerOutput;
double value;
double a1, b1, c1, d1, r2, r1, vo, tempC, tempF, tempK;

void setup() {

    Serial.begin(9600);
  a1 = 3.354016E-03 ;
  b1 = 2.569850E-04 ;
  c1 = 2.620131E-06 ;
  d1 = 6.383091E-08 ;

  desiredTemp = 35;
  
  r1 = 9720.0;
  
  pinMode(Sensor, INPUT);
  pinMode(output, OUTPUT);

}

void loop() {

  //desiredTemp = Serial.read();

   // read the temp
  Sensor = analogRead(Sensor);
  Sensor = Sensor / (1023.0 / 5.0);

  // voltage divider calculation
  // vo = 5 * r2 /(r1+r2)
  // solve for r2
  // get the exact value for voltage divider r1

  r2 = ( Sensor * r1) / (5.0 - Sensor);

  //equation from data sheet
  tempK = 1.0 / (a1 + (b1 * (log(r2 / 10000.0))) + (c1 * pow(log(r2 / 10000.0), 2.0)) + (d1 * pow(log(r2 / 10000.0), 3.0)));
  tempC  = (tempK - 273.15);
  tempF  = (tempC * 1.8) + 32.0;

  currTime = millis();
  elapsedTime = (double)(currTime - prevTime);
 
  err = desiredTemp - tempC; 
  errCum += err * elapsedTime;
  errDiff = (err - prevErr)/elapsedTime;

  if (err<0) {
    value = fanController (kpF, kiF, kdF, err, errCum, errDiff );
    Serial.println("fan");
  }
  
  else if (err>0) {
    value = heaterController (kpH, kiH, kdH, err, errCum, errDiff);
    Serial.println("heater");
  };
    
  analogWrite(output,  value);
  prevErr = err;
  prevTime = currTime;

  Serial.println("Temp - degK " + String(tempK));
  Serial.println("Temp - degC " + String(tempC));
  Serial.println("Temp - degF " + String(tempF));
  Serial.println(err);
  Serial.println(value);
  Serial.println(" ");
  
  }

double fanController(double Kp, double Ki, double Kd, double error, double errorCum, double errorDiff) {
  ControllerOutput = Kp*error + Ki*errorCum + Kd*errorDiff;

  return ControllerOutput;
  }

double heaterController(double Kp, double Ki, double Kd, double error, double errorCum, double errorDiff) {

  ControllerOutput = Kp*error + Ki*errorCum + Kd*errorDiff;

  return ControllerOutput;
  }
