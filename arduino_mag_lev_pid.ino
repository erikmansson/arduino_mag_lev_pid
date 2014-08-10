/*
Arduino Magnetic Levitator - PID controller
https://github.com/erikmansson/arduino_mag_lev_pid/

LED blink codes:
  1  Setup finished, controller ready
  2  Integral term was maxed out (shut off)
  3  Calibration is way off (shut off)
*/

const int pinLed = 13;
const int pinDriver = 10;
const int pinInput = 23;

//PWM setup (Teensy)
const int pwmBits = 13;
const int pwmFreq = 5859; //[Hz]

//ADC setup
const int adcBits = 12;

const long thrTimeout = 1000 * 200; //[us]
const int loopFreq = 1200; //[Hz]

const long dtMicros = roundToInt(1000000/double(loopFreq)); //[us] used for main loop

//exponential smoothing decay
//0.4 to 1.0 is okay, where 1.0 results in no smoothing
const double inputSmoothing = 0.60;
const double derivSmoothing = 0.60;

const double kp = 8.0;
const double ki = 25.0;
const double kd = 0.13;

const double set = 0.018;
const double thr = 0.008;

long lastLoop = -dtMicros;
long lastOverThr = -thrTimeout;
bool active = false;
double i = 0;
double output = 0;
double cal = 0;
double smoothInput = 0;
double smoothDeriv = 0;

void setup(){
  pinMode(pinLed, OUTPUT);

  //set up PWM (Teensy)
  pinMode(pinDriver, OUTPUT);
  analogWriteFrequency(pinDriver, pwmFreq);
  analogWriteResolution(pwmBits);
  writePWM(0);
  
  //set up ADC
  pinMode(pinInput, INPUT);
  analogReadResolution(adcBits);
  analogRead(pinInput);
  
  //Serial.begin(115200);

  //calibration
  const int nCalReads = 10; //number of readings
  const int calDelay = 5; //[ms] time between readings
  for(int i=0;i<nCalReads;i++){
    delay(calDelay);
    cal += readADC();
  }
  cal = cal / nCalReads; //take the average

  //check calibration
  if(0.4 > cal || cal > 0.6) shutoff(3);

  blink(1);
}

void loop(){
  long now = micros();
  if(now - lastLoop >= dtMicros){
    lastLoop = now;

    double input = signedSquare(readADC() - cal);
    double error = set - input;

    double lastSmoothInput = smoothInput; //for the derivative

    //exponential moving average of input
    smoothInput = input*inputSmoothing +
      (1 - inputSmoothing)*smoothInput;

    //ema of derivative
    smoothDeriv = derivSmoothing*(smoothInput - lastSmoothInput) +
      (1 - derivSmoothing)*smoothDeriv;

    //decide if the controller should be active
    if(input >= thr){
      lastOverThr = now;
      active = true;
    }
    else if(now - lastOverThr < thrTimeout){
      active = true;
    }
    else {
      active = false;
    }

    if(active){
      i += ki * error / loopFreq;
      double p = kp * error;
      double d = -kd * smoothDeriv * loopFreq;

      if(i>1.0) shutoff(2); //we don't want to fry our electronics

      i = constrainPct(i);
      output = constrainPct(signedSquare(p + i + d));
    }
    else{
      i = 0;
      output = 0;
    }

    writePWM(output);
    digitalWrite(pinLed, active);
    
    /* debug
    Serial.print(i);
    Serial.print("\t");
    Serial.println(output);
    */
  }
  else{
    // stuff to do when MCU isn't busy
  }
}

double signedSquare(double x){
  return x * abs(x);
}

int roundToInt(double x){
  return int(x + 0.5);
}

double constrainPct(double val){
  if(val > 1.0){
    return 1.0;
  }
  else if(val < 0.0){
    return 0.0;
  }
  else{
    return val;
  }
}

double readADC(){
  static double adcRange = pow(2, adcBits) - 1;

  return analogRead(pinInput) / adcRange;
}

//takes a double from 0 to 1, like 0.55
void writePWM(double dc){
  static double pwmRange = pow(2, pwmBits) - 1;;

  int pwmOutput = int(dc * pwmRange + 0.5);
  if(pwmOutput > pwmRange){
    pwmOutput = pwmRange;
  }
  else if(pwmOutput < 0){
    pwmOutput = 0;
  }
  analogWrite(pinDriver, pwmOutput);
}

void blink(int n){
  for(int i=0;i<n;i++){
    delay(250);
    digitalWrite(pinLed, 1);
    delay(50);
    digitalWrite(pinLed, 0);
  }
}

void shutoff(int code){
  analogWrite(pinDriver, 0);
  while(true){
    blink(code);
    delay(800);
  }
}
