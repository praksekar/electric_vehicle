#ifdef NO_ARDUINO
#include <iostream>
using namespace std;

#include "ard.h"
#define NPOINTS 2000

#else
#define uint16_t unsigned int
#define unit8_t unsigned char
#define NPOINTS 410

#endif

#define PWM_SCALE 400

const int pinPwm = 10;
const int pinDir = 13;
const int encoder_a = 2; // Green - pin 2 - Digital
const int encoder_b = 3; // White - pin 3 - Digital
long volatile encoder = 0;
//static  long lastEncoder = encoder;

#define FORWARD LOW
#define REVERSE HIGH

long start_time = 0;
long curPos=0;
long lastPos=0;
long curTime = 0;
long curSpd = 0;
int spdPwm=0; // value written to the pwm output to control motor speed

#ifndef NO_ARDUINO
void writeToMotor(int pwm) {
   if (pwm >= 0) {
    analogWrite(pinPwm, pwm);
    digitalWrite(pinDir, LOW);
  } 
  else {
    analogWrite(pinPwm, -pwm);
    digitalWrite(pinDir, HIGH);
  }
}
#endif

void pr(const char*s) {
#ifdef NO_ARDUINO
   cout << s;
#else
   Serial.print(s);
#endif
}

void pr(long l) {
#ifdef NO_ARDUINO
   cout << l;
#else
   Serial.print(l);
#endif
}

void pr(const char* s,  long l) {
   pr(s);
   pr(l);
}

void prln(const char* s,  long l) {
   pr(s);
   pr(l);
   pr("\n");
}

void prln(const char* s) {
   pr(s);
   pr("\n");
}

void prln(long l) {
   pr(l);
   pr("\n");
}

void encoderPinChangeA() {
   long e=encoder;
   if (digitalRead(encoder_a) == digitalRead(encoder_b))
      encoder = e-1;
   else encoder = e+1;
}

void encoderPinChangeB() {
   long e=encoder;
   if (digitalRead(encoder_a) != digitalRead(encoder_b))
      encoder = e-1;
   else encoder = e+1;
}

long getEncoder() {
  long e = encoder;
  /*
  long diff = encoder - lastEncoder;
  if (diff > 50000)
    prln(" Too fast: ", diff);
  diff = -diff;
  if (diff > 50000)
    prln(" Too fast: ", diff);
  */
  return e;
}

#define D1WIDTH 6
#define mask(x) ((1<<x)-1)
#define midpoint(x) (1<<(x-1))
#define D1MAX mask(D1WIDTH)
#define D1OFFSET midpoint(D1WIDTH)
#define D2WIDTH (16-D1WIDTH)
#define D2MAX mask(D2WIDTH)
#define D2OFFSET midpoint(D2WIDTH)
#define D2DISCARD 3 // How many LSBits to discard
#define INVALID_INT ((1<<15)-1)
#define INVALID_LONG ((int)(((1UL)<<31)-1))

struct PackedRecord {
 protected:
   int curPtr;
   long curD1;
   long curD2;
   uint16_t data[NPOINTS];

   inline int getD1(int pos) {
      if (pos < 0 || pos > curPtr)
         return INVALID_INT;
      int rv = (int)(data[pos] & D1MAX);
      if ((rv == 0) || (rv == D1MAX))
         return INVALID_INT;
      else return rv-D1OFFSET;
   }
   
   inline int getD2(int pos) {
      if (pos < 0 || pos > curPtr)
         return INVALID_INT;
      int rv = (int)(data[pos]>>D1WIDTH);
      if ((rv == 0) || (rv == D2MAX))
         return INVALID_INT;
      else return ((rv-D2OFFSET)<<D2DISCARD);
   }

 public:
   void init() {
      curPtr = -1;
      curD1 = curD2 = 0;
   }

   void push(long d1, long d2) {
      //pr("push: ", curTime);
      //pr("\t", d1);
      //prln("\t", d2);
      
      long diffd1 = d1-curD1+D1OFFSET;
      long diffd2 = ((d2-curD2+midpoint(D2DISCARD))>>D2DISCARD)+D2OFFSET;
      curD1 = d1; curD2 = d2;
      if (diffd1 > D1MAX)
         diffd1 = D1MAX;
      if (diffd1 < 0)
         diffd1 = 0;
      if (diffd2 > D2MAX)
         diffd2 = D2MAX;
      if (diffd2 < 0)
         diffd2 = 0;
      data[++curPtr] = ((diffd2<<D1WIDTH)|diffd1);
      //pr("stored ", diffd1); pr("\t", diffd2); prln("\t", ((diffd2<<D1WIDTH)|diffd1));
   }

   inline long getD1(int pos, long last) {
      long rv = getD1(pos);
      if (rv == INVALID_INT) return INVALID_LONG;
      return last+rv;
   }

   inline long getD2(int pos, long last) {
      long rv = getD2(pos);
      if (rv == INVALID_INT) return INVALID_LONG;
      return last+rv;
   }
};

struct Record: public PackedRecord {
   char sampleCycle;
   char cycleCount;
   int acc_cycle_ms;
   int dec_cycle_ms;
   int lastAcc;
   int accDoneTime;
   int lastDec;
   int decDoneTime;
   int timeDiff;
   long lastPos;

 public:
   void init() {
      PackedRecord::init();
      sampleCycle = 1;
      cycleCount = 0;
      acc_cycle_ms = dec_cycle_ms = 10;
      lastAcc = lastDec = -1;
      timeDiff = 0;
   }

   void init(int sample_cycle, int acc_cycle, int dec_cycle) {
      sampleCycle = sample_cycle;
      acc_cycle_ms = acc_cycle;
      dec_cycle_ms = dec_cycle;
   }

   void push(long d1, long d2) {
      if (++cycleCount == sampleCycle) {
         cycleCount = 0;
         PackedRecord::push(d1, d2);
      }
   }

   void done_acc(long d1, long d2) {
      cycleCount = sampleCycle-1; // ensures next will be recorded
      push(d1, d2);
      lastAcc = curPtr;
      accDoneTime=curTime;
   }

   void done_dec(long d1, long d2) {
      cycleCount = sampleCycle-1;
      push(d1, d2);
      lastDec = curPtr;
      decDoneTime=curTime;
   }

   void finish(long pos) {
      timeDiff = millis() - start_time - curTime;
      lastPos = pos;
   }

   void print_all() {
      int i;
      pr("sampleCycle=", sampleCycle);
      pr("\tacc_cycle_ms=", acc_cycle_ms);
      prln("\tdec_cycle_ms=", dec_cycle_ms);
      pr("lastAcc=", lastAcc);
      pr("\taccDoneTime=", accDoneTime);
      pr("\tlastDec=", lastDec);
      pr("\tdecDoneTime=", decDoneTime);
      prln("\ttimeDiff=", timeDiff);
      if (lastAcc < 0 || lastAcc >= NPOINTS || lastDec < 0
             || lastDec >= NPOINTS || curPtr < 0 || curPtr >= NPOINTS)
        return;
      long pwm=0; long pos; long lastpos=0; long time=0; long lasttime=0;
      for (i=0; i <= curPtr; i++) {
         long inc = 0;
         if (i < lastAcc)
            inc = sampleCycle*acc_cycle_ms;
         else if (i == lastAcc)
            time = accDoneTime;
         else if (i < lastDec)
            inc = sampleCycle*dec_cycle_ms;
         else if (i == lastDec)
            time = decDoneTime;
         else inc = sampleCycle*acc_cycle_ms;
         time += inc;
         pwm = getD1(i, pwm);
         pos = getD2(i, lastpos);
         if (i == curPtr) pos = lastPos;
         pr(time);
         pr("\t", pwm*PWM_SCALE);
         pr("\t", pos);
         prln("\t", (pos-lastpos)*1000/((float)(time-lasttime)));
         lastpos = pos;
         lasttime = time;
      }
   }
};

Record record __attribute__ ((section (".noinit")));

void reachSpd(int iter, int minPwm, int maxPwm,
              long dist, float targetSpd, int cycle_ms, int pwmStep, int stop) {
// spdPwm is increased or decreased by pwmStep per cycle_ms until targetSpd is
// reached. (NOTE: targetSpd is measured in encoder ticks per cycle_ms.) From
// here on, spdPwm will be adjusted to maintain targetSpd. Function will return
// when dist has been traversed. For safety, spdPwm will be limited between
// minPwm and maxPwm, and the routine will be limited to a maximum of iter
// cycles.

   if (stop && (curSpd < (targetSpd*4)/3)) {
      spdPwm = 0;
      writeToMotor(spdPwm);
   }
   curPos = getEncoder();
   long targetPos = curPos + dist;
   /*
   pr("reachSpd(iter=", iter);
   pr(", minPwm=", minPwm);
   pr(", maxPwm=", maxPwm);
   pr(", dist=", dist);
   pr(", targetSpd=", targetSpd);
   pr(", cycle_ms=", cycle_ms);
   pr(", pwmStep=", pwmStep);
   prln(") at pos=", curPos);
   */
   for (int i=0; i < iter; i++) {
      delay(cycle_ms);
      curTime += cycle_ms;
      lastPos = curPos;
      curPos = getEncoder();
      curSpd = curPos - lastPos;
      if (curPos >= targetPos) {
         if (stop) {
            if (curSpd < 2) {
               spdPwm=0;
               return;
            }
            else if (curSpd > targetSpd) {
              if (spdPwm >= minPwm+pwmStep)
                 spdPwm -= pwmStep;
            }
            else spdPwm = 0;
         }
         else return;
      }
      else if (curSpd < targetSpd) {
         if (spdPwm+pwmStep < maxPwm)
            spdPwm += pwmStep;
         else spdPwm = maxPwm;
      }
      else if (curSpd > targetSpd) {
         if (spdPwm > minPwm+pwmStep)
            spdPwm -= pwmStep;
         else spdPwm = minPwm;
      }
      writeToMotor(spdPwm);
      if (i < iter-1) record.push(spdPwm, curPos);
   }
}

void reachPosAndSpd(int iter, int minPwm, int maxPwm, long targetPos,
                    float targetSpd, int cycle_ms, int pwmStep) {
// Increase or decrease spdPwm by 1 per cycle_ms in order to reach targetPos,
// with a terminal speed of targetSpd. NOTE: position is measured in encoder
// ticks, and speed in ticks per cycle_ms. Function will return as soon as
// targetPos is reached; reaching targetSpd is a best-effort proposition. For
// safety, spdPwm will be limited between minPwm and maxPwm, and the routine
// will be limited to a maximum of iter cycles.

   curPos = getEncoder();
   /*
   pr("reachPosAndSpd(iter=", iter);
   pr(", minPwm=", minPwm);
   pr(", maxPwm=", maxPwm);
   pr(", targetPos=", targetPos);
   pr(", targetSpd=", targetSpd);
   pr(", cycle_ms=", cycle_ms);
   pr(", pwmStep=", pwmStep);
   prln(") at pos=", curPos);
   */
   long lastSpd = curSpd;
   long lastDist = targetPos - curPos;
   long targetDec, curDec;

   for (int i=0; i < iter; i++) {
      delay(cycle_ms);
      curTime += cycle_ms;
      lastPos = curPos;
      curPos = getEncoder();
      curSpd = curPos - lastPos;
      curDec = (lastSpd - curSpd);

#ifdef USE_INIT_SPD
      curDec /= (i+1);
#else
      lastSpd = curSpd;
      lastDist = targetPos - curPos;
#endif
      targetDec = (lastSpd-targetSpd)*(lastSpd+targetSpd)/lastDist;// 2x
      //pr("targetDec=",targetDec);pr(",curDec=",curDec);prln(",curSpd=", curSpd);

      if (curPos >= targetPos - 300) {
        if (curSpd <= (targetSpd*5)/4)
           return;
        else if (spdPwm >= minPwm+pwmStep)
           spdPwm -= pwmStep;
      }
      else if (curDec > targetDec) {
            if (spdPwm+pwmStep < maxPwm)
               spdPwm += pwmStep;
            else spdPwm = maxPwm;
      }
      else if (curDec < targetDec) {
         if (spdPwm > minPwm+pwmStep)
            spdPwm -= pwmStep;
         else spdPwm = minPwm;
      }
      writeToMotor(spdPwm);
      if (i < iter-1) record.push(spdPwm, curPos);
   }
}
                 
void go(long targetDist, float accFrac, long crawlDist, 
        int timeout, int minPwm, int maxPwm, int minCrawlPwm, int maxCrawlPwm,
        float targetSpd, float targetAcc, float targetDec, float crawlSpd) {
// The whole program: move targetDist (in ticks), reaching a max speed of
// targetSpd (ticks/sec), and then decelerating to crawlSpd, and then finally
// stop. Acceleration phase lasts for accFrac fraction of targetDist.
// Acceleration should be maintained at targetAcc (pwm/sec^2) until targetSpd is
// reached, unless pwm value reaches above maxPwm or below minPwm, where it
// would be capped. The last crawlDist is covered at crawlSpd, with pwm value
// limited between minCrawlPwm and maxCrawlPwm. In between the two phases is the
// celeration phase, which needs no additional parameters, since the starting
// speed, ending speed, starting position and end position are already
// determined. However, we need to be given a target rate to start the
// deceleration rate: in particular, to set the dec_cycle_ms. The whole program
// should stop in timeout seconds.

   //record.print_all();
   record.init();
   
   curPos = getEncoder();
   long stopPos = curPos + targetDist;
   long decPos = stopPos - crawlDist;

   float accDist = targetDist*accFrac;

   int pwmStep1, pwmStep2;
   int acc_cycle_ms = 1000/targetAcc;
   pwmStep1 = 40/acc_cycle_ms;
   if (pwmStep1 == 0) pwmStep1 = 1;
   acc_cycle_ms *= pwmStep1;
   if (acc_cycle_ms > 50) {
      prln("Acceleration too slow\n");
      return;
   }

   int dec_cycle_ms = 1000/targetDec;
   pwmStep2 = 40/dec_cycle_ms;
   if (pwmStep2 == 0) pwmStep2 = 1;
   dec_cycle_ms *= pwmStep2;
   if (dec_cycle_ms > 50) {
      prln("Deceleration too slow\n");
      return;
   }
   
   int sampleCycle = 1+(timeout*1000)/(min(acc_cycle_ms, dec_cycle_ms)*NPOINTS);
   if (pwmStep1*sampleCycle*2 >= D1MAX) {
      prln("Acceleration too fast\n");
      return;
   }
   else if (pwmStep2*sampleCycle*2 >= D1MAX) {
      prln("Deceleration too fast\n");
      return;
   }

   record.init(sampleCycle, acc_cycle_ms, dec_cycle_ms);

#ifndef NO_ARDUINO
   int tmaxPwm = (targetSpd/0.95)/400;
   int tminPwm = -0.8*tmaxPwm;
#else
   int tmaxPwm = maxPwm;
   int tminPwm = minPwm;
#endif
   
   if (maxPwm < tmaxPwm) tmaxPwm = maxPwm;
   if (minPwm > tminPwm) tminPwm = minPwm;

   reachSpd((timeout*600)/acc_cycle_ms, tminPwm, tmaxPwm,
            accDist, (targetSpd*acc_cycle_ms)/1000, acc_cycle_ms, pwmStep1, 0);
   record.done_acc(spdPwm, curPos);

   curSpd = (curSpd*dec_cycle_ms+(acc_cycle_ms/2))/acc_cycle_ms;
   reachPosAndSpd((timeout*800-curTime)/dec_cycle_ms, tminPwm, tmaxPwm,
                  decPos, (targetSpd*dec_cycle_ms)/10000, dec_cycle_ms, pwmStep2);
   record.done_dec(spdPwm, curPos);

   curSpd = (curSpd*acc_cycle_ms+(dec_cycle_ms/2))/dec_cycle_ms;
   reachSpd((timeout*1000-curTime)/acc_cycle_ms, minCrawlPwm, maxCrawlPwm,
            stopPos-curPos, (crawlSpd*acc_cycle_ms)/1000, acc_cycle_ms, 1, 1);
   writeToMotor(0);
   record.push(spdPwm, curPos);
   record.finish(curPos);
   writeToMotor(0);
}

#ifdef NO_ARDUINO
int32_t main(int32_t argc, char *argv[]) {
#ifndef TEST_MODEL
   if (argc != 13) {
      cout << "Usage: " << argv[0] << " dist accFrac crawlDist\n";
      cout << "\ttimeout minPwm maxPwm minCrawlPwm maxCrawlPwm\n";
      cout << "\ttargetSpd targetAcc targetDec crawlSpd\n";
      exit(1);
   }
   go((long)atoi(argv[1]), atof(argv[2]), (long)atoi(argv[3]),
      (int)atoi(argv[4]), (int)atoi(argv[5]), (int)atoi(argv[6]),
      (int)atoi(argv[7]), (int)atoi(argv[8]), atof(argv[9]),
      atof(argv[10]), atof(argv[11]), atof(argv[12]));
   record.print_all();
#else
   for (int i=0; i < 4; i++) {
      writeToMotor(-100);
      delay(4000);
      curTime += 4;
      writeToMotor(100);
      delay(4000);
      curTime += 4;
   }
   writeToMotor(0);
   delay(2000);
   curTime += 2;
#endif
}
#else
void loop() {
  delay(5000);
  
  if (start_time == 0) {
     start_time = millis();
     int iter = 0;
     long targetDist = 60000; //26000 before
     float accFrac = 0.4;
     long crawlDist = 3000;
     int timeout = 15;
     int minPwm = -60; //0 before
     int maxPwm = 80; //80;
     int minCrawlPwm = 5;
     int maxCrawlPwm = 12;
     float targetSpd = 31000; //10000 before
     float targetAcc = 125;
     float targetDec = 75;
     float crawlSpd = 600;
     go(targetDist, accFrac, crawlDist, timeout, minPwm, maxPwm, minCrawlPwm,
     maxCrawlPwm, targetSpd, targetAcc, targetDec, crawlSpd);
     //long s = getEncoder();
     //long e = s+CRAWL_DIST;
     //crawlToEnd(s, e, 0, 50);
  }
}

void setup() {
  pinMode(pinPwm, OUTPUT);
  pinMode(pinDir, OUTPUT);
  pinMode(encoder_a, INPUT_PULLUP);
  pinMode(encoder_b, INPUT_PULLUP);
  TCCR1B = (TCCR1B & 0b11111000) | 0x02;
  attachInterrupt(digitalPinToInterrupt(encoder_a), encoderPinChangeA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_b), encoderPinChangeB, CHANGE);
  Serial.begin(9600);
     record.print_all();
  }

#endif

