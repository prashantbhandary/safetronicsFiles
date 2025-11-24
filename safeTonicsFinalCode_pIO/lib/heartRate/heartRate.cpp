#include "heartRate.h"

static long lastBeatTime = 0;
static long delta = 0;
static int beatsPerMinute = 0;
static boolean firstBeat = true;
static boolean secondBeat = false;
static int P = 512;
static int T = 512;
static int thresh = 525;
static int amp = 100;
static boolean Pulse = false;
static boolean QS = false;

boolean checkForBeat(long sample) {
  boolean beatDetected = false;
  
  int Signal = sample;
  
  long N = millis() - lastBeatTime;
  
  // Find peak and trough
  if(Signal < thresh && N > (60000/220)) {
    if (Signal < T) {
      T = Signal;
    }
  }
  
  if(Signal > thresh && Signal > P) {
    P = Signal;
  }
  
  // Look for beat
  if (N > 250) {
    if ( (Signal > thresh) && (Pulse == false) && (N > (60000/220)) ) {
      Pulse = true;
      beatDetected = true;
      lastBeatTime = millis();
      
      int rate = 60000 / N;
      if(rate > 60 && rate < 100) {
        beatsPerMinute = rate;
      }
    }
  }
  
  if (Signal < thresh && Pulse == true) {
    Pulse = false;
    amp = P - T;
    thresh = amp/2 + T;
    P = thresh;
    T = thresh;
  }
  
  if (N > 2500) {
    thresh = 512;
    P = 512;
    T = 512;
  }
  
  return beatDetected;
}