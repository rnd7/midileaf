#include "arduinoFFT.h" // fast fourier transform library

/*
   Code for Arduino Mega 2560

   MONITOR out using Serial (TX, Pin1)
   MIDI out using Serial1 (TX1, Pin18)

*/



/*
   Compiler Statements. Occurances in the following code will be replaced with
   the given value
*/
#define SILENT_MODE 0
#define PLOT_MODE 1
#define NOTE_MODE 2
#define LOG_MODE 3

#define MIDI_BAUD_RATE 31250
#define SERIAL_BAUD_RATE 9600

#define NUM_SENSORS 3
#define FFT_SAMPLES 128
#define FFT_BANDS 4

const unsigned int SERIAL_MODE = NOTE_MODE; // Debug output

const unsigned int BPM = 80; // Base BPM
const unsigned int TICK = round(60 * 1000 / (BPM * 4)); // 1/16 note interval

const unsigned int SENSOR_SAMPLE_INTERVAL = 13; // Sampler Interval
const unsigned int FFT_FLUSH_INTERVAL = 47; // Flush FFT after interval
const unsigned int SENSOR_THRESHOLD_INTERVAL = 253; // Update dynamic min/max
const unsigned int SENSOR_FLUSH_INTERVAL = 167; // sensor flush cycle
const unsigned int PLOT_INTERVAL = 1000; // Debug Output Interval
const unsigned int VOICE_MANAGER_INTERVAL = TICK; // shortest note played

const float ANALOG_PRECISION = 1024.; // Precision of the Analog Input
const float SENSOR_HEADROOM = .01; // threshold above value used for dynamicMax
const float SENSOR_FOOTROOM = .01; // threshold below value used for dynamicMin

const unsigned int VOICES = (1 + FFT_BANDS); // Max simultaneously played notes

const byte NOTE_ON_CMD = 0x90; // MIDI note on command
const byte NOTE_OFF_CMD = 0x80; // MIDI note off command
const byte MAX_DATA = 0x7f; // MIDI note, velocity and other must not exceed
const byte MIN_NOTE = 0x2A; // lowest played midi note (lowest piano key 0x15)
const byte MAX_NOTE = 0x5F; // highest played midi note (highest piano key 0x6C)
const byte NOTE_RANGE = MAX_NOTE - MIN_NOTE; // midi note range
const unsigned int MIN_NOTE_TIME = TICK; // 1/16 Note
const unsigned int MAX_NOTE_TIME = TICK * 16 * 4; // 4/1 Note, 4 Bars

const byte MIN_FFT_NOTE = 0x15; // lowest played midi note (lowest piano key 0x15)
const byte MAX_FFT_NOTE = 0x4C; // highest played midi note (highest piano key 0x6C)
const byte FFT_NOTE_RANGE = MAX_NOTE - MIN_NOTE; // midi note range
const unsigned int MIN_FFT_NOTE_TIME = TICK * 4; // 1/4 Note
const unsigned int MAX_FFT_NOTE_TIME = TICK * 16 * 8; // 8/1 Note, 8 Bars
const unsigned int FFT_NOTE_TIME_RANGE = MAX_FFT_NOTE_TIME - MIN_FFT_NOTE_TIME;

const byte MIN_VELOCITY = 0x04; // Minimum key pressure
const byte MAX_VELOCITY = 0x6D; // Maximum key pressure
const byte VELOCITY_RANGE = MAX_VELOCITY-MIN_VELOCITY; // key pressure range

const float PLAY_THRESHOLD = .05; // All values below this threshold will be ignored (no contact)
const float DYNAMIC_MAX_FACTOR = .05; // How fast the upper limit converges
const float DYNAMIC_MIN_FACTOR = .03; // How fast the lower limit converges

/*
 * When current absolute Slope exceeds average absolute Slope
 * by this factor a melody note is triggered.
 */
const unsigned int SLOPE_TRIGGER_THRESHOLD = 1.9;

// 3rd party FFT transform library
const arduinoFFT FFT = arduinoFFT();

/* Struct to hold fft band data */
typedef struct
{
  int bin; // fft band bin index (proportional to freq)
  float amplitude; // fft band amplitude
} fftBand;

/* Struct to keep track of played notes */
typedef struct
{
  byte pitch; // note pitch (between 0x0 and 0x7f)
  unsigned long noteOnTime; // time when noteon was sent
  bool playing; // determine if voice is currently playing
  bool trigger; // true new note is triggerd
  byte triggerPitch; // pitch calculated on trigger
} voice;

/* Struct to accumulate values over time */
typedef struct {
  double value; // accumulate input
  unsigned long samples; // keep track of samples accumulated
} sampler;

/* Struct containing sensor data */
typedef struct
{
  unsigned int pin; // Analog Input Pin
  sampler fftSampler; // Accumulate Analog Read for FFT
  sampler valueSampler; // Accumulate Analog Read for Value
  float value; // Current sensor value between 0 and 1 proportional to 0-5V
  float average; // Average Value
  float dynamicMax; // Adaptive Max Value.
  float dynamicMin; // Adaptive Min Value.
  float dynamicRange; // dynamicMax - dynamicMin
  float slope; // Delta to previous value
  float averageSlope; // 
  unsigned int fftSamples; // Keep track of recorded fft samples
  double fftVReal[FFT_SAMPLES]; // Real numbers
  double fftVImag[FFT_SAMPLES]; // Imaginary Part
  fftBand bands[FFT_BANDS];
  byte midiChannel; // midi channel assigned to sensor
  voice voices[VOICES]; // simultaneous played voices
} sensor;

// Array containing all sensors
sensor sensors[NUM_SENSORS];

// Keep track of timings
unsigned long currentTime; // global time in millisecond
unsigned long lastTime = 0;
unsigned long lastSensorSample = 0;
unsigned long lastFFTFlush = 0;
unsigned long lastSensorFlush = 0;
unsigned long lastSensorThreshold = 0;
unsigned long lastVoiceManagerTime = 0;
unsigned long lastPlot = 0;

// Temporary Data
float prevFFTResult;
float fftDelta;
int fftResultdir;
int prevFFTResultDir;
bool mv = false;
float curAmp;
int curFreq;
float tmpAmp;
int tmpFreq;

float floatBuffer;

// iterator variables
int i;
int j;
int k;

byte midiChannelMapping[16] = {
  0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xA, 0xB, 0xC, 0xD, 0xF
};



//-----------------------------------------------------------------------------

/* Reset all */
void reset() {
  stopAllVoices();
  for (i = 0; i < NUM_SENSORS; i++) {
    sensors[i].midiChannel = midiChannelMapping[i];
    for (j = 0; j < VOICES; j++) {
      sensors[i].voices[j].pitch = 0x0;
      sensors[i].voices[j].noteOnTime = 0;
      sensors[i].voices[j].playing = false;
      sensors[i].voices[j].trigger = false;
    }
    sensors[i].pin = i;
    sensors[i].fftSampler.value = 0.;
    sensors[i].fftSampler.samples = 0;
    sensors[i].valueSampler.value = 0.;
    sensors[i].valueSampler.samples = 0;
    sensors[i].value = 0.;
    sensors[i].average = 0.;
    sensors[i].dynamicMax = 1.;
    sensors[i].dynamicMin = 0.;
    sensors[i].fftSamples = 0;
    for (j = 0; j < FFT_BANDS; j++) {
      sensors[i].bands[j].bin = -1;
      sensors[i].bands[j].amplitude = 0.;
    }

  }
}

/* Sample all analog inputs and accumulate values */
void sampleSensors() {
  for (i = 0; i < NUM_SENSORS; i++) {
    floatBuffer = analogRead(sensors[i].pin) / ANALOG_PRECISION;
    sensors[i].fftSampler.value += floatBuffer;
    sensors[i].fftSampler.samples++;
    sensors[i].valueSampler.value += floatBuffer;
    sensors[i].valueSampler.samples++;
  }
}

/* flush value sampler */
void flushSensor() {
  for (i = 0; i < NUM_SENSORS; i++) {
    floatBuffer = sensors[i].valueSampler.value / sensors[i].valueSampler.samples;
    sensors[i].slope = floatBuffer - sensors[i].value;
    sensors[i].averageSlope = (sensors[i].averageSlope + abs(sensors[i].slope)) / 2.;
    sensors[i].value = floatBuffer;
    sensors[i].average = (sensors[i].average + floatBuffer) / 2;
    sensors[i].dynamicMin = min(sensors[i].dynamicMin, floatBuffer);
    sensors[i].dynamicMax = max(sensors[i].dynamicMax, floatBuffer);
    sensors[i].valueSampler.value = 0;
    sensors[i].valueSampler.samples = 0;
  }
}

/* Update dynamic min/max. Continously approaching thresholds. */
void dynamicMinMax() {
  for (i = 0; i < NUM_SENSORS; i++) {
    floatBuffer = min(1., sensors[i].value + SENSOR_HEADROOM);
    sensors[i].dynamicMax -= (sensors[i].dynamicMax - floatBuffer) * DYNAMIC_MAX_FACTOR;
    floatBuffer = max(0., sensors[i].value - SENSOR_FOOTROOM);
    sensors[i].dynamicMin -= (sensors[i].dynamicMin - floatBuffer) * DYNAMIC_MIN_FACTOR;    
    sensors[i].dynamicRange = sensors[i].dynamicMax - sensors[i].dynamicMin;
  }
}

/* flush fft sampler */
void flushFFT() {
  for (i = 0; i < NUM_SENSORS; i++) {
    sensors[i].fftVReal[sensors[i].fftSamples] = sensors[i].fftSampler.value / sensors[i].fftSampler.samples - sensors[i].average; // average and center
    sensors[i].fftVImag[sensors[i].fftSamples] = 0.; // initialize Imaginary part on the fly
    sensors[i].fftSampler.value = 0;
    sensors[i].fftSampler.samples = 0;
    sensors[i].fftSamples++;
  }
}

/* Calculate fft bands by computing transform on sampled data */
void calcFFT(sensor &p) {
  FFT.Windowing(p.fftVReal, FFT_SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(p.fftVReal, p.fftVImag, FFT_SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(p.fftVReal, p.fftVImag, FFT_SAMPLES);

  prevFFTResult = 0.;
  fftDelta = 0.;
  fftResultdir = 0.;
  prevFFTResultDir = 0.;

  for (k = 0; k < FFT_BANDS; k++) {
    p.bands[k].bin = -1; // init, aka not found
    p.bands[k].amplitude = 0.; // reset amplitude
  }
  for (j = 0; j < FFT_SAMPLES >> 1; j++) {
    fftDelta = p.fftVReal[j] - prevFFTResult;
    if (fftDelta > 0) {
      fftResultdir = 1;
    } else if (fftDelta < 0) {
      fftResultdir = -1;
    } else {
      fftResultdir = 0;
    }
    // check for peaks
    if (prevFFTResultDir > fftResultdir) {
      mv = false;
      for (k = 0; k < FFT_BANDS; k++) {
        if (mv) {
          // move all following entries to the right, drop last
          curAmp = p.bands[k].amplitude;
          curFreq = p.bands[k].bin;
          p.bands[k].amplitude = tmpAmp;
          p.bands[k].bin = tmpFreq;
          tmpAmp = curAmp;
          tmpFreq = tmpFreq;
        } else if (p.fftVReal[j] > p.bands[k].amplitude) {
          // found better peak set value and move all following
          mv = true;
          tmpAmp = p.bands[k].amplitude;
          tmpFreq = p.bands[k].bin;
          p.bands[k].amplitude = p.fftVReal[j];
          p.bands[k].bin = j;
        }
      }
    }
    prevFFTResultDir = fftResultdir;
    prevFFTResult = p.fftVReal[j];
  }
  p.fftSamples = 0;
}

/* function to decide which note to play when */
void voiceManager()
{
  for (i = 0; i < NUM_SENSORS; i++ ) {
    if (sensors[i].value < PLAY_THRESHOLD) {
      // stop all voices when sensor value get close to null (Disconnected sensor)
      stopAllSensorVoices(i);
    } else {

      // MELODY VOICE
      
      // trigger
      if (abs(sensors[i].slope) > sensors[i].averageSlope * SLOPE_TRIGGER_THRESHOLD) {
        // trigger this voice due to uncommon high slope (ie react on touch)
        sensors[i].voices[0].triggerPitch = MIN_NOTE + ((sensors[i].value - sensors[i].dynamicMin) / sensors[i].dynamicRange) * NOTE_RANGE;
        if (sensors[i].voices[0].triggerPitch != sensors[i].voices[0].pitch) {
          sensors[i].voices[0].trigger = true;
        }
      }
      
      // note off
      if (
        sensors[i].voices[0].playing // when is playing
        && (
          sensors[i].voices[0].noteOnTime + MAX_NOTE_TIME <= currentTime // exceeding maximum note length
          || (sensors[i].voices[0].trigger && sensors[i].voices[0].noteOnTime + MIN_NOTE_TIME <= currentTime) // or after min note length when triggered
        )
      ) {
        // stop previous
        noteOff(sensors[i].midiChannel, sensors[i].voices[0].pitch);
        sensors[i].voices[0].playing = false;
      }

      // note on
      if (!sensors[i].voices[0].playing && sensors[i].dynamicRange > 0 ) {
        if (sensors[i].voices[0].trigger) {
          sensors[i].voices[0].pitch =  sensors[i].voices[0].triggerPitch;
        } else {
          sensors[i].voices[0].pitch = MIN_NOTE + ((sensors[i].value - sensors[i].dynamicMin) / sensors[i].dynamicRange) * NOTE_RANGE;
        }
        noteOn(sensors[i].midiChannel, sensors[i].voices[0].pitch, MIN_VELOCITY + abs(sensors[i].slope) / sensors[i].averageSlope * VELOCITY_RANGE);
        sensors[i].voices[0].noteOnTime = currentTime;
        sensors[i].voices[0].playing = true;
        sensors[i].voices[0].trigger = false;      
      }

      // FFT ACCOMPANIMENT VOICES
      
      for (j = 0; j < FFT_BANDS; j++) {
        
        // trigger
        if (
          sensors[i].bands[j].bin >= 0 
          && sensors[i].voices[1 + j].noteOnTime + MIN_FFT_NOTE_TIME + (1. - (float(sensors[i].voices[1 + j].pitch - MIN_FFT_NOTE) / FFT_NOTE_RANGE)) * FFT_NOTE_TIME_RANGE >= currentTime
        ) {
          // trigger this voice. higher pitches will played shorter. lower pitches last longer

          // calc note by position of fft band within freq spectrum 
          sensors[i].voices[1 + j].triggerPitch = MIN_FFT_NOTE + (sensors[i].bands[j].bin / float(FFT_SAMPLES >> 1))  * FFT_NOTE_RANGE;
          
          if (sensors[i].voices[1 + j].triggerPitch != sensors[i].voices[1 + j].pitch) {
            // only trigger when not pitch changed
            sensors[i].voices[1 + j].trigger = true;
          }
        }
        
        // note off
        if (
          sensors[i].voices[1 + j].playing // when is playing
          && (
            sensors[i].voices[1 + j].noteOnTime + MAX_FFT_NOTE_TIME <= currentTime // exceeding maximum note length
            || (sensors[i].voices[1 + j].trigger && sensors[i].voices[1 + j].noteOnTime + MIN_FFT_NOTE_TIME <= currentTime) // or after min note length when triggered
          )
        ) {
          // stop previous
          noteOff(sensors[i].midiChannel, sensors[i].voices[1 + j].pitch);
          sensors[i].voices[1 + j].playing = false;
        }

        // note on
        if (
          !sensors[i].voices[1 + j].playing 
          && sensors[i].bands[j].bin >= 0
          && sensors[i].dynamicRange > 0
        ) {
          if (sensors[i].voices[1 + j].trigger) {
            // use triggered note
            sensors[i].voices[1 + j].pitch = sensors[i].voices[1 + j].triggerPitch;
          } else {
            sensors[i].voices[1 + j].pitch = MIN_FFT_NOTE + (sensors[i].bands[j].bin / float(FFT_SAMPLES >> 1)) * FFT_NOTE_RANGE;
          }
          sensors[i].voices[1 + j].noteOnTime = currentTime;
          noteOn(sensors[i].midiChannel, sensors[i].voices[1 + j].pitch, MIN_VELOCITY + j / FFT_BANDS * VELOCITY_RANGE);
          sensors[i].voices[1 + j].playing = true;
          sensors[i].voices[1 + j].trigger = false;
        }
      }
      
    }
  }
}

/*  stop all voices played for all sensors */
void stopAllVoices() {
  for (i = 0; i < NUM_SENSORS; i++) {
    stopAllSensorVoices(i);
  }
}

/* Stop all voices played by a single sensor */
void stopAllSensorVoices(unsigned int index) {
  for (j = 0; j < VOICES; j++) {
    if (sensors[index].voices[j].playing) {
      noteOff(sensors[index].midiChannel, sensors[index].voices[j].pitch);
      sensors[index].voices[j].playing = false;
    }
  }
}

/* Trigger note on Command */
void noteOn(byte channel, byte pitch, byte velocity)
{
  midiSend(NOTE_ON_CMD + channel, pitch, velocity);
}

/* Trigger note off Command */
void noteOff(byte channel, byte pitch)
{
  midiSend(NOTE_OFF_CMD + channel, pitch, 0x0);
}

/* Send MIDI command via Serial output */
void midiSend(byte cmd, byte data1, byte data2)
{
  cmd = max(MAX_DATA+0x1, min(0xFF, cmd));
  data1 = max(0, min(MAX_DATA, data1));
  data2 = max(0, min(MAX_DATA, data2));
  Serial1.write(cmd);
  Serial1.write(data1);
  Serial1.write(data2);
}

/* formats data for arduino serial plotter */
void plot() {
  for (i = 0; i < NUM_SENSORS; i++) {
    for (k = 0; k < FFT_BANDS; k++) {
      Serial.print(max(0, sensors[i].bands[k].bin) / float(FFT_SAMPLES >> 1) * 100.);
      Serial.print(",");
    }
    Serial.print(abs(sensors[i].slope) * 100.);
    Serial.print(",");
    Serial.print(sensors[i].averageSlope * 100.);
    Serial.print(",");
    Serial.print(sensors[i].dynamicMin * 100.);
    Serial.print(",");
    Serial.print(sensors[i].average * 100.);
    Serial.print(",");
    Serial.print(sensors[i].value * 100.);
    Serial.print(",");
    Serial.println(sensors[i].dynamicMax * 100.);
  }
}

/* formats data for arduino serial plotter */
void plotNotes() {
  Serial.print(0.);
  Serial.print(",");
  Serial.print((float(MIN_FFT_NOTE) / MAX_DATA) * 100.);
  Serial.print(",");
  Serial.print((float(MIN_NOTE / MAX_DATA)) * 100.);
  Serial.print(",");
  for (i = 0; i < NUM_SENSORS; i++) {
    for (k = 0; k < VOICES; k++) {
      Serial.print((float(sensors[i].voices[k].pitch) / MAX_DATA) * 100.);
      Serial.print(",");
    }
  }
  Serial.print(",");
  Serial.print((float(MAX_FFT_NOTE) / MAX_DATA) * 100.);
  Serial.print(",");
  Serial.print((float(MAX_NOTE) / MAX_DATA) * 100.);
  Serial.print(",");
  Serial.println(100.);
}

/* formats data for arduino serial monitor */
void logger() {

  for (i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor: ");
    Serial.println(i);
    Serial.print("sensorMin: ");
    Serial.print(sensors[i].dynamicMin * 100.);
    //Serial.print(sensorMin[i]*100.);
    Serial.print(" sensorAvg: ");
    Serial.print(sensors[i].average * 100.);
    //Serial.print(sensorAvg[i]*100.);
    Serial.print(" sensorValue: ");
    Serial.print(sensors[i].value * 100.);
    //Serial.print(sensorValue[i]*100.);
    Serial.print(" sensorMin: ");
    Serial.print(sensors[i].dynamicMin * 100.);
    Serial.print(" sensorMax: ");
    Serial.println(sensors[i].dynamicMax * 100.);
    //Serial.println(sensorMax[i]*100.);

    for (k = 0; k < FFT_BANDS; k++) {
      Serial.print("Band: ");
      Serial.println(k);
      Serial.print("freq: ");
      Serial.print(sensors[i].bands[k].bin / 64. * 100.);
      //Serial.print(sensorFFTFrequency[i*FFT_BANDS+k]/64.*100.);
      Serial.print(" amp: ");
      Serial.println(sensors[i].bands[k].amplitude);
    }
    Serial.println("---");
  }
}

/* Setup Method called once on Arduino Startup */
void setup()
{
  // Set monitor baud rate
  Serial.begin(SERIAL_BAUD_RATE);

  // Set MIDI baud rate
  Serial1.begin(MIDI_BAUD_RATE);

  reset();
}

/* Loop Function. Called continously. */
void loop() {
  currentTime = millis();
  if (lastTime > currentTime) {
    // time overflow, reset all
    reset();
  } else {

    // most crucial for timing, run first
    if (currentTime - lastVoiceManagerTime >= VOICE_MANAGER_INTERVAL) {
      // flush the sensor buffer
      voiceManager();
      lastVoiceManagerTime = currentTime;
    }

    if (currentTime - lastSensorSample >= SENSOR_SAMPLE_INTERVAL) {
      // accumulate sensor values
      sampleSensors();
      lastSensorSample = currentTime;
    }

    // time critical
    if (currentTime - lastFFTFlush >= FFT_FLUSH_INTERVAL) {
      // flush the fft buffer
      flushFFT();
      // calculate freqs.
      lastFFTFlush = currentTime;
    }
    for (i = 0; i < NUM_SENSORS; i++) {
      if (sensors[i].fftSamples >= FFT_SAMPLES) {
        calcFFT(sensors[i]);
        break; // performance issue, continue next loop
      }
    }

    if (currentTime - lastSensorFlush >= SENSOR_FLUSH_INTERVAL) {
      // flush the sensor buffer
      flushSensor();
      lastSensorFlush = currentTime;
    }


    if (SERIAL_MODE > 0) {
      if (currentTime - lastPlot >= PLOT_INTERVAL) {
        if (SERIAL_MODE == PLOT_MODE) {
          plot();
        } else if(SERIAL_MODE == NOTE_MODE) {
          plotNotes();
        } else if (SERIAL_MODE == LOG_MODE) {
          logger();
        }
        lastPlot = currentTime;
      }
    }


    if (currentTime - lastSensorThreshold >= SENSOR_THRESHOLD_INTERVAL) {
      dynamicMinMax();
      lastSensorThreshold = currentTime;
    }
  }

  lastTime = currentTime;

}
