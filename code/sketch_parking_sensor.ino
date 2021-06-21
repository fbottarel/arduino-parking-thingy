#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <VL53L1X.h>

// NeoPixel array configuration
#define NEOPIXEL_PIN          2
#define NEOPIXEL_N_LED        12

// Knobs configuration (ANALOG PINS)
#define MIN_KNOB_PIN          2
#define MAX_KNOB_PIN          3

// Distance reading limits (mm)
#define MIN_DISTANCE_LOWER    100
#define MIN_DISTANCE_UPPER    500
#define MAX_DISTANCE_LOWER    1000
#define MAX_DISTANCE_UPPER    2000

// Led brightness (0-100)
#define LED_BRIGHTNESS 10

#define LOOP_MS               50

// Create the NeoPixel object
Adafruit_NeoPixel pixels(NEOPIXEL_N_LED, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Create the sensor object
VL53L1X sensor;

int sensor_distance;
int min_distance_threshold, max_distance_threshold;

void init_pixels()
{
  // Do a nice spin to show we are booting up 
  for (int idx=0; idx<NEOPIXEL_N_LED; idx++)
  {
    pixels.setPixelColor(idx, pixels.Color(int(float(LED_BRIGHTNESS)/100.0 * 255), 0, 0));
    delay(50);
    pixels.show();
  }
  for (int idx=0; idx<NEOPIXEL_N_LED; idx++)
  {
    pixels.setPixelColor(idx, pixels.Color(int(float(LED_BRIGHTNESS)/100.0 * 255), int(float(LED_BRIGHTNESS)/100.0 * 255), 0));
    delay(50);
    pixels.show();
  }
  for (int idx=0; idx<NEOPIXEL_N_LED; idx++)
  {
    pixels.setPixelColor(idx, pixels.Color(0, int(float(LED_BRIGHTNESS)/100.0 * 255), 0));
    delay(50);
    pixels.show();
  }
  for (int idx=0; idx<NEOPIXEL_N_LED; idx++)
  {
    pixels.setPixelColor(idx, pixels.Color(0, 0, int(float(LED_BRIGHTNESS)/100.0 * 255)));
    delay(50);
    pixels.show();
  }
}

void set_pixels(const int distance_reading){

  pixels.clear();

  if (distance_reading >= max_distance_threshold)
  {
    // If over max threshold, all blue!
    for (int idx=0; idx<NEOPIXEL_N_LED; idx++)
      pixels.setPixelColor(idx, pixels.Color(0, 0, int(float(LED_BRIGHTNESS)/100.0 * 255)));
  }
  else if (distance_reading <= min_distance_threshold)
  {
    // If under min threshold, all red!
    for (int idx=0; idx<NEOPIXEL_N_LED; idx++)
      pixels.setPixelColor(idx, pixels.Color(int(float(LED_BRIGHTNESS)/100.0 * 255), 0, 0));
  }  
  else {
    // Compute how many pixels to turn on
    float step = (max_distance_threshold - min_distance_threshold) / NEOPIXEL_N_LED;
    int leds_to_turn_on = NEOPIXEL_N_LED - min(NEOPIXEL_N_LED, int(distance_reading / step)) + 1;

    for(int idx=0; idx<leds_to_turn_on; idx++)
    {
      if (idx < NEOPIXEL_N_LED / 3)
        pixels.setPixelColor(idx, pixels.Color(0, int(float(LED_BRIGHTNESS)/100.0 * 255), 0));
      else if (idx < NEOPIXEL_N_LED / 3 * 2)
        pixels.setPixelColor(idx, pixels.Color(int(float(LED_BRIGHTNESS)/100.0 * 255), int(float(LED_BRIGHTNESS)/100.0 * 255), 0));
      else if (idx < NEOPIXEL_N_LED)
        pixels.setPixelColor(idx, pixels.Color(int(float(LED_BRIGHTNESS)/100.0 * 255), 0, 0));
    }
  }

  pixels.show();
}

int getMinFromKnob()
{
  // Raw analog reading is between 0 and 1024 
  return MIN_DISTANCE_LOWER + int((1024 - analogRead(MIN_KNOB_PIN)) * ((MIN_DISTANCE_UPPER - MIN_DISTANCE_LOWER) / 1024.0));
}

int getMaxFromKnob()
{
  // Raw analog reading is between 0 and 1024 
  return MAX_DISTANCE_LOWER + int((1024 - analogRead(MAX_KNOB_PIN)) * ((MAX_DISTANCE_UPPER - MAX_DISTANCE_LOWER) / 1024.0));
}

void setup() {

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }

  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);

  pixels.begin();
  pixels.clear();
  pixels.show();

  init_pixels();

  sensor_distance = MAX_DISTANCE_UPPER;

  min_distance_threshold = MIN_DISTANCE_LOWER;
  max_distance_threshold = MAX_DISTANCE_UPPER;

  Serial.begin(9600);

}

void loop() {

  min_distance_threshold = getMinFromKnob();
  max_distance_threshold = getMaxFromKnob();

  if (Serial.available() > 0)
  {
    sensor_distance = int(Serial.parseInt());
    set_pixels(sensor_distance);

  }

  // Remove endline character
  while (Serial.available() != 0 ) Serial.read();

  sensor_distance = sensor.read();
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println(sensor_distance);
  Serial.println(min_distance_threshold);
  Serial.println(max_distance_threshold);
  Serial.println();
  set_pixels(sensor_distance);

  delay(LOOP_MS);  

}
