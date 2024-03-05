/**
 * Based on: 
 * - https://github.com/wilhelmzeuschner/arduino_thermal_camera_with_sd_and_img_processing.
 * - https://github.com/sparkfun/SparkFun_MLX90640_Arduino_Example
 * 
 * Hardware:
 * - ESP32: https://www.espressif.com/en/products/hardware/esp32-devkitc/overview
 * - Sensor: https://shop.pimoroni.com/products/mlx90640-thermal-camera-breakout
 * - Display: https://www.amazon.de/gp/product/B07DPMV34R/, https://www.pjrc.com/store/display_ili9341.html
 */


#include <TFT_eSPI.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

#define EMMISIVITY 0.95
#define INTERPOLATE false

#define C_BLUE Display.color565(0,0,255)
#define C_RED Display.color565(255,0,0)
#define C_GREEN Display.color565(0,255,0)
#define C_WHITE Display.color565(255,255,255)
#define C_BLACK Display.color565(0,0,0)
#define C_LTGREY Display.color565(200,200,200)
#define C_DKGREY Display.color565(80,80,80)
#define C_GREY Display.color565(127,127,127)

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8 //Default shift for MLX90640 in open air
paramsMLX90640 mlx90640;


TFT_eSPI Display = TFT_eSPI();


// Added for measure Temp
boolean measure = true;
float centerTemp;
unsigned long tempTime = millis();
unsigned long tempTime2 = 0;

// start with some initial colors
float minTemp = 20.0;
float maxTemp = 40.0;


// variables for interpolated colors
byte red, green, blue;

// variables for row/column interpolation
float intPoint, val, a, b, c, d, ii;
int x, y, i, j;


// array for the 32 x 24 measured tempValues

float tempValues[32*24];

// Output size
#define O_WIDTH 135
#define O_HEIGHT 240

float **interpolated = NULL;
uint16_t *imageData = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("Hello.");

  // Connect thermal sensor.
  Wire.begin(16,17,400000);
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0) {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring.");
  }
  else {
    Serial.println("MLX90640 online!");
  }
  // Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0) Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0) Serial.println("Parameter extraction failed");
  // Set refresh rate
  MLX90640_SetRefreshRate(MLX90640_address, 0x05); // Set rate to 8Hz effective - Works at 800kHz
  // Once EEPROM has been read at 400kHz we can increase
  Wire.setClock(800000);

  // Set up Display.
  pinMode(TFT_DC, OUTPUT);
  SPI.begin();
  SPI.setFrequency(80000000L);
  Display.begin();
  Display.setRotation(3);
  Display.fillScreen(C_BLACK);


  // Prepare interpolated array
  interpolated = (float **)malloc(O_HEIGHT * sizeof(float *));
  for (int i=0; i<O_HEIGHT; i++) {
    interpolated[i] = (float *)malloc(O_WIDTH * sizeof(float));
  }

  // Prepare imageData array
  imageData = (uint16_t *)malloc(O_WIDTH * O_HEIGHT * sizeof(uint16_t));

  // get the cutoff points for the color interpolation routines
  // note this function called when the temp scale is changed
  setAbcd();

  Display.drawLine(0, 0, 0, 134, TFT_YELLOW);  // zlta ciara os Y x=0
  Display.drawLine(0, 0, 239, 0, TFT_PINK);    // ruzova ciara os X y=0
}


void loop() {
  /*
  while(Serial.available() == 0);
  while(Serial.available() > 0){
    Serial.read();
  }
  */
  
  tempTime = millis();
  
  readTempValues();
  setTempScale();

  // zero the center temps
  /*tempValues[383 - 16] = 0;
  tempValues[383 - 15] = 0;
  tempValues[384 + 15] = 0;
  tempValues[384 + 16] = 0;
  */
  drawPicture();
  drawMeasurement();
}


// Read pixel data from MLX90640.
void readTempValues() {
  for (byte x = 0 ; x < 2 ; x++) // Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, EMMISIVITY, tr, tempValues);
  }
}


int row;
float temp, temp2;

void interpolate() {
  for (row=0; row<24; row++) {
    for (x=0; x<O_WIDTH; x++) {
      temp  = tempValues[(31 - (x/7)) + (row*32) + 1];
      temp2 = tempValues[(31 - (x/7)) + (row*32)];
      interpolated[row*7][x] = lerp(temp, temp2, x%7/7.0);
    }
  }
  for (x=0; x<O_WIDTH; x++) {
    for (y=0; y<O_HEIGHT; y++) {
      temp  = interpolated[y-y%7][x];
      temp2 = interpolated[min((y-y%7)+7, O_HEIGHT-7)][x];
      interpolated[y][x] = lerp(temp, temp2, 1);//y%7/7.0);
    }
  }
}


// Linear interpolation
float lerp(float v0, float v1, float t) {
  return v0 + t * (v1 - v0);
}


void drawPicture() {

  if (INTERPOLATE) {
    interpolate();
    for (y=0; y<O_HEIGHT; y++) {
      for (x=0; x<O_WIDTH; x++) {
        imageData[(y*O_WIDTH) + x] = getColor(interpolated[y][x]);
      }
    }
    Display.pushImage(8, 8, O_WIDTH, O_HEIGHT, imageData);
  }
  else {
    for (y=0; y<24; y++) {
      for (x=0; x<32; x++) {
        //Display.fillRect(8 + x*7, 8 + y*7, 7, 7, getColor(tempValues[(31-x) + (y*32)]));
        
        Display.fillRect(8 + x*5, 8 + y*5, 5, 5, getColor(tempValues[(31-x) + (y*32)]));

      }
    }
  }
}



// Get color for temp value.
uint16_t getColor(float val) {
  /*
    pass in value and figure out R G B
    several published ways to do this I basically graphed R G B and developed simple linear equations
    again a 5-6-5 color display will not need accurate temp to R G B color calculation

    equations based on
    http://web-tech.ga-usa.com/2012/05/creating-a-custom-hot-to-cold-temperature-color-gradient-for-use-with-rrdtool/index.html

  */

  red = constrain(255.0 / (c - b) * val - ((b * 255.0) / (c - b)), 0, 255);

  if ((val > minTemp) & (val < a)) {
    green = constrain(255.0 / (a - minTemp) * val - (255.0 * minTemp) / (a - minTemp), 0, 255);
  }
  else if ((val >= a) & (val <= c)) {
    green = 255;
  }
  else if (val > c) {
    green = constrain(255.0 / (c - d) * val - (d * 255.0) / (c - d), 0, 255);
  }
  else if ((val > d) | (val < a)) {
    green = 0;
  }

  if (val <= b) {
    blue = constrain(255.0 / (a - b) * val - (255.0 * b) / (a - b), 0, 255);
  }
  else if ((val > b) & (val <= d)) {
    blue = 0;
  }
  else if (val > d) {
    blue = constrain(240.0 / (maxTemp - d) * val - (d * 240.0) / (maxTemp - d), 0, 240);
  }

  // use the displays color mapping function to get 5-6-5 color palet (R=5 bits, G=6 bits, B-5 bits)
  return Display.color565(red, green, blue);
}


void setTempScale() {
  float lmin = 2000;
  float lmax = -200;
  //Serial.println();
  //Serial.println();
  for (y=0; y<24; y++) {
    for (x=0; x<32; x++) {

      lmin = min(lmin, tempValues[(31-x) + (y*32)]);
      lmax = max(lmax, tempValues[(31-x) + (y*32)]);
      //Serial.print(tempValues[(31-x) + (y*32)], 0);
      //delay(12);
      //Serial.print(",");
      
    }
    //Serial.println();
  }
  //Serial.println();
  maxTemp = lmax;
  minTemp = lmin;

  //Serial.print("max min: ");
  //Serial.print(lmax);
  //Serial.print(", ");
  //Serial.print(lmin);

  setAbcd();
  
  Display.setTextFont(2);
  Display.setTextSize(1);

  Display.setCursor(180, 0);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(maxTemp).substring(0, 5));

  Display.setCursor(180, 20);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.print(String(minTemp).substring(0, 5));
}


// Function to get the cutoff points in the temp vs RGB graph.
void setAbcd() {
  a = minTemp + (maxTemp - minTemp) * 0.2121;
  b = minTemp + (maxTemp - minTemp) * 0.3182;
  c = minTemp + (maxTemp - minTemp) * 0.4242;
  d = minTemp + (maxTemp - minTemp) * 0.8182;
}

// Draw a circle + measured value.
void drawMeasurement() {

  // Mark center measurement
  Display.drawCircle(6+16*5, 7+12*5, 3, TFT_WHITE);
  Display.drawCircle(0, 0, 5, TFT_RED);
  // Measure and print center temperature
  centerTemp = (tempValues[383 - 16] + tempValues[383 - 15] + tempValues[384 + 15] + tempValues[384 + 16]) / 4;
  Display.setCursor(180, 40);
  Display.setTextColor(TFT_WHITE, TFT_BLACK);
  Display.setTextFont(2);
  Display.setTextSize(1);
  Display.print(String(centerTemp).substring(0, 5) + " °C");
}
