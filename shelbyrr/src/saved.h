#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <ESPAsyncWebServer.h>
#include <StringArray.h>
#include <SPIFFS.h>
#include <FS.h>

const char* ssid = "RUBENBARBOSA";
const char* password = "shelbyrr";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

boolean takeNewPhoto = false;

// Photo File Name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"

// OV2640 camera module pins (CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { text-align:center; }
    .vert { margin-bottom: 10%; }
    .hori{ margin-bottom: 0%; }
  </style>
</head>
<body>
  <div id="container">
    <h2>ESP32-CAM Last Photo</h2>
    <p>It might take more than 5 seconds to capture a photo.</p>
    <p>
      <button onclick="rotatePhoto();">ROTATE</button>
      <button onclick="capturePhoto()">CAPTURE PHOTO</button>
      <button onclick="location.reload();">REFRESH PAGE</button>
    </p>
  </div>
  <div><img src="saved-photo" id="photo" width="70%"></div>
</body>
<script>
  var deg = 0;
  function capturePhoto() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/capture", true);
    xhr.send();
  }
  function rotatePhoto() {
    var img = document.getElementById("photo");
    deg += 90;
    if(isOdd(deg/90)){ document.getElementById("container").className = "vert"; }
    else{ document.getElementById("container").className = "hori"; }
    img.style.transform = "rotate(" + deg + "deg)";
  }
  function isOdd(n) { return Math.abs(n % 2) == 1; }
</script>
</html>)rawliteral";

void cpuFreq(){
  uint32_t Freq = 0;
  Freq = getCpuFrequencyMhz();
  Serial.print("CPU Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getXtalFrequencyMhz();
  Serial.print("XTAL Freq = ");
  Serial.print(Freq);
  Serial.println(" MHz");
  Freq = getApbFrequency();
  Serial.print("APB Freq = ");
  Serial.print(Freq);
  Serial.println(" Hz");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Print ESP32 Local IP Address
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }

  // Turn-off the 'brownout detector'
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // OV2640 camera module
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 63;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 63;
    config.fb_count = 1;
  }
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/capture", HTTP_GET, [](AsyncWebServerRequest * request) {
    takeNewPhoto = true;
    request->send_P(200, "text/plain", "Taking Photo");
  });

  server.on("/saved-photo", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, FILE_PHOTO, "image/jpg", false);
  });

  // Start server
  server.begin();

}



// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

// Capture Photo and Save it to SPIFFS
void capturePhotoSaveSpiffs( void ) {
  camera_fb_t * fb = NULL; // pointer
  bool ok = 0; // Boolean indicating if the picture has been taken correctly

  do {
    // Take a photo with the camera
    Serial.println("Taking a photo...");

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }

    // Photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);

    // Insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    // Close the file
    file.close();
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
  } while ( !ok );
}

void loop() {
  if (takeNewPhoto) {
    capturePhotoSaveSpiffs();
    takeNewPhoto = false;
  }
  delay(1);
}

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  
  char * part_buf[64];
  static int64_t last_frame = 0;
  
  if(!last_frame) {
      last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){

    fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {

      decodeImage(fb);
      processImage();

      bool jpeg_converted = fmt2jpg((uint8_t *)imgPtr, imgWidth*imgHeight, imgWidth, imgHeight, PIXFORMAT_GRAYSCALE, 100, &_jpg_buf, &_jpg_buf_len);

      esp_camera_fb_return(fb);
      fb = NULL;

      if(!jpeg_converted){
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
      }
    }
    
    if(!RACEMODE){
      if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      }
      if(res == ESP_OK){
        res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
      if(res == ESP_OK){
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
    }

    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if(res != ESP_OK){
      break;
    }

    
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;

    Serial.printf("MJPG: %uKB %ums (%.1ffps)",(uint32_t)(_jpg_buf_len/1024),(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    Serial.println("");
  }

  last_frame = 0;
  return res;
}

int8_t kernelBlur [3][3] = {{1,1,1},
                            {1,1,1},
                            {1,1,1}};

int8_t kernelX [3][3] = {{-1,0,1},
                         {-2,0,2},
                         {-1,0,1}};

int8_t kernelY [3][3] = {{-1,-1,-1},
                         {0,0,0},
                         {1,1,1}};

void print_free_memory()
{
    Serial.println("\n>>>---------- Memory Info ----------<<<");
    Serial.print("Free Heap: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Free PSRAM: ");
    Serial.println(ESP.getFreePsram());
    Serial.print("Free sketch sapce: ");
    Serial.println(ESP.getFreeSketchSpace());
    Serial.print("Heap Size: ");
    Serial.println(ESP.getHeapSize());
    Serial.print("Max Alloc Heap: ");
    Serial.println(ESP.getMaxAllocHeap());
    Serial.print("Max Alloc Psram: ");
    Serial.println(ESP.getMaxAllocPsram());
    Serial.print("Psram Size: ");
    Serial.println(ESP.getPsramSize());
    Serial.println(">>>---------- Memory Info End ----------<<<\n");
}



void print_image_shape(camera_fb_t * frame){

     // print shape of image and total length (=heigth*width)
     Serial.print("Width: ");
     Serial.print(frame->width);
     Serial.print("\tHeigth: ");
     Serial.print(frame->height);
     Serial.print("\tLength: ");
     Serial.println(frame->len);
 }

void crop_image(camera_fb_t *fb, unsigned short cropLeft, unsigned short cropRight, unsigned short cropTop, unsigned short cropBottom)
{
    uint8_t newBuffer[(fb->width*fb->height)/SCALEFACTOR];
    unsigned int maxTopIndex = cropTop * fb->width;
    unsigned int minBottomIndex = ((fb->width*fb->height) - (cropBottom * fb->width));
    unsigned short maxX = fb->width - cropRight; // In pixels
    unsigned short newWidth = fb->width - cropLeft - cropRight;
    unsigned short newHeight = fb->height - cropTop - cropBottom;
    size_t newJpgSize = newWidth * newHeight;

    unsigned int writeIndex = 0;
    // Loop over all bytes
    
    for(int i = 0; i < fb->len; i+=2){
        // Calculate current X, Y pixel position
        int x = (i/2) % fb->width;

        // Crop from the top
        if(i < maxTopIndex){ continue; }

        // Crop from the bottom
        if(i > minBottomIndex){ continue; }

        // Crop from the left
        if(x <= cropLeft){ continue; }

        // Crop from the right
        if(x > maxX){ continue; }

        // If we get here, keep the pixels
        fb->buf[writeIndex++] = fb->buf[i];
        fb->buf[writeIndex++] = fb->buf[i+1];
    }
    
    
    fb->width = newWidth;
    fb->height = newHeight;
    fb->len = newJpgSize;
    

    /*
    uint16_t mean = 0;
    uint8_t flag = 0;

    for (int h=0; h < fb->height; h++){
         //Serial.println(h);
         for (int w=0; w < fb->width; w++){
             //Serial.println(w);
             int position = h*(fb->len/fb->height)+w;

             //Serial.println(position);
             mean = mean + fb->buf[position];
             flag++;

             if(flag > DOWNSAMPLE){
              newBuffer[position/DOWNSAMPLE] = mean/DOWNSAMPLE;
              flag = 0;
             }
         }
     }

    fb->width = fb->width / DOWNSAMPLE;
    fb->height = newHeight / DOWNSAMPLE;
    fb->len = fb->len / DOWNSAMPLE;

    fb->buf = newBuffer;*/
}



void drawLine(uint8_t ** out, size_t * out_len){


}

void resize(camera_fb_t *fb){
  unsigned int writeIndex = 0;
  int position = 0;
  int mean = 0;

  for (int h=0; h < fb->len; h = h + SCALEFACTOR){
    
    mean = fb->buf[h] + fb->buf[h + 1] +fb->buf[h + 2] +
           fb->buf[h + fb->width] + fb->buf[h + 1 + fb->width] +fb->buf[h + 2 + fb->width] +
           fb->buf[h + 2*fb->width] + fb->buf[h + 1 + 2*fb->width] +fb->buf[h + 2 + 2*fb->width];

    fb->buf[writeIndex++] = mean/9;

    if(h == (fb->width - SCALEFACTOR))
      h == fb->width * 3;
    }

  fb->width = fb->width / SCALEFACTOR;
  fb->height = fb->height / SCALEFACTOR;
  fb->len = (fb->width*fb->height);

}

void edgeDetector(){
  for (int h=0; h < imgHeight*imgWidth; h++){
      imgPtr[h] = conv(h, kernelX, 0);
      //yWay[h][w] = conv(fb, position, &kernelY);
  }
}

/*
void jpegInfo() {
  Serial.println(F("==============="));
  Serial.println(F("JPEG image info"));
  Serial.println(F("==============="));
  Serial.print(F(  "Width      :")); Serial.println(JpegDec.width);
  Serial.print(F(  "Height     :")); Serial.println(JpegDec.height);
  Serial.print(F(  "Components :")); Serial.println(JpegDec.comps);
  Serial.print(F(  "MCU / row  :")); Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F(  "MCU / col  :")); Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F(  "Scan type  :")); Serial.println(JpegDec.scanType);
  Serial.print(F(  "MCU width  :")); Serial.println(JpegDec.MCUWidth);
  Serial.print(F(  "MCU height :")); Serial.println(JpegDec.MCUHeight);
  Serial.println(F("==============="));
}
*/



#include <WiFi.h>
#include <JPEGDEC.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_http_server.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "OTA.h"
#include "camera.h"
#include "parameters.h"

//STREAM VARIABLES
#define PART_BOUNDARY "123456789000000000000987654321"

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
size_t _jpg_buf_len = 0;
uint8_t * _jpg_buf = NULL;

//IMAGE VARIABLES
uint8_t imgWidth = 60;
uint8_t imgHeight = 60;
uint8_t* imgPtr = (uint8_t*)malloc((imgWidth * imgHeight) * sizeof(uint8_t));

struct limitLines
{
     int INI;
     int FIN;
};

struct linePoint
{
     float x;
     float y;
};

int8_t kernelBlur [3][3] = {{1,1,1},
                            {1,1,1},
                            {1,1,1}};

int8_t kernelX [3][3] = {{-1,0,1},
                         {-2,0,2},
                         {-1,0,1}};

int8_t kernelY [3][3] = {{-1,-1,-1},
                         {0,0,0},
                         {1,1,1}};

JPEGDEC jpeg;

void print_free_memory()
{
    Serial.println("\n>>>---------- Memory Info ----------<<<");
    Serial.print("Free Heap: ");
    Serial.println(ESP.getFreeHeap());
    Serial.print("Free PSRAM: ");
    Serial.println(ESP.getFreePsram());
    Serial.print("Free sketch sapce: ");
    Serial.println(ESP.getFreeSketchSpace());
    Serial.print("Heap Size: ");
    Serial.println(ESP.getHeapSize());
    Serial.print("Max Alloc Heap: ");
    Serial.println(ESP.getMaxAllocHeap());
    Serial.print("Max Alloc Psram: ");
    Serial.println(ESP.getMaxAllocPsram());
    Serial.print("Psram Size: ");
    Serial.println(ESP.getPsramSize());
    Serial.println(">>>---------- Memory Info End ----------<<<\n");
}

void print_image_shape(camera_fb_t * frame){

     // print shape of image and total length (=heigth*width)
     Serial.print("Width: ");
     Serial.print(frame->width);
     Serial.print("\tHeigth: ");
     Serial.print(frame->height);
     Serial.print("\tLength: ");
     Serial.println(frame->len);
 }

void crop_image(camera_fb_t *fb, unsigned short cropLeft, unsigned short cropRight, unsigned short cropTop, unsigned short cropBottom)
{
    uint8_t newBuffer[(fb->width*fb->height)/SCALEFACTOR];
    unsigned int maxTopIndex = cropTop * fb->width;
    unsigned int minBottomIndex = ((fb->width*fb->height) - (cropBottom * fb->width));
    unsigned short maxX = fb->width - cropRight; // In pixels
    unsigned short newWidth = fb->width - cropLeft - cropRight;
    unsigned short newHeight = fb->height - cropTop - cropBottom;
    size_t newJpgSize = newWidth * newHeight;

    unsigned int writeIndex = 0;
    // Loop over all bytes
    
    for(int i = 0; i < fb->len; i+=2){
        // Calculate current X, Y pixel position
        int x = (i/2) % fb->width;

        // Crop from the top
        if(i < maxTopIndex){ continue; }

        // Crop from the bottom
        if(i > minBottomIndex){ continue; }

        // Crop from the left
        if(x <= cropLeft){ continue; }

        // Crop from the right
        if(x > maxX){ continue; }

        // If we get here, keep the pixels
        fb->buf[writeIndex++] = fb->buf[i];
        fb->buf[writeIndex++] = fb->buf[i+1];
    }
    
    
    fb->width = newWidth;
    fb->height = newHeight;
    fb->len = newJpgSize;
    

    /*
    uint16_t mean = 0;
    uint8_t flag = 0;

    for (int h=0; h < fb->height; h++){
         //Serial.println(h);
         for (int w=0; w < fb->width; w++){
             //Serial.println(w);
             int position = h*(fb->len/fb->height)+w;

             //Serial.println(position);
             mean = mean + fb->buf[position];
             flag++;

             if(flag > DOWNSAMPLE){
              newBuffer[position/DOWNSAMPLE] = mean/DOWNSAMPLE;
              flag = 0;
             }
         }
     }

    fb->width = fb->width / DOWNSAMPLE;
    fb->height = newHeight / DOWNSAMPLE;
    fb->len = fb->len / DOWNSAMPLE;

    fb->buf = newBuffer;*/
}

int conv(uint16_t pos, int8_t kernel[3][3], uint8_t mode){
  uint16_t topLeft = 0;
  uint16_t topCenter = 0;
  uint16_t topRight = 0;
  uint16_t middleLeft = 0;
  uint16_t middleRight = 0;
  uint16_t bottomLeft = 0;
  uint16_t bottomCenter = 0;
  uint16_t bottomRight = 0;
  int mean = 0;

  if(pos < imgWidth){
    topCenter = pos;
    topLeft = pos;
    topRight = pos;
  }else{
    topCenter = pos - imgWidth;
    topLeft = pos - imgWidth - 1;
    topRight = pos - imgWidth + 1;
  }

  if(imgWidth*(imgHeight-1) < pos && pos < imgWidth*imgHeight){
    bottomCenter = pos;
    bottomLeft = pos;
    bottomRight = pos;
  }else{
    bottomCenter = pos + imgWidth;
    bottomLeft = pos + imgWidth - 1;
    bottomRight = pos + imgWidth + 1;
  }

  if(pos % imgWidth == 0){
    middleLeft = pos;
    topLeft = pos;
    bottomLeft = pos;
  }
  else
    middleLeft = pos - 1;

  if(pos % (imgWidth - 1) == 0 && pos != 0){
    middleRight = pos;
    topRight = pos;
    bottomRight = pos;
  }
  else
    middleRight = pos + 1;
  /*
  Serial.println("---------");
  Serial.print(topLeft);
  Serial.print(" ");  
  Serial.print(topCenter);
  Serial.print(" ");  
  Serial.println(topRight);

  Serial.print(middleLeft);
  Serial.print(" ");  
  Serial.print(pos); 
  Serial.print(" "); 
  Serial.println(middleRight);

  Serial.print(bottomLeft);
  Serial.print(" ");  
  Serial.print(bottomCenter);
  Serial.print(" ");  
  Serial.println(bottomRight);

  delay(100);*/
  
   mean = kernel[2][2]*imgPtr[topLeft] + 
         kernel[2][1]*imgPtr[topCenter] + 
         kernel[2][0]*imgPtr[topRight] +
         kernel[1][2]*imgPtr[middleLeft] + 
         kernel[1][1]*imgPtr[pos] + 
         kernel[1][0]*imgPtr[middleRight] +
         kernel[0][2]*imgPtr[bottomLeft] + 
         kernel[0][1]*imgPtr[bottomCenter] + 
         kernel[0][0]*imgPtr[bottomRight];

  if(mode == 1)
    mean = mean/9;

  return mean;
}

void drawLine(uint8_t ** out, size_t * out_len){


}

void adaptativeThreshold(){

}

void toBinary(){
  unsigned int writeIndex = 0;
  int position = 0;

  for (int h=0; h < imgHeight; h++){
      for (int w=0; w < imgWidth; w++){
          position = h*(imgWidth)+w;
        if(imgPtr[position] > GRAYOFFSET)
          imgPtr[position] = 0;
        else
          imgPtr[position] = 255;

        imgPtr[writeIndex++] = imgPtr[position];
      }
    }

}

void findThreshold(){
  int16_t meanThreshold = 0;
  int8_t samples = 0;

  for (size_t i = 0; i < imgHeight; i++)
  {
    meanThreshold = meanThreshold + imgPtr[imgWidth/2 + i*imgWidth];
    samples++;
  }

  meanThreshold = meanThreshold/imgHeight;

  Serial.printf("threshold value: %d, for %d samples \n", meanThreshold, samples);
}

void resize(camera_fb_t *fb){
  unsigned int writeIndex = 0;
  int position = 0;
  int mean = 0;

  for (int h=0; h < fb->len; h = h + SCALEFACTOR){
    
    mean = fb->buf[h] + fb->buf[h + 1] +fb->buf[h + 2] +
           fb->buf[h + fb->width] + fb->buf[h + 1 + fb->width] +fb->buf[h + 2 + fb->width] +
           fb->buf[h + 2*fb->width] + fb->buf[h + 1 + 2*fb->width] +fb->buf[h + 2 + 2*fb->width];

    fb->buf[writeIndex++] = mean/9;

    if(h == (fb->width - SCALEFACTOR))
      h == fb->width * 3;
    }

  fb->width = fb->width / SCALEFACTOR;
  fb->height = fb->height / SCALEFACTOR;
  fb->len = (fb->width*fb->height);

}

void blur(){
  int mean = 0;
  int newLine = 0;

  /*
  for (uint16_t h = 0; h < imgHeight*imgWidth; h = h + STRIDEBLUR){ 
    imgPtr[h] = conv(h, kernelBlur, 1);
  }*/
  
  for (int h = imgWidth; h < imgWidth*imgHeight - 2*imgWidth; h = h + STRIDEBLUR){
    mean = imgPtr[h] + imgPtr[h + 1] + imgPtr[h + 2] +
           imgPtr[h + imgWidth] + imgPtr[h + 1 + imgWidth] +imgPtr[h + 2 + imgWidth] +
           imgPtr[h + 2*imgWidth] + imgPtr[h + 1 + 2*imgWidth] +imgPtr[h + 2 + 2*imgWidth];

    imgPtr[h + 1 + imgWidth] = mean/9;
    
    if(h == (imgWidth - STRIDEBLUR))
      newLine++;
      h == imgWidth * newLine;
    }
}

limitLines getHorizontalPoints(int pos){
  limitLines lines;
  lines.INI = 0;
  lines.FIN = 0;

  for (uint16_t i = 0; i < imgWidth; i++)
  {
    if(imgPtr[pos] == 0 && imgPtr[pos + 1] == 255){
      lines.INI = pos;
    }
    if(imgPtr[pos] == 255 && imgPtr[pos + 1] == 0)
      lines.FIN = pos;
    pos++;
  }

  return lines;
}

//function that returns, given an initial position, 
//a point (x,y) of the line in the x direction
//image referential starts on pixel (0,0)
//  (0,0) --------> x
//    |
//    |
//    ⬇
//    y
void getLinePoint(float *ix, float *iy, int pos){
  *iy = pos/imgWidth;
  *ix = 0;

  limitLines lines;
  lines.INI = 0;
  lines.FIN = 0;

  for (uint8_t i = 0; i < imgWidth; i++)
  {
    if(imgPtr[pos] == 0 && imgPtr[pos + 1] == 255){
      lines.INI = pos;
    }
    if(imgPtr[pos] == 255 && imgPtr[pos + 1] == 0)
      lines.FIN = pos;
    pos++;
  }

  if(lines.INI != 0 && lines.FIN != 0){
      *ix = (lines.INI + lines.FIN)/2;
  }
}

//function that receives the line limits than maps its direction
//direction is the slope of the real line of the image
//it is not converted to real world coordinates
void getLine(){
  int position;
  limitLines lines;
  lines.INI = 0;
  lines.FIN = 0;

  int meanINI = 0;
  int meanFIN = 0;
  uint8_t count = 0;
  uint8_t direction = 0;

  for (uint8_t i = 1; i < NLINES; i++)
  {
    position = imgWidth * (imgHeight/NLINES) * i;

    lines = getHorizontalPoints(position);

    if(lines.INI != 0 && lines.FIN != 0){
      meanINI = lines.INI + meanINI - position;
      meanFIN = lines.FIN + meanFIN - position;
      count++;
    }
    //Serial.printf("posINI: %d, posFIN: %d\n", lines.INI, lines.FIN);
  }

  meanINI = meanINI/count;
  meanFIN = meanFIN/count;
  direction = map(meanINI+meanFIN, 100, 200, 0, 68);

  Serial.printf("INI: %d, FIN: %d, direction: %d\n", meanINI, meanFIN, direction);
}

// convert from pixel coordinates to world coordinates in mm
//
// - pixel coordinates are as received from camera, with (0,0) being top left
//   , i.e., x == 0 is the first at the third pixel received from the camera
// - world coordinates have (0,0)mm in the middle of the front axle of the car,
//   with Y axis aligned with that axle, and X extending towards the front of
//   the car
void pixelToWorld(float px, float py, float *wx, float *wy)
{
	const float k = 0.005f;
	const float f = -0.606661978f;
	const float h = 99.31516461f;
	const float d = 22.181157951f;
	const float cos_teta = 0.64930051f;
	const float sin_teta = 0.7605319505f;

	// center coordinates
	//py -= 48.0;
	//px -= 48.0;

	float cpx = py * (k * cos_teta) - (f * sin_teta);
	float cpy = px * k;
	float cpz = py * (k * sin_teta) + (f * cos_teta);

	*wx = -(h / cpz) * cpx + d;
	*wy = -(h / cpz) * cpy;
}

//obtains image coordinates of the line
//converts to real world points
//gets the slope of that line
void getWorldLine(){
  int position;

  linePoint linePointWorld;
  linePoint linePointImage;

  uint8_t count = 0;
  float direction = 0;

  float meanX = 0;
  float meanY = 0;

  for (uint8_t i = 1; i < NLINES; i++)
  {
    position = imgWidth * (imgHeight/NLINES) * i;

    getLinePoint(&linePointImage.x, &linePointImage.y, position);

    if(linePointImage.x != 0){
      pixelToWorld(linePointImage.x, linePointImage.y, &linePointWorld.x, &linePointWorld.y);
      count++;
      //Serial.printf("image X: %f, image Y: %f - world X: %f, world Y: %f\n",linePointImage.x, linePointImage.y, linePointWorld.x, linePointWorld.y);
      meanX = meanX + linePointWorld.x;
      meanY = meanY + linePointWorld.y;
    }
  }

  direction = meanX/meanY;

  Serial.printf("meanX: %f, meanY: %f, direction: %f, angle: %f\n", meanX, meanY, direction, (atan(meanX/meanY)* 180 / PI));
}


void edgeDetector(){
  for (int h=0; h < imgHeight*imgWidth; h++){
      imgPtr[h] = conv(h, kernelX, 0);
      //yWay[h][w] = conv(fb, position, &kernelY);
  }
}

void processImage(){
  //findThreshold();
  //blur();
  toBinary();
  getWorldLine();
  //getLine();
}

int fillArray(JPEGDRAW *pDraw)
{
  uint8_t width = 0;
  uint16_t pos = 0;

  //Serial.printf("Draw pos = %d,%d. size = %d x %d, bpp = %d, width used = %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight, pDraw->iBpp, pDraw->iWidthUsed);
  
  width = pDraw->iWidth;
  pos = pDraw->y * width;
  
    for (int i = 0; i < width ; i++)
    {
      imgPtr[pos] = pDraw->pPixels[i] & 0xff;
      imgPtr[pos + 1] = (pDraw->pPixels[i] >> 8) & 0xff;

      pos = pos + 2;
    }
  
  return 1;
}

void decodeImage(camera_fb_t *fb){

  if (jpeg.openFLASH((uint8_t *)fb->buf, fb->len, fillArray))
  {
    //Serial.printf("Image size: %d x %d, orientation: %d, bpp: %d\n", jpeg.getWidth(),jpeg.getHeight(), jpeg.getOrientation(), jpeg.getBpp());

    jpeg.setPixelType(EIGHT_BIT_GRAYSCALE);

    if (jpeg.decode(0,0,JPEG_SCALE_QUARTER))
    {
      //Serial.println("Error decoding JPEG");
    }
    jpeg.close();
  }
}

/*
void jpegInfo() {
  Serial.println(F("==============="));
  Serial.println(F("JPEG image info"));
  Serial.println(F("==============="));
  Serial.print(F(  "Width      :")); Serial.println(JpegDec.width);
  Serial.print(F(  "Height     :")); Serial.println(JpegDec.height);
  Serial.print(F(  "Components :")); Serial.println(JpegDec.comps);
  Serial.print(F(  "MCU / row  :")); Serial.println(JpegDec.MCUSPerRow);
  Serial.print(F(  "MCU / col  :")); Serial.println(JpegDec.MCUSPerCol);
  Serial.print(F(  "Scan type  :")); Serial.println(JpegDec.scanType);
  Serial.print(F(  "MCU width  :")); Serial.println(JpegDec.MCUWidth);
  Serial.print(F(  "MCU height :")); Serial.println(JpegDec.MCUHeight);
  Serial.println(F("==============="));
}
*/

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  
  char * part_buf[64];
  static int64_t last_frame = 0;
  
  if(!last_frame) {
      last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){

    fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {

      decodeImage(fb);
      processImage();

      bool jpeg_converted = fmt2jpg((uint8_t *)imgPtr, imgWidth*imgHeight, imgWidth, imgHeight, PIXFORMAT_GRAYSCALE, 80, &_jpg_buf, &_jpg_buf_len);

      esp_camera_fb_return(fb);
      fb = NULL;

      if(!jpeg_converted){
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
      }
    }
    
    if(!RACEMODE){
      if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      }
      if(res == ESP_OK){
        res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
      }
      if(res == ESP_OK){
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      }
    }

    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if(res != ESP_OK){
      break;
    }

    
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;

    //Serial.printf("MJPG: %uKB %ums (%.1ffps)",(uint32_t)(_jpg_buf_len/1024),(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    //Serial.println("");
  }

  last_frame = 0;
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}

void captureImage(void *arg) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  
  static int64_t last_frame = 0;

  while(true){
    fb = esp_camera_fb_get();

    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } 
    else {
      decodeImage(fb);
      processImage();

      esp_camera_fb_return(fb);
      fb = NULL;
    }

    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;

    //Serial.printf("MJPG: %uKB %ums (%.1ffps)",(uint32_t)(imgHeight*imgWidth/1024),(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
    //Serial.println("");
  }
  last_frame = 0;
}

void captureImageTask(){
  xTaskCreate(
  captureImage, 
  "Demo_Task",
  8192, 
  NULL, 
  1, 
  NULL);
}

void setupWifi(){

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");

  TelnetStream.begin();

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
 }

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  
  setupWifi();

  //setupOTA("shelbyRR", ssid, password);

  startCamera();

  if (!RACEMODE)
    startCameraServer();
  else
    captureImageTask();
  
}

void loop() {
  //ArduinoOTA.handle();
  delay(1);
}

