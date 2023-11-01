
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

#include "credentials.h"
#include "OTA.h"
#include "camera.h"
#include "parameters.h"
#include "pid.h"
#include "motor.h"
#include "semaphore.h"

//STREAM VARIABLES
#define PART_BOUNDARY "123456789000000000000987654321"

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

TaskHandle_t cameraTaskHandle = NULL;
TaskHandle_t raceTaskHandle = NULL;

httpd_handle_t stream_httpd = NULL;
size_t _jpg_buf_len = 0;
uint8_t * _jpg_buf = NULL;

//IMAGE VARIABLES
uint8_t imgWidth = 60;
uint8_t imgHeight = 60;
uint8_t* imgPtr = (uint8_t*)malloc((imgWidth * imgHeight) * sizeof(uint8_t));
int* imgInt = (int*)malloc((imgWidth * imgHeight) * sizeof(int));


int8_t kernelBlur [3][3] = {{1,1,1},
                            {1,1,1},
                            {1,1,1}};

float lineOrientation = 0;
float lineDisplacement = 0;

float kTeta = 1;
float kY = 0;

bool endRace = false;

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

int hallCounter = 0;

JPEGDEC jpeg;

PID pid = PID(SAMPLINGTIME, PIDMAX, PIDMIN, KP, KD, KI);

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

void adaptativeThreshold(){
  int position = 0;
  int sum = 0;
  int count = 0;
  int x1, x2, y1, y2 = 0;
  int A, B, C, D = 0;

  int width, height = 0;

  for (int h=0; h < imgHeight; h++){
    sum = 0;
    for (int w=0; w < imgWidth; w++){
      position = h * imgWidth + w;
      
      sum = sum + imgPtr[position];

      if(position < imgWidth)
        imgInt[position] = sum;
      else
        imgInt[position] = imgInt[position - imgWidth] + sum;
    }
  }

  for (int h=BINARYWINDOW/2; h < imgHeight - BINARYWINDOW/2; h++){
    for (int w=BINARYWINDOW/2; w < imgWidth - BINARYWINDOW/2; w++){
      position = h * imgWidth + w;

      /*
      if(h <= (BINARYWINDOW/2))
        height = BINARYWINDOW/2;
      else if (h >= (imgHeight - (BINARYWINDOW/2)))
        height = imgHeight - (BINARYWINDOW/2);
      else
        height = h;
        
      if(w <= (BINARYWINDOW/2))
        width = BINARYWINDOW/2;
      else if (w >= (imgWidth - (BINARYWINDOW/2)))
        width = imgWidth - BINARYWINDOW/2;
      else
        width = w;
      
      x1 = (height - BINARYWINDOW/2) * imgWidth;
      x2 = (height + BINARYWINDOW/2) * imgWidth;
      y1 = width - BINARYWINDOW/2; 
      y2 = width + BINARYWINDOW/2;*/
      
      //count = (x2 - x1) * (y2 - y1);

      //Serial.printf("h: %d, w: %d\n", height, width);
      //Serial.printf("x1: %d, x2: %d, y1: %d, y2: %d\n", x1, x2, y1, y2);
      //Serial.printf("x2y2: %d, x2y1: %d, x1y2: %d, y1x1: %d\n", x2 + y2, x2 + y1, x1 + y2, x1 - 1+ y1 - 1);
      //Serial.printf("intImg x2y2: %d, x2y1: %d, x1y2: %d, y1x1: %d\n", imgInt[x2 + y2], imgInt[x2 + y1 - 1], imgInt[x1 - 1 + y2], imgInt[x1 - 1 + y1 - 1]);
      
      A = position - (BINARYWINDOW/2)*imgWidth - BINARYWINDOW/2; //X1Y1
      B = position - (BINARYWINDOW/2)*imgWidth + BINARYWINDOW/2; //X1Y2
      C = position + (BINARYWINDOW/2)*imgWidth - BINARYWINDOW/2; //X2Y1
      D = position + (BINARYWINDOW/2)*imgWidth + BINARYWINDOW/2; //X2Y2

      //border checking
      if(h <= (BINARYWINDOW/2)){
        A = position - BINARYWINDOW/2;
        B = position + BINARYWINDOW/2;
      }
      else if (h >= (imgHeight - (BINARYWINDOW/2))){
        C = position - BINARYWINDOW/2;
        D = position + BINARYWINDOW/2;
      }
      if(w <= (BINARYWINDOW/2)){
        A = position - (BINARYWINDOW/2)*imgWidth;
        C = position + (BINARYWINDOW/2)*imgWidth;
      }
      else if (w >= (imgWidth - (BINARYWINDOW/2))){
        B = position - (BINARYWINDOW/2)*imgWidth; 
        D = position + (BINARYWINDOW/2)*imgWidth; 
      }

      if((h <= (BINARYWINDOW/2)) && (w <= (BINARYWINDOW/2)))
        A = position;
      else if ((h <= (BINARYWINDOW/2)) && (w >= (imgWidth - (BINARYWINDOW/2))))
        B = position;
      else if((h >= (imgHeight - (BINARYWINDOW/2))) && (w <= (BINARYWINDOW/2)))
        C = position;
      else if ((h >= (imgHeight - (BINARYWINDOW/2))) && (w >= (imgWidth - (BINARYWINDOW/2))))
        D = position;
      
        //Serial.printf("h: %d, w: %d\n", h, w);
        //Serial.printf("position: %d, A: %d, B: %d, C: %d, D: %d\n", position, A, B, C, D);
        //Serial.printf("intImg x2y2: %d, x2y1: %d, x1y2: %d, y1x1: %d\n", imgInt[D], imgInt[C], imgInt[B], imgInt[A]);
        //Serial.printf("B-A: %d, D-C: %d\n", B-A, (C/imgWidth) - (A / imgWidth));
        //Serial.printf("soma: %d, count: %d, imgPtr: %d\n", sum, count, imgPtr[position]);

      count = (B - A) * ((C / imgWidth) - (A / imgWidth));
      
      //(soma = imgInt[x2 + y2] - imgInt[x2 + y1 - 1] - imgInt[x1 - 1 + y2] + imgInt[x1 - 1 + y1 - 1];
      sum = imgInt[D] - imgInt[C] - imgInt[B] + imgInt[A];

      //Serial.printf("count: %d, soma: %d, pixel: %d, offset: %d\n", count, soma, (imgPtr[position] * count), (soma * (100 - BINARYOFFSETPERCENTAGE) / 100));

      if((imgPtr[position] * count) <= (sum * (100 - BINARYOFFSETPERCENTAGE) / 100))
        imgPtr[position] = 255;
      else
        imgPtr[position] = 0;
    }
  }
}

int getThreshold(int pos){
  int A, B, C, D = 0;

  A = imgInt[pos - 1 - imgWidth];
  B = imgInt[pos + 1 - imgWidth];
  C = imgInt[pos - 1 + imgWidth];
  D = imgInt[pos + 1 + imgWidth];

  int mean = (A + B + C + D) - (A + B) - (A + C) + A;

  mean /= 9;

  return mean * BINARYOFFSETPERCENTAGE;
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

void blur(){
  int mean = 0;
  int newLine = 0;

  
  for (uint16_t h = 0; h < imgHeight*imgWidth; h = h + STRIDEBLUR){ 
    imgPtr[h] = conv(h, kernelBlur, 1);
  }
  
  /*
  for (int h = imgWidth; h < imgWidth*imgHeight - 2*imgWidth; h = h + STRIDEBLUR){
    mean = imgPtr[h] + imgPtr[h + 1] + imgPtr[h + 2] +
           imgPtr[h + imgWidth] + imgPtr[h + 1 + imgWidth] +imgPtr[h + 2 + imgWidth] +
           imgPtr[h + 2*imgWidth] + imgPtr[h + 1 + 2*imgWidth] +imgPtr[h + 2 + 2*imgWidth];

    imgPtr[h + 1 + imgWidth] = mean/9;
    
    if(h == (imgWidth - STRIDEBLUR))
      newLine++;
      h == imgWidth * newLine;
    }*/
}

int8_t count = 0;

void imageGrid(){
  int position = 0;
  
  if(count < 10){
    for (int h=0; h < imgHeight; h++){
      for (int w=0; w < imgWidth; w++){
        position = h*(imgWidth)+w;
        if(h%2 == 0)
          imgPtr[position] = 255;
        if(w%2==0)
          imgPtr[position] = 255;
      }
    }
  }
  count++;
  if(count == 20)
  count = 0;
}

// position for servo as received
// puts the servos with toe out for stability
// each servo has its own linear equation
void servoPositioning(float kTeta, float kY, float lineOrientation, float lineDisplacement){

  double pidValue = pid.calculate(kTeta * PIDSETPOINTTETA + kY * PIDSETPOINTY, kTeta*lineOrientation + kY*lineDisplacement);

  int8_t servoPos = pidValue;

  //Serial.printf("lineOrientation: %f, lineDisplacement: %f, lineOrientationkTeta: %f, lineDisplacementkY: %f, pid value: %f, servoPos: %d\n",lineOrientation, lineDisplacement, kTeta * lineOrientation, kY* lineDisplacement, pidValue, servoPos);
  //vTaskDelay(10);
  //Serial.printf("ServoRight: %d, ServoLeft: %d\n", servoPos+SERVORZERO, servoPos+SERVOLZERO);

  servoR.write(servoPos + SERVORZERO);
  servoL.write(servoPos + SERVOLZERO);
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

//  function that returns, given an initial position, 
//  a point (x,y) of the line in the x direction
//  image referential starts on pixel (0,0)
//  (0,0) --------> x
//    |
//    |
//    ⬇
//    y
void getLinePoint(float *ix, float *iy, int pos){
  *iy = pos/imgWidth;
  *ix = 0;
  int posInit = 0;
  int countHorizontalLine = 0;
  bool endline = true;

  limitLines lines;
  lines.INI = 0;
  lines.FIN = 0;

  posInit = pos;
  
  imgPtr[posInit] =255;
  for (uint8_t i = 0; i < imgWidth; i++){
    if(imgPtr[pos] == 0 && imgPtr[pos + 1] == 255){
      lines.INI = pos - posInit;
    }
    if(imgPtr[pos] == 255 && imgPtr[pos + 1] == 0)
      lines.FIN = pos - posInit;
    pos++;
  }

  if(lines.INI != 0 && lines.FIN != 0){
      *ix = (lines.INI + lines.FIN)/2;
  }
}

//  function that receives the line limits than maps its direction
//  direction is the slope of the real line of the image
//  it is not converted to real world coordinates
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
void pixelToWorld(float px, float py, float *wx, float *wy){
	const float k = -0.621702080f;
	const float f = 33.925879062f;
	const float h = -71.698063818f;
	const float d = -60.271957509f;
	const float cos_teta = -0.605555555f;
	const float sin_teta = 0.795803035f;

	// center coordinates
	py -= 32.0;
	px -= 32.0;

	float cpx = py * (k * cos_teta) - (f * sin_teta);
	float cpy = px * k;
	float cpz = py * (k * sin_teta) + (f * cos_teta);

	*wx = -(h / cpz) * cpx + d;
	*wy = -(h / cpz) * cpy;
}

//  obtains image coordinates of the line
//  converts to real world points
//  gets the slope of that line
void getWorldLine(){
  int position;

  linePoint linePointWorld;
  linePoint linePointImage;
  
  float lineOrientationMean = 0;
  float cnt = 0;

  float distance = 0;
  float direction = 0;

  float meanX = 0;
  float meanY = 0;

  for (uint8_t i = 1; i < NLINES - 1; i++)
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
  
  direction = meanY/meanX + PI/2;

  lineOrientation = atan2(meanY,meanX) * 180 / PI + 180/2;
  lineDisplacement = meanY / (NLINES - 1);

  distance = hallCounter * HALLK;

  if(distance < BRAKEDISTANCE){
    kY = distance / 10000;
    kTeta = 1 - kY;
  }
  else if (distance < 2000)
  {
    kY = 0.1;
    kTeta = 0.9;
  }
  else{
    count += 0.05;
    kTeta = 1 - (BRAKEDISTANCE / 10000) + cnt;

    if(kTeta > 1)
      kTeta = 1;

    kY = 1 - kTeta;
  }

  
  
  //line orientation + line displacement
  servoPositioning(kTeta, kY, lineOrientation, lineDisplacement);

  /*
  lineOrientationMean = lineOrientationMean + lineOrientation;
  cnt++;
  if(cnt == 5) {
    servoPositioning(lineOrientationMean/5);
    lineOrientationMean = 0;
    cnt = 0;
  }*/
  
  
  //Serial.printf("meanX: %f, meanY: %f, direction: %f, angle: %f\n", meanX, meanY, direction, lineOrientation);
}

//get end line vertically
void getEndLine(){
  lockVariable();
  int pos = 0;
  int offset = 0;
  int cnt = 0;

  for (uint8_t i = BINARYWINDOW/2 + 10; i < imgWidth - BINARYWINDOW/2; i++){
    offset = i;
    for (uint8_t i = BINARYWINDOW/2; i < imgHeight; i++)
    {
      pos = offset + i*imgWidth;
      //Serial.printf("pos: %d\n", pos);
      if(imgPtr[pos] == 0 && imgPtr[pos + imgWidth] == 255){
        cnt++;
      }
    }
  }

  //Serial.printf("count: %d\n", cnt);

  if(cnt > 40){
    endRace = true;
  }    
  unlockVariable();
}

void getEndLine2(){
  lockVariable();
  bool endlineRight = false;
  bool endlineLeft = false;
  int pos = 0;
  int posWidth = 0;
  int count = 0;

  for (uint8_t h = BINARYWINDOW/2 + 1; h < 30; h++)
  {
    pos = h*imgWidth;
    //Serial.printf("height: %d\n", h);
    for (uint8_t w = BINARYWINDOW/2 + 1; w < 20; w++)
    {
      posWidth = pos + w;
      if (imgPtr[posWidth] == 255 && imgPtr[posWidth + 1] == 255 && imgPtr[posWidth + 2] == 255)
        endlineLeft = true;
    }
  }
  for (uint8_t h = BINARYWINDOW/2 + 1; h < 30; h++)
  {
    pos = h*imgWidth;
    for (uint8_t w = 33; w < imgWidth - BINARYWINDOW/2 - 1; w++)
    {
      posWidth = pos + w;
      if (imgPtr[posWidth] == 255 && imgPtr[posWidth + 1] == 255 && imgPtr[posWidth + 2] == 255)
        endlineRight = true;
    }
  }
  
    //if(endlineLeft || endlineRight)
     //break;
  
  endRace = endlineLeft && endlineRight;
  unlockVariable();  
}

//get end line horizontally
void getEndLine3(){
  lockVariable();
  bool endline;
  int pos = 0;
  int count = 0;

  for (uint8_t h = BINARYWINDOW/2; h < 25; h++)
  {
    count = 0;
    //Serial.printf("height: %d\n", h);
    for (uint8_t w = BINARYWINDOW/2; w < 25; w++)
    {
      pos++;
      if (imgPtr[pos] == 255 && imgPtr[pos + 1] == 255)
        count++;
      else
        count--;
    }
    //TelnetStream.printf("count: %d\n", count);
    //TelnetStream.println("");

    /*
    if(count > -40){
      endRace = true;
      break;
    }*/
  }
  unlockVariable();  
}

void processImage(){
  blur();
  adaptativeThreshold();
  //toBinary();
  getWorldLine();
  getEndLine();
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
      //processImage();

      bool jpeg_converted = fmt2jpg((uint8_t *)imgPtr, imgWidth*imgHeight, imgWidth, imgHeight, PIXFORMAT_GRAYSCALE, 200, &_jpg_buf, &_jpg_buf_len);
      
      //_jpg_buf_len = fb->len;
      //_jpg_buf = fb->buf;

      esp_camera_fb_return(fb);
      fb = NULL;
    }
    
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

    Serial.printf("MJPG: %uKB %ums (%.1ffps)\n",(uint32_t)(_jpg_buf_len/1024),(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
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
    //Serial.printf("MJPG: %uKB %ums (%.1ffps)\n",(uint32_t)(imgHeight*imgWidth/1024),(uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
  }
  last_frame = 0;
}

void captureImageTask(){
  xTaskCreate(
  captureImage, 
  "Image Task",
  20000, 
  NULL, 
  1, 
  &cameraTaskHandle);
}

void setupMotors(){
  // Servo config
  servoR.attach(servoRight); 
  servoL.attach(servoLeft); 
  servoR.write(SERVORZERO);
  servoL.write(SERVOLZERO);

  // Motor Config
  //pinMode(R_ENPIN, OUTPUT);
  
  //digitalWrite(L_ENPIN, LOW);
  ledcSetup(PWMChannel, PWMFrequency, PWMResolution); //0 to 1024
  ledcAttachPin(PWMMotorPIN, PWMChannel);
}

// reads ldr output
// returns true if the condition  is triggered 
bool ldrStart(){
  int64_t timeIni = esp_timer_get_time();

  pinMode(LDRPIN, OUTPUT);
  digitalWrite(LDRPIN, LOW);
  delay(1);

  pinMode(LDRPIN, INPUT);
  while(!digitalRead(LDRPIN)){}

  int64_t timeFin = esp_timer_get_time(); 
  int64_t timeLDR = timeFin - timeIni;
  //Serial.printf("time: %uus\n",(uint32_t)timeLDR);

  if(timeLDR < LDRTHRESHOLD)
    return true;
  else
    return false;
}

// launches robot for less slip
void launch(){
  
  for (size_t i = 100; i < RACESPEEDMAX; i++)
  {
    ledcWrite(PWMChannel, i);
    vTaskDelay(1);
  }
  //ledcWrite(PWMChannel, RACESPEEDMAX);
}

// controls motor pwm during race
void racing(){
  ledcWrite(PWMChannel, RACESPEEDMAX);
}

// brakes robot by connecting h-bridge to ground
void brake(){
  for (size_t i = RACESPEEDMAX; i > BRAKESPEED; i = i - 1)
  {
    ledcWrite(PWMChannel, i);
    //vTaskDelay(1);
  }
  //ledcWrite(PWMChannel, BRAKESPEED);
}

// brakes robot by connecting h-bridge to ground
void stop(){
  ledcWrite(PWMChannel, 0);
}

// function that handles all the moving processes
void race(void *arg){
  float distance = 0;

  while (!ldrStart()){}
  int64_t timeIni = esp_timer_get_time();

  hallCounter = 0;

  //digitalWrite(R_ENPIN, HIGH);
  launch();
  
  while (distance < BRAKEDISTANCE){
    racing();
    distance = hallCounter*HALLK;
    vTaskDelay(10);
    //Serial.printf("distance: %f\n", distance);
  } 
  
  brake();

  while(true){
    lockVariable();
    distance = hallCounter * HALLK;
    //Serial.println(endRace);
    if(endRace)
      break;
    unlockVariable();
    vTaskDelay(100);
  }
  unlockVariable();
  
  stop();
  
  int64_t timeFin = esp_timer_get_time(); 
  int64_t timeLDR = timeFin - timeIni;
  //Serial.printf("racetime: %ums\n",(uint32_t)timeLDR/1000);

  while(true){
    TelnetStream.printf("racetime: %ums\n",(uint32_t)timeLDR/1000);
    TelnetStream.println("");
    vTaskDelay(500);
  }
  //servoL.write(2);
  //Serial.println(xPortGetCoreID());
}

void raceNoTask(){
  int64_t timeIni = esp_timer_get_time();

  while (!ldrStart()){}

  digitalWrite(R_ENPIN, HIGH);
  launch();

  while (!endRace){
    racing();
  }

  brake();
  digitalWrite(R_ENPIN, LOW);

  int64_t timeFin = esp_timer_get_time(); 
  int64_t timeLDR = timeFin - timeIni;
  //TelnetStream.printf("racetime: %uus\n",(uint32_t)timeLDR);

  //servoL.write(2);
  //Serial.println(xPortGetCoreID());
}

void raceTask(){
  xTaskCreate(
    race,      // Função a ser chamada
    "Race",    // Nome da tarefa
    4096,               // Tamanho (bytes)
    NULL,               // Parametro a ser passado
    0,                  // Prioridade da tarefa
    &raceTaskHandle               // Task handle
  );
}

void hallSensorCounter(void *arg){
  hallCounter++;
}

void setupHallSensor(){
  //pinMode(HALLPIN, INPUT_PULLUP);
  //attachInterrupt(HALLPIN, hallSensorCounter, CHANGE);

  gpio_pad_select_gpio(HALLPIN);
  gpio_set_direction(HALLPIN, GPIO_MODE_INPUT);
  gpio_pulldown_dis(HALLPIN);
  gpio_pullup_en(HALLPIN);
  gpio_set_intr_type(HALLPIN, GPIO_INTR_NEGEDGE);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(HALLPIN, hallSensorCounter, (void *)HALLPIN);
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

  Serial.print("MAC Address:  ");
  Serial.println(WiFi.macAddress());
 }

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);

  if (!RACEMODE)
    setupWifi();

  createSemaphore();

  //setupOTA("shelbyRR", ssid, password);

  startCamera();

  setupMotors();

  setupHallSensor();

  if (!RACEMODE)
    startCameraServer();
  else
    captureImageTask();

  raceTask();
}

void loop() {
  //ArduinoOTA.handle();
  vTaskDelay(5 / portTICK_PERIOD_MS);
  //Serial.printf("distance: %f\n", hallCounter * HALLK);
  //Serial.printf("hallcounter: %d\n", hallCounter);
  //Serial.println("");
}

