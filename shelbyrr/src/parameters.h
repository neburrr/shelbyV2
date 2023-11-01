//IMAGE PROCESSING
#define GRAYOFFSET 30 
#define SCALEFACTOR 2
#define STRIDEBLUR 1
#define FACTORBLUR 1/9
#define NLINES 10 //does -2 times
#define BINARYOFFSETPERCENTAGE 30
#define BINARYWINDOW 12

//LDR THRESHOLD
#define LDRPIN 15
#define LDRTHRESHOLD 2500

//HALL VARIABLES
#define HALLPIN GPIO_NUM_14
#define HALLK 79.36 //46.94 //46.875 //mm 64 impulses is equivalent to 3000mm

//PID VARIABLES
#define PIDSETPOINTTETA 86.12
#define PIDSETPOINTY -5.22
#define KP 0.8
#define KI 0.05
#define KD 0.5
#define SAMPLINGTIME 0.0195  //19,5ms for 20000000Hz; 24,5ms for 16000000
#define PIDMAX 5
#define PIDMIN -5

//RACE VARIABLES
#define RACESPEEDMAX 1024
#define BRAKESPEED 100
#define BRAKEDISTANCE 15000
#define RACEMODE 0

