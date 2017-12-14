//Libraries used are the Arduino PID library, ESP8266 built in libraries, aws-sdk-esp8266, DHT, TimeLib, and RCSwitch libraries
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include <string>

#include <mosquittopp.h>

#include <PID_v1.h>

#include <aws/core/Aws.h>
#include <aws/core/utils/Outcome.h>
#include <aws/dynamodb/DynamoDBClient.h>
#include <aws/dynamodb/model/AttributeDefinition.h>
#include <aws/dynamodb/model/PutItemRequest.h>
#include <aws/dynamodb/model/PutItemResult.h>

// #include <Aws.h>
// #include </utils/Outcome.h> 
// #include <aws/dynamodb/DynamoDBClient.h>
// #include <aws/dynamodb/model/AttributeDefinition.h>
// #include <aws/dynamodb/model/GetItemRequest.h>


// #include <PID_v1.h>
// #include <ESP8266WiFi.h>
// #include <WiFiUdp.h>
// #include "Esp8266AWSImplementations.h"
// #include "AmazonDynamoDBClient.h"
// #include "AWSFoundationalTypes.h"
// #include "keys.h"
// #include <DHT.h>
// #include <Wire.h>
// #include <RCSwitch.h>
// #include <TimeLib.h>
//// Not using a LCD screen anymore - deprecated
// #include <LiquidCrystal_I2C.h>


// RTC data fields - used for data restore on device reset. This is largely taken from the RTC example in the ESP8266 library.
// CRC function used to ensure data validity
uint32_t calculateCRC32(const uint8_t *data, size_t length);

// helper function to dump memory contents as hex
void printMemory();

// Structure which will be stored in RTC memory.
// First field is CRC32, which is calculated based on the
// rest of structure contents.
// Any fields can go after CRC32.
// We use byte array as an example.
struct {
  uint32_t crc32;
  double t5;
  double t90;
  double u5;
  double u90;
  double p5;
  double p90;
  double h5;
  double h90;
  int c5;
  int c90;
  unsigned long initial;
  unsigned long last90;
  unsigned long last1d;
} rtcData;

//Initializing transistor pins
int transistorPin = 14;
int incomingByte;  // a variable to read incoming serial data into
//Setpoint hardcoded, will switch to a GetItem from DynamoDB in the future
double setPoint = 69.0;
//Intializing the PID output, to avoid errors on the first PID run
double pidOutput = 0;
//Setting to store transistor state
int transistorGate;

//This section of doubles and double vectors stores the current and running data for the data that will be outputted to DynamoDB 

std::vector <double> temp_f_20s;
std::vector <double> humidity_20s;
std::vector <double> pid_20s;
std::vector <double> heater_20s;

std::vector <double> temp_f_5m;
std::vector <double> humidity_5m;
std::vector <double> pid_5m;
std::vector <double> heater_5m;

std::vector <double> temp_f_90m;
std::vector <double> humidity_90m;
std::vector <double> pid_90m;
std::vector <double> heater_90m;

double temp_f_inst_1d, humidity_inst_1d, pid_inst_1d, heater_inst_1d = 0;

double heater_now;

int counter_20s = 0;
int counter_5m = 0;
int counter_90m = 0;

WiFiUDP udp;

//Variables to keep track of current and past upload time (format is seconds since last unix epoch)
unsigned long epoch, epoch_initial, epoch_now, epoch_last_5m, epoch_last_90m, epoch_last_1d;

//Currently not in use, this is a setting that should trip and stop average uploads if too many lower level time periods have passed (i.e. if the 5min average has taken 100 measurements of 20sec apiece)
bool flagDontPushAvgTemp = false; 

// PID tuning variables 
double KP = 25;
double KI = 0.028;
double KD = 0;   // Not yet used
unsigned long windowSize = 1800; // 30 minutes (ish)

//Variable to keep track of PID window
unsigned long windowStartTime = 0;

//Variable to keep track of heater on or off state
boolean stateVariable = false;

//AWS constants
const char* AWS_REGION = "us-west-1";
const char* AWS_ENDPOINT = "amazonaws.com";

// Constants describing DynamoDB table and values being used
const char* TABLE_20S = "Temperatures_20_sec";
const char* TABLE_5M = "Temperatures_5_min";
const char* TABLE_90M = "Temperatures_90_min";
const char* TABLE_1D = "Temperatures_1_day";

const char* TABLE_STATE = "HeaterManualState";
const char* HASH_KEY_NAME2 = "Id";
const char* HASH_KEY_VALUE2 = "Website";
const char* RANGE_KEY_VALUE2 = "1";
const char* RANGE_KEY_NAME2 = "Date";
static const int KEY_SIZE = 2;

//Variable to internally track the heater state of the website
static int HeaterControl;

//DynamoDB objects
Esp8266HttpClient httpClient;
//I suspect the ESP8266 AWS DateTimeProvider is causing some of the device crashes, so I'm rolling my own
Esp8266DateTimeProvider dateTimeProvider;
GetItemInput getItemInput;
PutItemInput putItemInput;
AttributeValue hashKey;
AttributeValue rangeKey;
ActionError actionError;

AmazonDynamoDBClient ddbClient;

//LCD screen no longer used
//LiquidCrystal_I2C  lcd(0x3F, 4, 5);

RCSwitch mySwitch = RCSwitch();

//Temperature sensor variables
DHT dht(5, DHT22, 20);
double humidity, temp_f = 0;

//PID variable initial setting
PID myPID(&temp_f, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

//Loop and AWS offset time variables 
unsigned long lastTimeUpdate, lastTempOutputUpdate, lastHeaterRefresh = 0;

//Setup section
void setup()
{
  Serial.begin(115200);
  Serial.println("Serial initialized...");

  // Set pins for transistor and onboard LED blink
  pinMode(transistorPin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  //Initialize Wifi - ssid and password manually set in and then taken from keys.h in the AWS SDK library
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    yield();
    Serial.print(".");
  }
  Serial.println("");

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //Reset the WiFi if the NTP server can't be looked up (i.e. if DNS is unfunctional)
  if(!WiFi.hostByName(ntpServerName, timeServerIP)) { // Get the IP address of the NTP server
    Serial.println("DNS lookup failed. Rebooting.");
    Serial.flush();
    ESP.reset();
  }

  //Initialize UDP
  Serial.println("Starting UDP");
  udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(udp.localPort());
  
  //Initialize RFID switch
  mySwitch.enableTransmit(2);
  mySwitch.setPulseLength(184);

  //Initialize DHT sensor
  dht.begin();

  //Initialize LCD screen on unit (deprecated) 
  //  lcd.init();
  //  lcd.backlight();
  
  //Flash builtin to alert startup
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
  digitalWrite(LED_BUILTIN, HIGH);
  
  //Initialize PID limits
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);

  //Take first temp reading and compute PID for initial output
  temp_f = dht.readTemperature(true);
  if (!isnan(temp_f)) {
    yield();
    myPID.Compute();
  }

  Serial.print("Initial PID output: ");
  Serial.println(pidOutput);

  //Initialize DynamoDB client
  ddbClient.setAWSRegion(AWS_REGION);
  ddbClient.setAWSEndpoint(AWS_ENDPOINT);
  ddbClient.setAWSSecretKey(awsSecKey);
  ddbClient.setAWSKeyID(awsKeyID);
  ddbClient.setHttpClient(&httpClient);
  ddbClient.setDateTimeProvider(&dateTimeProvider);

  //Attempt to read the RTC memory to see if there are any long-time upload data remaining from before the device reset, and if read is valid then set the available data
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("RTC Read: ");
    printMemory();
    Serial.println();
    uint32_t crcOfData = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
    Serial.print("CRC32 of data: ");
    Serial.println(crcOfData, HEX);
    Serial.print("CRC32 read from RTC: ");
    Serial.println(rtcData.crc32, HEX);
    if (crcOfData != rtcData.crc32) {
      Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Data is probably invalid!");
    }
    else {
      Serial.println("CRC32 check ok, data is probably valid.");

  temp_f_sum_5m = rtcData.t5;    
  temp_f_sum_90m = rtcData.t90;
      
  humidity_sum_5m = rtcData.u5;    
  humidity_sum_90m = rtcData.u90;
      
  pid_sum_5m = rtcData.p5;    
  pid_sum_90m = rtcData.p90;
      
  heater_sum_5m = rtcData.h5;    
  heater_sum_90m = rtcData.h90;

  counter_5m = rtcData.c5;
  counter_90m = rtcData.c90;

  epoch_initial = rtcData.initial;
  epoch_last_90m = rtcData.last90;
  epoch_last_1d = rtcData.last1d;
  
    }
    
  }

}

//Debugger functions for troubleshooting
void yieldEspCPUTime(int x) {
//  delay(100);
  ESP.wdtFeed();
//  yield();
  Serial.print("TimeDebug");
  Serial.print(x);
  Serial.print(",");
}

void yieldEspCPUTemp(int x) {
//  delay(100);
  ESP.wdtFeed();
//  yield();
  Serial.print("TempDebug");
  Serial.print(x);
  Serial.print(",");
}

void yieldEspCPU(int x) {
  delay(100);
  ESP.wdtFeed();
  yield();
  Serial.print("StdDebug");
  Serial.print(x);
  Serial.print(",");
}

//MQTT callback initializations
void my_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{

  if(!strcmp(message->topic,"sensors/+/temp"))
  {
    vector <string> uploadData;


    char * ESPBuf = std::strtok(message->payload,",");
    while (ESPBuf!=0)
    {
      uploadData.push_back(ESPBuf);
      ESPBuf = std::strtok(NULL,",")
    }

    ProcessTemp(uploadData);

  }

	if(message->payloadlen){
		printf("%s %s\n", message->topic, message->payload);
	}else{
		printf("%s (null)\n", message->topic);
	}
	fflush(stdout);
}

void my_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
	int i;
	if(!result){
		/* Subscribe to broker information topics on successful connect. */
		mosquitto_subscribe(mosq, NULL, "$SYS/#", 2);
		mosquitto_subscribe(mosq, NULL, "sensors/#", 1);
	}else{
		fprintf(stderr, "Connect failed\n");
	}
}

void my_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
	int i;

	printf("Subscribed (mid: %d): %d", mid, granted_qos[0]);
	for(i=1; i<qos_count; i++){
		printf(", %d", granted_qos[i]);
	}
	printf("\n");
}

void my_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
	/* Pring all log messages regardless of level. */
	printf("%s\n", str);
}

//End of MQTT callback section

const int NUM_SECONDS = 10;

void initialize() {
  // Setup AWS config
  Aws::SDKOptions options;
  Aws::InitAPI(options);

  Aws::Client::ClientConfiguration clientConfig;
  Aws::DynamoDB::DynamoDBClient dynamoClient(clientConfig);

  Aws::DynamoDB::Model::GetItemRequest req;

  //Initialize timer for program

  epoch = std::chrono::system_clock::now();
  
	    if (!epoch_initial) {
	    epoch_initial = epoch;
	    epoch_last_5m = epoch;
	    epoch_last_90m = epoch;
	    epoch_last_1d = epoch;
	    } 

	  if (!epoch_last_5m) {
	    //Just in case RTC memory loads a update time for 90m/1d that would be sooner than the 5m
	      epoch_last_5m = epoch;
	  }

    epoch_now = epoch;

    // Set up MQTT daemon and subscribe to topic
    // Mosquitto variables
    int i;
  	char *host = "localhost";
  	int port = 1883;
  	int keepalive = 60;
  	bool clean_session = true;
    struct mosquitto *mosq = NULL;

    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, clean_session, NULL);
	  if(!mosq){
		  fprintf(stderr, "Error: Out of memory.\n");
    }
    mosquitto_log_callback_set(mosq, my_log_callback);
	  mosquitto_connect_callback_set(mosq, my_connect_callback);
  	mosquitto_message_callback_set(mosq, my_message_callback);
    mosquitto_subscribe_callback_set(mosq, my_subscribe_callback);

    if(mosquitto_connect(mosq, host, port, keepalive)){
      fprintf(stderr, "Unable to connect to MQTT.\n")
    }

    
  //Temperature then output update loop, executes every 20 sec 
  if ( epoch_now - lastTempOutputUpdate > 20000 ) {
    Serial.println();
    Serial.print("Get Temp-");
    Serial.println();
    gettemperature();
    yield();
    Serial.println();
    Serial.println("Temp Successful");
    Serial.println();
    Serial.print("Update Output-");
    updateOutput();
    yield();
    Serial.println("Successful");
    Serial.println();
    lastTempOutputUpdate = epoch_now;
  }
  
}

//Checksum function to validate RTC data
uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

//RTC memory printout
void printMemory() {
  Serial.println();
  Serial.print("t5:");
  Serial.print(rtcData.t5);
  Serial.print(" t90:");
  Serial.print(rtcData.t90);
  Serial.print(" u5:");
  Serial.print(rtcData.u5);
  Serial.print(" u90:");
  Serial.print(rtcData.u90);
  Serial.print(" p5:");
  Serial.print(rtcData.p5);
  Serial.print(" p90:");
  Serial.print(rtcData.p90);
  Serial.print(" h5:");
  Serial.print(rtcData.h5);
  Serial.print(" h90:");
  Serial.print(rtcData.h90);
  Serial.print(" c5:");
  Serial.print(rtcData.c5);  
  Serial.print(" c90:");
  Serial.print(rtcData.c90);
  Serial.print(" initial:");
  Serial.print(rtcData.initial);
  Serial.print(" last90:");
  Serial.println(rtcData.last90);
  Serial.print(" last1d:");
  Serial.print(rtcData.last1d);
  Serial.println();
}

void DynamoPlaceTemp(const char* table_name, vector <string> outputData) {

  const Aws::String table(table_name);
  const Aws::String name(outputData[2]);

  Aws::DynamoDB::Model::PutItemRequest pir;
  pir.SetTableName(table);

  for (int x = 0; x < outputData.size(); x++)
  {
    const Aws::String arg(outputData[x]);
    const Aws::Vector<Aws::String>& flds = Aws::Utils::StringUtils::Split(arg, ':');
    if (flds.size() == 3)
    {
      if (flds[2] == "N")
      {
        Aws::DynamoDB::Model::AttributeValue val;
        val.SetN(flds[1]);
        pir.AddItem(flds[0], val);
      } else if (flds[2] == "S") {
        Aws::DynamoDB::Model::AttributeValue val;
        val.SetS(flds[1]);
        pir.AddItem(flds[0], val);
      } else {
        std::cout << "Invalid variable type: " << flds[2] << std::endl << USAGE;
      }
    }
    else
    {
        std::cout << "Invalid argument: " << arg << std::endl << USAGE;
    }
  }

  const Aws::DynamoDB::Model::PutItemOutcome result = dynamoClient.PutItem(pir);
    if (!result.IsSuccess())
    {
        std::cout << result.GetError().GetMessage() << std::endl;
        return 1;
    }
    std::cout << "PlaceTemp Done!" << std::endl;

}

void TimeLog(std::string calledby) {
  Serial.print("TimeLog ")
  Serial.print(calledby)
  Serial.print("Epoch now: ");
    Serial.print(epoch_now);
  Serial.print(" Epoch last 5m: ");
    Serial.print(epoch_last_5m);
  Serial.print(" Epoch last 90m: ");
    Serial.print(epoch_last_90m);
  Serial.print(" Epoch last 1d: ");
    Serial.print(epoch_last_1d);
} 

void ProcessTemp(vector <string> outputData) {

  std::vector<int>::iterator it;

  // extract humidity data from ESP MQTT msg
  it = std::find(outputData.begin(), outputData.end(), "humidity") 
  if (it != outputData.end())   
  {
    std::vector<string> flds;
    char * pch;
    pch = strtok(it, ":")
    while (pch != NULL)
    {
      flds.push_back(pch)
      pch = strtok (NULL, ":")
    }
    humidity = atof (flds[1]);
  }
  else
    cout << "Humidity value not found in message"

  // extract temperature data from ESP MQTT msg
  it = std::find(outputData.begin(), outputData.end(), "temp") 
  if (it != outputData.end())   
  {
    std::vector<string> flds;
    char * pch;
    pch = strtok(it, ":")
    while (pch != NULL)
    {
      flds.push_back(pch)
      pch = strtok (NULL, ":")
    }
    temp_f = atof (flds[1]);
  }
  else
    cout << "Temperature value not found in message"
  
  // extract heater data from ESP MQTT msg
  it = std::find(outputData.begin(), outputData.end(), "heaterState") 
  if (it != outputData.end())   
  {
    std::vector<string> flds;
    char * pch;
    pch = strtok(it, ":")
    while (pch != NULL)
    {
      flds.push_back(pch)
      pch = strtok (NULL, ":")
    }
    heaterState = atof (flds[1]);
    if (heaterState > 50)
      stateVariable = true;
    else 
      stateVariable = false;
  }
  else
    cout << "Heater value not found in message"

  // get the data supplied by the Pi for upload here - first get the current time and convert it to Amazon format
  epoch = std::chrono::system_clock::now();
  time_t awstime = epoch; 
  sprintf(awstimestr, "%04d%02d%02d%02d%02d%02d", year(awstime), month(awstime), day(awstime), hour(awstime), minute(awstime), second(awstime));
  uploadData.push_back("Date:" + awstimestr + ":N"); 

  // now get the current PID output
  updateOutput();
  uploadData.push_back("pidOutput:" + pidOutput ":N");

  //Push 20sec data to DyanmoDB
  DynamoPlaceTemp(TABLE_20S, uploadData);
  
  //Store 20sec data in vectors
  temp_f_20s.push_back(temp_f);
  humidity_20s.push_back(humidity);
  pid_20s.push_back(pidOutput);
  heater_20s.push_back(heaterState);

  TimeLog("After 20s data stored in vectors: ");

  //Handling once NTP time has passed 5min
  if (epoch_now - epoch_last_5m > 300) { 

    Serial.print("TE 48, ");

    //Averaging 20sec measurements and setting them to a 5min variable
    double temp_f_inst_5m = accumulate( temp_f_20s.begin(), temp_f_20s.end(), 0.0)/temp_f_20s.size();
    double humidity_inst_5m = accumulate( humidity_20s.begin(), humidity_20s.end(), 0.0)/humidity_20s.size(); 
    double pid_inst_5m = accumulate( pid_20s.begin(), pid_20s.end(), 0.0)/pid_20s.size();
    double heater_inst_5m = accumulate( heater_20s.begin(), heater_20s.end(), 0.0)/heater_20s.size();

    //store 5min data in vectors
    temp_f_5m.push_back(temp_f_inst_5m);
    humidity_5m.push_back(humidity_inst_5m);
    pid_5m.push_back(pid_inst_5m);
    heater_5m.push_back(heater_inst_5m);

    //Resetting 20sec vectors
    temp_f_20s.swap(std::vector<double>());
    humidity_20s.swap(std::vector<double>());
    pid_20s.swap(std::vector<double>());
    heater_20s.swap(std::vector<double>()); 

	  //Writing values to RTC memory in case of crash
	  RTCMemWrite();
      
	  Serial.print("TE 50, ");
    
      //Sending 5m avg measurements
      if (!flagDontPushAvgTemp) {
       	Serial.println("Pushing 5m average to DB:\t");
        Serial.print(temp_f_inst_5m);
        Serial.print(",");
        Serial.print(humidity_inst_5m);
        Serial.print(",");
        Serial.println(counter_20s);
            // print the hour, minute and second:
        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
        Serial.print(':');
        if (((epoch % 3600) / 60) < 10) {
          // In the first 10 minutes of each hour, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ((epoch % 60) < 10) {
          // In the first 10 seconds of each minute, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print(epoch % 60); // print the second
        Serial.print(",Epoch Now");
        Serial.print(",");
        Serial.print(epoch_now);
        Serial.print(",");
        Serial.print("Epoch Last Day");
        Serial.print(",");
        Serial.print(epoch_last_1d);
        Serial.print("Epoch Last 90m");
        Serial.print(",");
        Serial.print(epoch_last_90m);
        Serial.print("Epoch Last 5m");
        Serial.print(",");
        Serial.print(epoch_last_5m);
        Serial.print("Epoch Difference Now to Last 5m");
        Serial.print(",");
        Serial.print(epoch_now - epoch_last_5m);
        Serial.println();
	
      Serial.print("TE 51, ");

        PushTempToDynamoDB(temp_f_inst_5m, humidity_inst_5m, pid_inst_5m, heater_inst_5m, TABLE_NAME3, HASH_KEY_NAME3, HASH_KEY_VALUE3, RANGE_KEY_NAME3);
  yield();
	
      Serial.print("TE 52, ");

      }
  }

  if (epoch_now - epoch_last_90m > 5400) {

      Serial.print("TE 53, ");
      
      //Averaging 90min measurements and setting them to a 90min variable
      temp_f_inst_90m = temp_f_sum_5m / counter_5m;
      humidity_inst_90m = humidity_sum_5m / counter_5m; 
      pid_inst_90m = pid_sum_5m / counter_5m;
      heater_inst_90m = heater_sum_5m / counter_5m;

      //Resetting 5min sums
      temp_f_sum_5m = 0;
      humidity_sum_5m = 0;
      pid_sum_5m = 0;
      heater_sum_5m = 0;
    
      //Resetting 5min measurement instance counter
      counter_5m = 0;

      Serial.print("TE 54, ");
      
      flagDontPushAvgTemp = false;
    
      //Ticking over 90m time counter
      epoch_last_90m = epoch_now;
    
      //Adding cumulatively the values of the 90min measurements
      temp_f_sum_90m = temp_f_sum_90m + temp_f_inst_90m;
      humidity_sum_90m = humidity_sum_90m + humidity_inst_90m;
      pid_sum_90m = pid_sum_90m + pid_inst_90m;
      heater_sum_90m = heater_sum_90m + heater_inst_90m;
      
      //Incrementing 90min measurement instance counter
      counter_90m++;

        //Writing values to RTC memory
  RTCMemWrite();

      Serial.print("TE 55, ");
    
      //Sending 90m avg measurements
      if (!flagDontPushAvgTemp) {
        Serial.println("Pushing 90m average to DB:\t");
        Serial.print(temp_f_inst_90m);
        Serial.print(",");
        Serial.print(humidity_inst_90m);
        Serial.print(",");
        Serial.println(counter_5m);
            // print the hour, minute and second:
        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
        Serial.print(':');
        if (((epoch % 3600) / 60) < 10) {
          // In the first 10 minutes of each hour, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ((epoch % 60) < 10) {
          // In the first 10 seconds of each minute, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print(epoch % 60); // print the second
        Serial.print(",Epoch Now");
        Serial.print(",");
        Serial.print(epoch_now);
        Serial.print(",");
        Serial.print("Epoch Last Day");
        Serial.print(",");
        Serial.print(epoch_last_1d);
        Serial.print("Epoch Last 90m");
        Serial.print(",");
        Serial.print(epoch_last_90m);
        Serial.print("Epoch Last 5m");
        Serial.print(",");
        Serial.print(epoch_last_5m);
        Serial.print("Epoch Difference Now to Last 90m");
        Serial.print(",");
        Serial.print(epoch_now - epoch_last_90m);
        Serial.println();

      Serial.print("TE 56, ");

        PushTempToDynamoDB(temp_f_inst_90m, humidity_inst_90m, pid_inst_90m, heater_inst_90m, TABLE_NAME4, HASH_KEY_NAME4, HASH_KEY_VALUE4, RANGE_KEY_NAME4);
  yield();

      Serial.print("TE 57, ");

      }
     
  }

    if (epoch_now - epoch_last_1d > 86400) {
      
      Serial.print("TE 58, ");
      
      //Averaging 1 day measurements and setting them to a 1 day variable
      temp_f_inst_1d = temp_f_sum_90m / counter_90m;
      humidity_inst_1d = humidity_sum_90m / counter_90m; 
      pid_inst_1d = pid_sum_90m / counter_90m;
      heater_inst_1d = heater_sum_90m / counter_90m;

      //Resetting 90min sums
      temp_f_sum_90m = 0;
      humidity_sum_90m = 0;
      pid_sum_90m = 0;
      heater_sum_90m = 0;
    
      //Resetting 90min measurement instance counter
      counter_90m = 0;
      
      flagDontPushAvgTemp = false;
    
      //Ticking over 1d time counter
      epoch_last_1d = epoch_now;
    
//      //Adding cumulatively the values of the 1 day measurements ( no need here)
//      temp_f_sum_1d = temp_f_sum_1d + temp_f_inst_1d;
//      humidity_sum_1d = humidity_sum_1d + humidity_inst_1d;
//      pid_sum_1d = pid_sum_1d + pid_inst_1d;
//      heater_sum_1d = heater_sum_1d + heater_inst_1d;
      
//      //Incrementing 1 day measurement instance counter ( no need here)
//      1d_counter++;

        //Writing values to RTC memory
  RTCMemWrite();

      Serial.print("TE 59, ");
    
      //Sending 1d avg measurements
      if (!flagDontPushAvgTemp) {
         Serial.println("Pushing 1d average to DB:\t");
        Serial.print(temp_f_inst_1d);
        Serial.print(",");
        Serial.print(humidity_inst_1d);
        Serial.print(",");
        Serial.println(counter_90m);
            // print the hour, minute and second:
        Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
        Serial.print(':');
        if (((epoch % 3600) / 60) < 10) {
          // In the first 10 minutes of each hour, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
        Serial.print(':');
        if ((epoch % 60) < 10) {
          // In the first 10 seconds of each minute, we'll want a leading '0'
          Serial.print('0');
        }
        Serial.print(epoch % 60); // print the second
        Serial.print(",Epoch Now");
        Serial.print(",");
        Serial.print(epoch_now);
        Serial.print(",");
        Serial.print("Epoch Last Day");
        Serial.print(",");
        Serial.print(epoch_last_1d);
        Serial.print("Epoch Last 90m");
        Serial.print(",");
        Serial.print(epoch_last_90m);
        Serial.print("Epoch Last 5m");
        Serial.print(",");
        Serial.print(epoch_last_5m);
        Serial.print("Epoch Difference Now to Last 90m");
        Serial.print(",");
        Serial.print(epoch_now - epoch_last_90m);
        Serial.println();

      Serial.print("TE 60, ");

        PushTempToDynamoDB(temp_f_inst_1d, humidity_inst_1d, pid_inst_1d, heater_inst_1d, TABLE_NAME5, HASH_KEY_NAME5, HASH_KEY_VALUE5, RANGE_KEY_NAME5);
  yield();

      Serial.print("TE 61, ");

      }

  }

//Currently not using the counter overflow watchdog - NTP time checking seems to be sufficient 
//  if ((counter_20s > 20) || (counter_5m > 20) || (counter_90m > 20)) {
//    Serial.println("Too many measurements for time period!");
//    Serial.println("20sec measurements (per 5 min) now stored: ");
//    Serial.println(counter_20s);
//    Serial.println("5m measurements (per 90 min) now stored: ");
//    Serial.println(counter_5m);
//    Serial.println("90m measurements (per 1 day) now stored: ");
//    Serial.println(counter_90m);
//    flagDontPushAvgTemp = true;
//    Serial.println("Halting publish of avg measurements to DynamoDB tables");
//  }

}

void getHeaterControl(int *HeaterControl, GetItemOutput getItemOutput) {
  char szC[6];

  Serial.print("TE 7, ");

  /* Get the "item" from the getItem output. */
  MinimalMap < AttributeValue > attributeMap = getItemOutput.getItem();
  AttributeValue av;

  Serial.print("TE 8, ");
      
  attributeMap.get("HeaterControl", av);
  *HeaterControl = atoi(av.getS().getCStr());

  delay(10);
}

void awsGetHeaterControl(int *HeaterControl) {

  Serial.print("TE 31, ");

  /* Pull Heater State. */
  AttributeValue id;
  id.setS(HASH_KEY_VALUE2);
  rangeKey.setN(RANGE_KEY_VALUE2);

  Serial.print("TE 32, ");

  MinimalKeyValuePair < MinimalString, AttributeValue > pair1(HASH_KEY_NAME2, id);
  MinimalKeyValuePair < MinimalString, AttributeValue > pair2(RANGE_KEY_NAME2, rangeKey);
  MinimalKeyValuePair<MinimalString, AttributeValue> keyArray[] = { pair1, pair2 };
  getItemInput.setKey(MinimalMap < AttributeValue > (keyArray, KEY_SIZE));
  
  MinimalString attributesToGet[] = { "HeaterControl" };
  getItemInput.setAttributesToGet(MinimalList < MinimalString > (attributesToGet, 1));

  // Set Table Name
  getItemInput.setTableName(TABLE_NAME2);

  Serial.print("TE 33, ");

  // Perform getItem and check for errors.
  GetItemOutput getItemOutput = ddbClient.getItem(getItemInput, actionError);
  yield();

  Serial.print("TE 34, ");  

  switch (actionError) {

  Serial.print("TE 35, ");
    
    case NONE_ACTIONERROR:
      getHeaterControl(HeaterControl, getItemOutput);
      yield();
      break;

    case INVALID_REQUEST_ACTIONERROR:
      Serial.print("ERROR: ");
      Serial.println(getItemOutput.getErrorMessage().getCStr());
      break;
    case MISSING_REQUIRED_ARGS_ACTIONERROR:
      Serial.println("ERROR: Required arguments were not set for GetItemInput");
      break;
    case RESPONSE_PARSING_ACTIONERROR:
      Serial.println("ERROR: Problem parsing http response of GetItem\n");
      break;
    case CONNECTION_ACTIONERROR:
      Serial.println("ERROR: Connection problem");
      break;
  }
  
  Serial.print("TE 36, ");
}

void PushTempToDynamoDB(double temp_f_update, double humidity_update, double pid_update, double heater_update, const char* T_N, const char* HK_N, const char* HK_V, const char* RK_N) {

  Serial.print("TE 37, ");
  
    /* Create an Item. */
  AttributeValue id;
  id.setS(HK_V);
  AttributeValue timest;

  //Calculating my own time value for AWS given the last NTP time
  time_t awstime = epoch + (awsoffset - millis()) / 1000; 
  sprintf(awstimestr, "%04d%02d%02d%02d%02d%02d", year(awstime), month(awstime), day(awstime), hour(awstime), minute(awstime), second(awstime));

  timest.setN(awstimestr);
//  timest.setN(dateTimeProvider.getDateTime()); //Now using my own time system
//  yield();

  /* Create an AttributeValue for 'temp', convert the number to a
     string (AttributeValue object represents numbers as strings), and
     use setN to apply that value to the AttributeValue. */

  int d1 = temp_f_update;
  double f2 = temp_f_update - d1;
  int d2 = f2 * 100 + 1;

  char numberBuffer[20];
  AttributeValue tempAttributeValue;
  snprintf(numberBuffer, 20, "%d.%02d", d1, d2);
  tempAttributeValue.setN(numberBuffer);

  //AttributeValue for PID output
  int d3 = pid_update;
  double f3 = pid_update - d3;
  int d4 = f3 * 100 + 1;

  char pidBuffer[20];
  AttributeValue pidAttributeValue;
  snprintf(pidBuffer, 20, "%d.%02d", d3, d4);
  pidAttributeValue.setN(pidBuffer);

  //AttributeValue for heater output
  int d7 = heater_update;
  double f5 = heater_update - d7;
  int d8 = f5 * 100 + 1;

  char heaterbuffer[20];
  AttributeValue heaterOnState;
  snprintf(heaterbuffer, 20, "%d.%02d", d7, d8);
  heaterOnState.setN(heaterbuffer);

  //AttributeValue for humidity
  int d5 = humidity_update;
  double f4 = humidity_update - d5;
  int d6 = f4 * 100 + 1;

  char humidityBuffer[20];
  AttributeValue humidityAttributeValue;
  snprintf(humidityBuffer, 20, "%d.%02d", d5, d6);
  humidityAttributeValue.setN(humidityBuffer);

  /* Create the Key-value pairs and make an array of them. */
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att1(HK_N, id);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att2(RK_N, timest);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att3("temp", tempAttributeValue);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att4("pidOutput", pidAttributeValue);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att5("heaterState", heaterOnState);
  MinimalKeyValuePair < MinimalString, AttributeValue
  > att6("humidity", humidityAttributeValue);
  MinimalKeyValuePair<MinimalString, AttributeValue> itemArray[] = { att1,
                                                                     att2, att3, att4, att5, att6
                                                                   };

  Serial.print("TE 38, ");

  if (!isnan(temp_f)) {
    
    /* Set values for putItemInput. */
    putItemInput.setItem(MinimalMap < AttributeValue > (itemArray, 6));
    putItemInput.setTableName(T_N);
    
  Serial.print("TE 39, ");
  
    /* Perform putItem and check for errors. */

    PutItemOutput putItemOutput = ddbClient.putItem(putItemInput,
                                  actionError);
  
   Serial.print("TE 39.5, "); 

    yield();

  Serial.print("TE 40, ");
    
    switch (actionError) {
      case NONE_ACTIONERROR:
//        Serial.println("PutItem succeeded!");
        break;
      case INVALID_REQUEST_ACTIONERROR:
        Serial.print("ERROR: Invalid request");
        Serial.println(putItemOutput.getErrorMessage().getCStr());
        break;
      case MISSING_REQUIRED_ARGS_ACTIONERROR:
        Serial.println(
          "ERROR: Required arguments were not set for PutItemInput");
        break;
      case RESPONSE_PARSING_ACTIONERROR:
        Serial.println("ERROR: Problem parsing http response of PutItem");
        break;
      case CONNECTION_ACTIONERROR:
        Serial.println("ERROR: Connection problem");
        break;
    }
    /* wait to not double-record */
  }
    Serial.print("DUBRECWAIT, ");

  delay(2000);

  Serial.print("PDUBRECWAIT, ");
  
}



//Helper function to write the long-running data to RTC memory
void RTCMemWrite() {
  
  rtcData.t5 = temp_f_sum_5m;    
  rtcData.t90 = temp_f_sum_90m;
      
  rtcData.u5 = humidity_sum_5m;    
  rtcData.u90 = humidity_sum_90m;
      
  rtcData.p5 = pid_sum_5m;    
  rtcData.p90 = pid_sum_90m;
      
  rtcData.h5 = heater_sum_5m;    
  rtcData.h90 = heater_sum_90m;

  rtcData.c5 = counter_5m;
  rtcData.c90 = counter_90m;

  rtcData.initial = epoch_initial;
  rtcData.last90 = epoch_last_90m;
  rtcData.last1d = epoch_last_1d;

    // Update CRC32 of data
  rtcData.crc32 = calculateCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);
  // Write struct to RTC memory
  if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.println("Write: ");
    printMemory();
    Serial.println();
  }
  
}

void updateOutput() {
  unsigned long now2 = millis();

  //Don't compute the PID output if temp is invalid (will give an overflow as the PID output)
  if (!isnan(temp_f)) {
    myPID.Compute();
  }

  //Set initial window start time on initial run
  if (windowStartTime = 0)
    windowStartTime = epoch;
  
  //Shift the window once past 30min
  if (difftime(epoch, windowStartTime) > windowSize) {
    windowStartTime += windowSize;
  }

  //Heater control updates based on setting state and PID output - you can see at the start that this was based on the RF switch/outlet system, given the 'Outlet' helper function names
  if ((HeaterControl == 1) && (stateVariable))
  {
    Serial.println("Manual Control set to Off & Heater is On - Turning it Off");
    OffOutlet();
  }
  else if ((HeaterControl == 3) && (!stateVariable))
  {
    Serial.println("Manual Control set to On & Heater is Off - Turning it On");
    OnOutlet();
  }
  //I need to investigate whether the thermostat will turn the heater back on during a cycle that it has already turned it off within - ideally, the heater should only come on at the beginning of the window - I could set an additional 'HasTurnedOff' bool if need be to fix this if it's an issue
  else if ((HeaterControl == 2) && (pidOutput * windowSize > ((epoch - windowStartTime) * 100)) && (!stateVariable))
  {
    Serial.println("Automatic Control - Turning On");
    OnOutlet();
  }
  else if ((HeaterControl == 2) && (pidOutput * windowSize < ((epoch - windowStartTime) * 100)) && (stateVariable))
  {
    Serial.println("Automatic Control - Turning Off");
    OffOutlet();
  }
}


//Flash LED and turn heater on - transistor setting read before and after setting it for debugging purposes
void OnOutlet() {
  // if (check)
  // {
  //   Serial.println("Heater is already on; no need to turn it on. Exiting OnOutlet.")
  //   return;
  // }
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting before evaluation: ");
  Serial.println(transistorGate);
  delay(100);
  digitalWrite(transistorPin, HIGH);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting after evaluation: ");
  Serial.println(transistorGate);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Turning On Heater!");
  stateVariable = true;
  //Currently not using the RF switch functionality
  //  mySwitch.send("000101010001110100000011");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, LOW);
  //  mySwitch.send("000101010001110100000011");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
}

//Flash LED and turn heater off - transistor setting read before and after setting it for debugging purposes
void OffOutlet() {
  // if (!check)
  // {
  //   Serial.println("Heater is already off; no need to turn it off. Exiting OffOutlet.")
  //   return;
  // }
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting before evaluation: ");
  Serial.println(transistorGate);
  delay(100);
  digitalWrite(transistorPin, LOW);
  delay(100);
  transistorGate = digitalRead(14);
  Serial.print("Transistor gate setting after evaluation: ");
  Serial.println(transistorGate);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Turning Off Heater!"); //
  stateVariable = false;
  //Currently not using the RF switch functionality
  //  digitalWrite(LED_BUILTIN, LOW);
  //  mySwitch.send("000101010001110100001100");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, LOW);
  //  mySwitch.send("000101010001110100001100");
  //  delay(500);
  //  digitalWrite(LED_BUILTIN, HIGH);
}
