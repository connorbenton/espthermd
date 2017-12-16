//Libraries used are the Arduino PID library, ESP8266 built in libraries, aws-sdk-esp8266, DHT, TimeLib, and RCSwitch libraries
#include <stdio.h>
#include <stdint.h>
#include <iostream>
#include <chrono>
#include <cstring>
#include <string>

#include <mosquittopp.h>

#include <PID_v1.h>

//Still need to figure out what to do when an internet connection is not available, even if the router is still on
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


//Setting to store transistor state
int transistorGate;

//Setting to store heater state as reported by ESP
bool espState;

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



//Currently not in use, this is a setting that should trip and stop average uploads if too many lower level time periods have passed (i.e. if the 5min average has taken 100 measurements of 20sec apiece)
bool flagDontPushAvgTemp = false; 



//AWS constants
const char* AWS_REGION = "us-west-1";
const char* AWS_ENDPOINT = "amazonaws.com";



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

int findThermostat(vector<Thermostat*> &vals, string name_in)
{
  if(vals.size() == 0)
    return -1;
  for (int i = 0; i < vals.size(); i++){
    if (vals[i].thermoId == name_in)
      return i;
  }
}

//MQTT callback initializations
void my_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{

  if(!strcmp(message->topic,"sensors/+/temp"))
  {
    //Initialize string vector to store message data
    std::vector<string> espData;

    //Read message data into string vector
    char * ESPBuf = std::strtok(message->payload,",");
    while (ESPBuf!=0)
    {
      espData.push_back(ESPBuf);
      ESPBuf = std::strtok(NULL,",")
    }

    //Extract ID of the sending sensor
    std::string msgthermoId = extractDataStr(espData, "Id");

    //Vector to store all possible thermostats, so they can be checked
    static std::vector<Thermostat*> allThermos;
    
    //If vector has no items, automatically create the first new class member and process temp
    if(allThermos.size() == 0){
      thermo = new Thermostat(msgthermoId);
      thermo.ProcessTemp(espData);
      allThermos.push_back(thermo);
    }

    //Check if the thermo member already exists; if it does, process temp
    bool createnewThermoflag = true;
    for (int i = 0; i < allThermos.size(); i++){
      if(allThermos[i].thermoId == msgthermoId){
        allThermos[i].ProcessTemp(espData);
        createnewThermoflag = false;
        break;
      }
    }
    //If it doesn't, create a new thermo member then process temp
    if (createnewThermoflag){
      thermo = new Thermostat(msgthermoId);
      thermo.ProcessTemp(espData);
      allThermos.push_back(thermo);
    }

    //Clear message data vector
    espData.swap(std::vector<string>());

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

void DynamoPlaceTemp(const char* table_name, vector <string> uploadData) {

  const Aws::String table(table_name);
  const Aws::String name(uploadData[2]);

  Aws::DynamoDB::Model::PutItemRequest pir;
  pir.SetTableName(table);

  for (int x = 0; x < uploadData.size(); x++)
  {
    const Aws::String arg(uploadData[x]);
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

void TimeLog(std::string calledby, long epoch, long epoch_last_5m, long epoch_last_90m, long epoch_last_1d) {
  Serial.print("TimeLog ")
  Serial.print(calledby)
  Serial.print("Epoch now: ");
    Serial.print(epoch);
  Serial.print(" Epoch last 5m: ");
    Serial.print(epoch_last_5m);
  Serial.print(" Epoch last 90m: ");
    Serial.print(epoch_last_90m);
  Serial.print(" Epoch last 1d: ");
    Serial.print(epoch_last_1d);
} 

void OutputDataLog{
Serial.println("Pushing 90m average to DB:\t");
        Serial.print(temp_f_inst_90m);
        Serial.print(",");
        Serial.print(humidity_inst_90m);
        Serial.print(",");
        Serial.println(counter_5m);
        Serial.print size of the arrays 
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
}

double extractDataNum(std::vector<string> extractData, std::string iden) {
  std::vector<int>::iterator it;
  it = std::find(extractData.begin(), extractData.end(), iden) 
  if (it != extractData.end())   
  {
    std::vector<string> flds;
    char * pch;
    pch = strtok(it, ":")
    while (pch != NULL)
    {
      flds.push_back(pch)
      pch = strtok (NULL, ":")
    }
    double out = atof (flds[1]);
    return out;
  }
  else
  {
    cout << iden + " value not found in message"
    throw error here
  }
}

std::string extractDataStr(std::vector<string> extractData, std::string iden) {
  std::vector<int>::iterator it;
  it = std::find(extractData.begin(), extractData.end(), iden) 
  if (it != extractData.end())   
  {
    std::vector<string> flds;
    char * pch;
    pch = strtok(it, ":")
    while (pch != NULL)
    {
      flds.push_back(pch)
      pch = strtok (NULL, ":")
    }
    string out = atof (flds[1]);
    return out;
  }
  else
  {
    cout << iden + " value not found in message"
    throw error here
  }
}

class Thermostat {
  //This stores the unique ID of the thermostat
  std::string thermoId;
  //This stores the immediate (20sec) values of the thermostat
  double humidity, temp_f, heaterState, pidOutput;
  //This keeps track of whether the heater is currently turned on or off (at higher intervals, tells average time on)
  bool heaterbool;

  //Variables to keep track of current and past upload time (format is seconds since last unix epoch)
  unsigned long epoch, epoch_initial, epoch_last_5m, epoch_last_90m, epoch_last_1d;

  //This section of double vector matrices stores the current and running data for the data that will be outputted to DynamoDB 
  std::vector< vector<double> > temp_f_vec(3), humidity_vec(3), pid_vec(3), heater_vec(3);

  public:
  Thermostat(std::string);
  void ProcessTemp (std::vector<string>);
  void ProcessAvgTemp (int);
  double updateOutput (bool);

};

Thermostat::Thermostat (std::string identifier){
  thermoId = identifier;
}



void Thermostat::ProcessTemp(std::vector<string> inputData) {

  // Constants describing DynamoDB table and values being used
  const static char* TABLE_NAMES[] = { "Temperatures_20_sec", "Temperatures_5_min", "Temperatures_90_min", "Temperatures_1_day" };

  //Vector for 20s upload data
  std::vector<string> output_20s(inputData);

  // extract humidity data from ESP MQTT msg
  humidity = extractDataNum(inputData, "humidity");

  // extract temperature data from ESP MQTT msg
  temp_f = extractDataNum(inputData, "temp");
  
  // extract heater data from ESP MQTT msg
  heaterState = extractDataNum(inputData, "heaterState");
  if (heaterState > 50 && heaterState < 110)
    heaterbool = true;
  else if (heaterState < 50 && heaterState > -10)
    heaterbool = false;
  else 
    throw error here "invalid heater state from ESP"

  //Get current time
  epoch = std::chrono::system_clock::now();

  //Set initial time values if they're empty (start of program) 
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

  // get the data supplied by the Pi for upload here - first get the current time and convert it to Amazon format
  time_t awstime = epoch; 
  string awstimestr;
  sprintf(awstimestr, "%04d%02d%02d%02d%02d%02d", year(awstime), month(awstime), day(awstime), hour(awstime), minute(awstime), second(awstime));
  output_20s.push_back("Date:" + awstimestr + ":N"); 

  // now get the current PID output
  pidOutput = this->updateOutput(true);
  output_20s.push_back("pidOutput:" + pidOutput ":N");

  //Store 20sec data in vectors
  temp_f_vec[0].push_back(temp_f);
  humidity_vec[0].push_back(humidity);
  pid_vec[0].push_back(pidOutput);
  heater_vec[0].push_back(heaterState);

  //Push 20sec data to DyanmoDB
  DynamoPlaceTemp(TABLE_NAMES[0], output_20s);

  //Resetting output vector
  output_20s.swap(std::vector<string>());

  TimeLog("After 20s data stored in vectors: ");

  //Handling once time has passed beyond a new interval 
  if (epoch - epoch_last_5m > 300){
    this->ProcessAvgTemp(0);
    epoch_last_5m = epoch;
  } 

  if (epoch - epoch_last_90m > 5400){
    this->ProcessAvgTemp(1);
    epoch_last_90m = epoch;
  }
    
  if (epoch - epoch_last_1d > 86400){ 
    this->ProcessAvgTemp(2);
    epoch_last_1d = epoch;
  }

}

void Thermostat::ProcessAvgTemp (int i) {

  //Averaging input interval measurements and setting them to an output single variable
  double t_i = accumulate( temp_f_vec[i].begin(), temp_f_vec[i].end(), 0.0)/temp_f_vec[i].size();
  double hu_i = accumulate( humidity_vec[i].begin(), humidity_vec[i].end(), 0.0)/humidity_vec[i].size(); 
  double p_i = accumulate( pid_vec[i].begin(), pid_vec[i].end(), 0.0)/pid_vec[i].size();
  double he_i = accumulate( heater_vec[i].begin(), heater_vec[i].end(), 0.0)/heater_vec[i].size();

  //Create vector for upload data and add Id
  std::vector<string> outputvector;
  outputvector.push_back("Id:" + Thermostat.thermoId + ":S");

  //Upload time data
  unsigned long epoch = std::chrono::system_clock::now(); 
  time_t awstime = epoch; 
  string awstimestr;
  sprintf(awstimestr, "%04d%02d%02d%02d%02d%02d", year(awstime), month(awstime), day(awstime), hour(awstime), minute(awstime), second(awstime));
  outputvector.push_back("Date:" + awstimestr + ":N"); 

  //Convert other data to strings and add to output vector
  char buffer[20];

  snprintf(buffer, sizeof(buffer), "%.2f", t_i);
  outputvector.push_back("temp:" + buffer + ":N")
  buffer[0] = '\0';

  snprintf(buffer, sizeof(buffer), "%.2f", hu_i);
  outputvector.push_back("humidity:" + buffer + ":N")
  buffer[0] = '\0';

  snprintf(buffer, sizeof(buffer), "%.2f", p_i);
  outputvector.push_back("pidOutput:" + buffer + ":N")
  buffer[0] = '\0';

  snprintf(buffer, sizeof(buffer), "%.2f", he_i);
  outputvector.push_back("heaterState:" + buffer + ":N")
  buffer[0] = '\0';

  //If not dealing with the day long upload, then push average to output interval vector
  if (i<2){
    temp_f_vec[i+1].push_back(t_i);
    humidity_vec[i+1].push_back(hu_i);
    pid_vec[i+1].push_back(p_i);
    heater_vec[i+1].push_back(he_i);
  }

  //Resetting input interval vectors
  temp_f_vec[i].swap(std::vector<double>());
  humidity_vec[i].swap(std::vector<double>());
  pid_vec[i].swap(std::vector<double>());
  heater_vec[i].swap(std::vector<double>()); 

  //Writing values to RTC memory in case of crash, then log/debug output and time
  RTCMemWrite();
  LogOutputData();

  //Push data to DyanmoDB
  DynamoPlaceTemp(TABLE_NAMES[i+1], outputvector);

  //Resetting output vector
  outputvector.swap(std::vector<string>());

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

double Thermostat::updateOutput(bool set_compute_flag) {
  
  //Variables to keep track of current time (format is seconds since last unix epoch)
  static unsigned long epoch;
  epoch = std::chrono::system_clock::now();
  //Variables to keep track of PID window
  static unsigned long windowStartTime = 0;
  const static unsigned long windowSize = 1800; // 30 minutes (ish)
  //Set initial window start time on initial run
  if (windowStartTime = 0)
    windowStartTime = epoch;
  //Shift the window once past 30min
  if (difftime(epoch, windowStartTime) > windowSize) {
    windowStartTime += windowSize;
  }

  //PID tuning variables 
  const static double KP = 25;
  const static double KI = 0.028;
  const static double KD = 0;   // Not yet used
  //PID output variable
  static double pidCompute;
  static double temp_input = temp_f;
  //Setpoint hardcoded, will switch to a GetItem from DynamoDB in the future
  static double setPoint = 69.0;
  //PID variable initial setting
  const static PID myPID(&temp_input, &pidCompute, &setPoint, KP, KI, KD, DIRECT);

  //If only a setpoint update, don't step into the set loop - commented out for now, will see if fiddling with setpoint harms program
  // if (!set_compute_flag)
  //   return pidCompute;

  //Don't compute the PID output if temp is invalid (will give an overflow as the PID output)
  if (temp_f < 100 && temp_f > 0) {
    myPID.Compute();
  }

  //Heater control updates based on setting state and PID output - you can see at the start that this was based on the RF switch/outlet system, given the 'Outlet' helper function names
  if ((HeaterControl == 1) && (heaterbool))
  {
    Serial.println("Manual Control set to Off & Heater is On - Turning it Off");
    OffOutlet();
  }
  else if ((HeaterControl == 3) && (!heaterbool))
  {
    Serial.println("Manual Control set to On & Heater is Off - Turning it On");
    OnOutlet();
  }
  //I need to investigate whether the thermostat will turn the heater back on during a cycle that it has already turned it off within - ideally, the heater should only come on at the beginning of the window - I could set an additional 'HasTurnedOff' bool if need be to fix this if it's an issue
  else if ((HeaterControl == 2) && (pidOutput * windowSize > ((epoch - windowStartTime) * 100)) && (!heaterbool))
  {
    Serial.println("Automatic Control - Turning On");
    OnOutlet();
  }
  else if ((HeaterControl == 2) && (pidOutput * windowSize < ((epoch - windowStartTime) * 100)) && (heaterbool))
  {
    Serial.println("Automatic Control - Turning Off");
    OffOutlet();
  }

  return pidCompute;

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
