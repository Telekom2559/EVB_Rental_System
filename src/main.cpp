// MCU Board https://github.com/Xinyuan-LilyGO/LilyGO-T-A76XX
#include <Arduino.h>
#include <math.h>
#include <ArduinoJson.h>
#include <CRC32.h>
#include <Update.h>
#include "SPIFFS.h"
#include <esp_wifi.h>
#include <TinyGPSPlus.h>
#include <PubSubClient.h> // https://pubsubclient.knolleary.net/api
#include <SoftwareSerial.h>
#include <MPU6500_WE.h>
#include <Wire.h>
#include <Kalman.h>
// Serial Configuration
#define SerialAT                Serial1
#define SerialMon               Serial
#define UART_BAUD               115200
#define NEO6M_BAUD              9600
#define DEBUG                   false
// LTE Modem Configuration
#define MODEM_MODEL_A7670E // MODEM_MODEL_7600E // 
#if defined(MODEM_MODEL_A7670E)
#define MODEM_TX                26
#define MODEM_RX                27
#define MODEM_SUP               12
#define MODEM_RST               5
#define MODEM_PWR               4
#endif
#if defined(MODEM_MODEL_7600E)
#define MODEM_TX                27
#define MODEM_RX                26
#define MODEM_PWR               4
#define MODEM_FLIGHT            25
#endif
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
// LED Confugration
#define LED_LTE_PIN             23
#define LED_GPS_PIN             13
// ADC & Sensor Configuration
#define BATT_ADC                34
#define BATT_FULL_V             84.1 // 77
#define BATT_LOW_V              65.1
#define GPIO_VREF               3.3
#define ADC_VREF                1.1
#define ADC_12b_RES             4096
#define ADC_VAR                 278
#define ADC_INIT_W              100
#define KEY_ADC                 33
#define STAND_PIN               14
#define BUZZER_PIN              15
#define NEO6M_TX                19 //18
#define NEO6M_RX                18 //19
TinyGPSPlus gps_module;
SoftwareSerial gpsAT(NEO6M_RX, NEO6M_TX);
#define MPU6500_ADDR        0x68
#define DEG_RAD_SF          56.273
#define SDA_PIN             21
#define SCL_PIN             22

// Safety Confuration
#define CRITICAL_ANGLE          (int)70
#define STOP_MOTION_SPEED_KMH   (int)5

// MQTT Configuration
#define MQTT_QOS                0
#define MQTT_PRT                1883
# define FMT_MQTT // ANDA_MQTT // 
#if defined(ANDA_MQTT)
#define MQTT_SVR                "mq2.mydevice.cloud"
#define MQTT_RSP_TOPIC_PREFIX   "gpslocate/test/EVB_PROJ/report/"
#define MQTT_USERNAME           "blescan"
#define MQTT_PASS               "anunda795"
#endif
#if defined(FMT_MQTT)
#define MQTT_SVR                "fmt.aisquare.info"
#define MQTT_RSP_TOPIC          "/EVBIKE/update"
#define MQTT_USERNAME           "fmt"
#define MQTT_PASS               "Evbik3"
#endif
char MQTT_NAME[20];
char MQTT_CMD_TOPIC[50]; // format is /<dev_id>
PubSubClient mqtt(client);
char mqtt_client_id[20];

// Firmware Configuration OTA
#define FW_SERVER               "161.246.35.199"
#define FW_PRT                  80
#define FW_SRC_FNAME            "/~tung/T2/evb_firmware.bin"
#define FW_DEST_FNAME           "/evb_firmware.bin"
#define DANGER_SPACE            30000
uint32_t knownCRC32 = 0;

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
Kalman kalmanRoll; // Create the Kalman instances
Kalman kalmanPitch;

// Global Var
xyzFloat gyr;
xyzFloat aValue;
// float temp;
float resultantG;
uint32_t ts_rollpitch;

// Task Duration
#define LTE_WAIT                (5*60000)
#define REPORT_WAIT             15000
// Task Handle
TaskHandle_t                    lteTask_handle = NULL;
TaskHandle_t                    statusUpdateTask_handle = NULL;
TaskHandle_t                    cmdReceiveTask_handle = NULL;
TaskHandle_t                    sideStandTask_handle = NULL;
TaskHandle_t                    otaUpdateTask_handle = NULL;
TaskHandle_t                    buzzerTask_handle = NULL;
TaskHandle_t                    pubAliveTask_handle = NULL;
TaskHandle_t                    imuProcessTask_handle = NULL;
// Semaphore handles
SemaphoreHandle_t               sem_at;
SemaphoreHandle_t               sem_pub_data;
SemaphoreHandle_t               sem_cmd_process;
SemaphoreHandle_t               sem_side_stand;
SemaphoreHandle_t               sem_ota_update;
SemaphoreHandle_t               sem_buzzer;
SemaphoreHandle_t               sem_pub_alive;
// Queu handles
QueueHandle_t                   queue_cmd = NULL;

// initialize variable here
bool relay_trig = false;
char DEV_NAME[10];
String init_drive_mode = "";
uint8_t mqtt_failed_connect = 0;

// Sensor & Data structure
typedef struct GPS_DATA
{ 
    String date = "";
    String timen = "";
    String lat = "";
    String lon = "";
    String alt = "";
    String speed = "";
    String course = ""; // convert to int8?
};

typedef struct BATT_DATA
{ 
    double v_batt; 
    double percent_batt; 
};

typedef struct NETWORK_INFO
{
  // LBS
  char date[20];
  char utc[20];
  double lat = 0;
  double lon = 0;
  // CPSI
  char type[10];
  char mode[10];
  int mcc = 0;
  int mnc = 0;
  int tac = 0;
  int cid = 0;
  char freq_b[15];
  double rsrq=0;
  double rsrp=0;
  double rssi=0;
  int rssnr=0;
};
NETWORK_INFO network_lbs_info;

// typedef struct BIKE_INTERFACE
// {
//   char park[5];
//   char key[5];
//   char buzzer_state[5];
// };

struct MOTION_DATA
{
  float roll = 0;
  float pitch = 0;
  float longitudinal_acc = 0;
};

typedef struct BIKE_INTERFACE
{
  String park = "";
  String key = "";
  String buzzer_state = "";
};

typedef struct JS_EVB_SUB
{
  char dev_id[10];
  char drive_mode[20];
  char buzzer_state[20];
  char system[10];
  char params[15];
};
JS_EVB_SUB js_sub_dev;

typedef struct JS_EVB_PUB // Include all data structure here
{
  char dev_id[10];
  char drivemode[20];
  String route_id = "";
  GPS_DATA gps;
  BATT_DATA batt;
  NETWORK_INFO network;
  BIKE_INTERFACE bike;
};
JS_EVB_PUB js_pub_dev;

// SPIFFS Config
bool espWriteFile(char* _path, char* _fname, char* _mode, String _data_to_save)
{
  bool _flag = 0;
  char _full_fname[20];
  memset(_full_fname,0,sizeof(_full_fname));
  memset(_full_fname,0,sizeof(_full_fname));
  if (strcmp(_path,"/") == 0)
  {
    sprintf(_full_fname,"%s%s",_path,_fname);
  }
  else
  {
    sprintf(_full_fname,"%s/%s",_path,_fname);
  }
  SerialMon.println("_full_fname is "+String(_full_fname));

  if(!SPIFFS.begin(true))
  {
      SerialMon.print(F("An Error has occurred while mounting SPIFFS \r\n"));
      return _flag;
  }

  // Check spiffs space
  int _free_space = (SPIFFS.totalBytes() - SPIFFS.usedBytes());
  if (_free_space <= (int)DANGER_SPACE)
  {
    SerialMon.printf("SPIFFS is full!!!, start to remove SPIFFS file.\r\n");
    File _root = SPIFFS.open("/");
    File _file = _root.openNextFile();
    char _fname_target[20];
    while(_file)
    {
      SerialMon.println(String(_file.name())+", size = "+String(_file.size())+" bytes.");
      String _tmp_check_fname = String(_file.name());
      memset(_fname_target,0,sizeof(_fname_target));
      if (_tmp_check_fname.endsWith(".csv"))
      {
        sprintf(_fname_target,"%s%s",_path,_file.name());
        SPIFFS.remove(_fname_target);
        (SPIFFS.exists(_fname_target))? SerialMon.printf("%s still exists.\r\n",_fname_target): SerialMon.printf("%s has already removed.\r\n",_fname_target);
      }
      _file = _root.openNextFile();
    }
  }

  File _fileToWrite = SPIFFS.open(_full_fname, _mode);
  if(!_fileToWrite)
  {
    SerialMon.print(F("There was an error opening the file for writing \r\n"));
    return _flag;
  }
  if(_fileToWrite.println(_data_to_save))
  {
    // SerialMon.printf("data saved : %s \r\n",_data);
    SerialMon.print(F("File was written \r\n"));
    _flag = 1;
  } 
  else 
  {
    SerialMon.print(F("File write failed \r\n"));
    return _flag;
  }
  _fileToWrite.close();
  SPIFFS.end();
  return _flag;
}

void espReadConfig(char* _path, char* _conf_name, char* _dev_name)
{
  char _conf_filename[20];
  memset(_conf_filename,0,sizeof(_conf_filename));
  StaticJsonDocument<300> _doc;
  if (strcmp(_path,"/") == 0)
  {
    sprintf(_conf_filename,"%s%s",_path, _conf_name);
  }
  else
  {
    sprintf(_conf_filename,"%s/%s",_path, _conf_name);
  }
  // SerialMon.println("_conf_filename = "+String(_conf_filename));

  if(!SPIFFS.begin(true)){
      SerialMon.println("An Error has occurred while mounting SPIFFS");
      return;
  }

  File _file = SPIFFS.open(_conf_filename, "r");

  if(!_file) {
    SerialMon.println("Failed to open file to read.");
    return;
  }

  String _tmp_fread = _file.readString();
  SerialMon.println(_tmp_fread);
  	
  DeserializationError error = deserializeJson(_doc, _tmp_fread.c_str());
  if (error) {
    SerialMon.print("deserializeJson() failed: ");
    SerialMon.println(error.c_str());
    return;
  }
  strcpy(_dev_name,_doc["dev_id"]);
  _file.close();
  SPIFFS.end();
}

String espReadDriveState(char* _path, char* _conf_name)
{
  char _conf_filename[20];
  memset(_conf_filename,0,sizeof(_conf_filename));
  String _drive_state = "off";
  // espListFile("/");
  StaticJsonDocument<400> _doc;
  if (strcmp(_path,"/") == 0)
  {
    sprintf(_conf_filename,"%s%s",_path, _conf_name);
  }
  else
  {
    sprintf(_conf_filename,"%s/%s",_path, _conf_name);
  }
  // SerialMon.println("_conf_filename = "+String(_conf_filename));

  if(!SPIFFS.begin(true)){
      SerialMon.println("An Error has occurred while mounting SPIFFS");
      return _drive_state;
  }

  File _file = SPIFFS.open(_conf_filename, "r");

  if(!_file) {
    SerialMon.println("Failed to open file to read.");
    return _drive_state;
  }

  String _tmp_fread = _file.readString();
  SerialMon.println(_tmp_fread);
  	
  DeserializationError error = deserializeJson(_doc, _tmp_fread.c_str());
  if (error) {
    SerialMon.print("deserializeJson() failed: ");
    SerialMon.println(error.c_str());
    return _drive_state;
  }
  if (_doc.containsKey("drive_mode"))
  {
    char _drive_state_ch[11]; memset(_drive_state_ch,0,sizeof(_drive_state_ch));
    strcpy(_drive_state_ch,_doc["drive_mode"]);
    _drive_state = String(_drive_state_ch);
  }
  // strcpy(_drive_state,_doc["drive_mode"]);
  _file.close();
  SPIFFS.end();
  return _drive_state;
}

// LTE Function
void modemOn()
{
  // Choose starting method follow the modem's model
  bool reply = false;
  #if defined(MODEM_MODEL_A7670E)
    pinMode(MODEM_SUP, OUTPUT);
    digitalWrite(MODEM_SUP, HIGH);
    pinMode(MODEM_RST, OUTPUT);
    digitalWrite(MODEM_RST, LOW);
    delay(100);
    digitalWrite(MODEM_RST, HIGH);
    delay(3000);
    digitalWrite(MODEM_RST, LOW);
    pinMode(MODEM_PWR, OUTPUT);
    digitalWrite(MODEM_PWR, LOW);
    delay(100);
    digitalWrite(MODEM_PWR, HIGH);
    delay(1000);
    digitalWrite(MODEM_PWR, LOW);
  #endif
  #if defined(MODEM_MODEL_7600E)
    pinMode(MODEM_PWR, OUTPUT);
    digitalWrite(MODEM_PWR, HIGH);
    delay(300); //Need delay
    digitalWrite(MODEM_PWR, LOW);
    pinMode(MODEM_FLIGHT, OUTPUT);
    digitalWrite(MODEM_FLIGHT, HIGH);
  #endif
  // For modem wakeup ensuring, test with AT command & response
  int i = 10;
  SerialMon.println("\nTesting Modem Response...\n");
  SerialMon.println("****");
  while (i) {
    SerialAT.println("AT");
    delay(500);
    if (SerialAT.available()) {
      String r = SerialAT.readString();
      SerialMon.println(r);
      if ( r.indexOf("OK") >= 0 ) {
        reply = true;
        break;
      }
    }
    delay(500);
    i--;
  }
  SerialMon.println("****\n");
}

String sendData(String _command, const int _timeout, boolean _debug)
{
  String _response = "";
  SerialAT.println(_command);
  SerialAT.flush();
  long int time = millis();
  while ( (time + _timeout) > millis())
  {
    while (SerialAT.available())
    {
      char _c = SerialAT.read();
      _response += _c;
    }
  }
  if (_debug)
  {
    SerialMon.print(_response);
  }
  return _response;
}

void connectLTE()
{
  if (SerialAT.availableForWrite())
  {
    sendData("AT+CPMS=\"SM\", \"SM\", \"SM\"",1000, DEBUG);
    sendData("AT+CMGD=,4",1000,DEBUG);
    sendData("AT+NETCLOSE",2000,DEBUG);
    sendData("AT+CCID", 3000, DEBUG);
    sendData("AT+CREG?", 3000, DEBUG);
    sendData("AT+CGATT=1", 1000, DEBUG);
    sendData("AT+CGACT=1,1", 1000, DEBUG);
    sendData("AT+CGDCONT=1,\"IP\",\"internet\"", 1000, DEBUG);
    sendData("AT+NETOPEN",2000,DEBUG);
    sendData("AT+IPADDR",2000,DEBUG);
    sendData("AT+CPING=\"www.google.co.th\",1,4",9000,1);
  }
  else
  {
    SerialMon.printf("SIM7600 serial is busy!!. \r\n");
  }
  SerialAT.flush();
}

void readNetworkInfo(NETWORK_INFO* _network_info)
{
  char _buff[112];
  uint8_t _field_idx = 0;
  uint8_t _sub_field_idx = 0;
  // char _tmp_mcc_mnc[10];
  String _res_cnet = sendData("AT+CNETSTART?",500,0);
  if(strcmp(_res_cnet.substring(_res_cnet.indexOf('+'),_res_cnet.indexOf('+')+13).c_str(),"+CNETSTART: 2") != 0)
  {
    sendData("AT+CNETSTOP",500,0);
    _res_cnet = sendData("AT+CNETSTART",500,0);
  }
  String _cpsi_stat = sendData("AT+CPSI?",2000,0);
  // Serial.println()
  // Exmaple of cpsi data "\r\n+CPSI: LTE,Online,520-03,0x0338,157455481,55,EUTRAN-BAND1,350,3,3,-124,-873,-575,14\r\n\r\nOK\r\n";
  memset(_buff,0,sizeof(_buff));
  if (strcmp(_cpsi_stat.substring(_cpsi_stat.indexOf('+'),_cpsi_stat.indexOf('+')+10).c_str(),"+CPSI: LTE") == 0)
  {
    sprintf(_buff, _cpsi_stat.substring(_cpsi_stat.indexOf('+')+7,_cpsi_stat.indexOf('K')-5).c_str());
    char* _token = strtok(_buff, ",");
    while(_token != NULL)
    {
      // SerialMon.println(_token);
      if (_field_idx == 0)
      {
        strcpy(_network_info->type,_token);
      }
      if (_field_idx == 1)
      {
        strcpy(_network_info->mode,_token);
      }
      if (_field_idx == 2)
      {
        String _tmp_mcc_mnc = String(_token);
        _network_info->mcc = atoi(_tmp_mcc_mnc.substring(0,_tmp_mcc_mnc.indexOf('-')).c_str());
        _network_info->mnc = atoi(_tmp_mcc_mnc.substring(_tmp_mcc_mnc.indexOf('-')+1).c_str());
      }
      if (_field_idx == 3)
      {
        _network_info->tac = strtoul(_token,NULL,16);
      }
      if (_field_idx == 4)
      {
        _network_info->cid = atoi(_token);
      }
      if (_field_idx == 6)
      {
        strcpy(_network_info->freq_b,_token);
      }
      if (_field_idx == 10)
      {
        _network_info->rsrq = atoi(_token)/10;
      }
      if (_field_idx == 11)
      {
        _network_info->rsrp = atoi(_token)/10;
      }
      if (_field_idx == 12)
      {
        _network_info->rssi = atoi(_token)/10;
      }
      if (_field_idx == 13)
      {
        _network_info->rssnr = atoi(_token);
      }
      _token = strtok(NULL, ",");
      _field_idx ++;
    }
  }
}

// --------------- Battery Reading Function ------------------------------
uint16_t readAdcBattery(uint8_t bat_adc) // see from https://deepbluembedded.com/esp32-adc-tutorial-read-analog-voltage-arduino/
{
  // Read with 12 bits resolution [0,4096] in integer
  return (uint16_t)analogRead(bat_adc);
}

void adcAvg(uint8_t bat_adc, uint16_t* adc, uint8_t w) // ใช้หาค่า prior_adc ตอนเริ่มต้น
{
  uint8_t i = 0;
  uint32_t sum = 0;
  while (i < w)
  {
    sum += readAdcBattery(bat_adc);
    i++;
    // delay(100);
  }
  *adc = (uint16_t)(sum/w);
}

double readVoltBattery(uint16_t _adc) // แปลงค่า prior_adc เป็นแรงดันในหน่วยโวลต์ โดยมีค่าไม่เกิน BATT_FULL_V
{
  double _volt = 48*((0.9889*_adc+165)*(GPIO_VREF/ADC_12b_RES)+0.04)-0.896; //(24.95)*_adc*(GPIO_VREF/ADC_12b_RES)+0.94; // Should fit from csv
  // Serial.println("Volt = "+String(_volt)+" V.");
  if (_volt > (double) BATT_FULL_V)
  {
    _volt = (double) BATT_FULL_V;
  }
  else if (_volt < BATT_LOW_V)
  {
    _volt = (double) BATT_LOW_V;
  }
  return _volt;
}

double percentBattery(double _adc) // คำนวณ % Batt ที่เหลืออยู่เทียบกับแรงดันตอนแบตเต็ม (BATT_FULL_V)
{
  return (100*(1-(BATT_FULL_V-_adc)/(BATT_FULL_V-BATT_LOW_V)));
}

// ----------------  IMU Function -------------------
float rollCalculation(float _ax, float _ay, float _az)
{
  (abs(_az) < 1E-4)? _az = 1E-4 : _az = _az - 9.81;
  return atan(_ay/sqrt(pow(_ax,2) + pow(_az,2)))*DEG_RAD_SF;
}

float pitchCalculation(float _ax, float _ay, float _az)
{
  (abs(_az) < 1E-4)? _az = 1E-4 : _az = _az - 9.81;
  return atan(_ax/sqrt(pow(_ay,2) + pow(_az,2)))*DEG_RAD_SF;
}

bool imuInitial()
{
  bool _imu_init = 0;
  Wire.setPins(SDA, SCL);
  if(!Wire.begin())
  {
    SerialMon.println("Init wire error!!!");
    return _imu_init;
  }
  delay(2000);
  uint8_t imu_connect_try = 3;
  
  while(imu_connect_try)
  {
    if (!myMPU6500.init())
    {
      SerialMon.print(F("Try to connect IMU agian.\r\n"));
    }
    else
    {
      SerialMon.println("MPU6500 is connected");
      SerialMon.println("Position you MPU6500 flat and don't move it - calibrating...");
      delay(1000);
      myMPU6500.autoOffsets();
      SerialMon.println("Done!");
      myMPU6500.enableGyrDLPF();
      myMPU6500.setGyrDLPF(MPU6500_DLPF_6);
      myMPU6500.setSampleRateDivider(5);
      myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_1000); //(MPU6500_GYRO_RANGE_250);
      myMPU6500.setAccRange(MPU6500_ACC_RANGE_8G); //(MPU6500_ACC_RANGE_2G);
      myMPU6500.enableAccDLPF(true);
      myMPU6500.setAccDLPF(MPU6500_DLPF_6);
      delay(200);

      aValue = myMPU6500.getAccRawValues();
      gyr = myMPU6500.getGyrValues();
      float z_roll = rollCalculation(aValue.x,aValue.y,aValue.z);
      float z_pitch = pitchCalculation(aValue.x,aValue.y,aValue.z);

      SerialMon.println("roll = "+String(z_roll)+" deg, Pitch = "+String(z_pitch)+" deg");
      kalmanRoll.setAngle(z_roll); // Set starting angle
      kalmanPitch.setAngle(z_pitch);
      // kalmanRoll.setRmeasure(5);
      // kalmanPitch.setRmeasure(5);
      ts_rollpitch = micros();
      _imu_init = 1;
      break;
    }
    imu_connect_try--;
    delay(1000);
  }
  return _imu_init;
}

// Motorcycle Interface
void relayTrigger(int _rl_pin, bool _logic)
{
  digitalWrite(_rl_pin, _logic);
}

void buzzerBeep(uint16_t _beep_rate_ms, uint8_t _beep_round)
{
  SerialMon.print(F("------ buzzerBeepkTask ------\r\n"));
  for (uint8_t _i=0; _i<_beep_round; _i++)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    vTaskDelay(_beep_rate_ms/portTICK_PERIOD_MS);
    digitalWrite(BUZZER_PIN, LOW);
    vTaskDelay(_beep_rate_ms/portTICK_PERIOD_MS);
  }
  digitalWrite(BUZZER_PIN, LOW);
}

// Report JSON function
String reportJsonUpdater(JS_EVB_PUB* _input_data, String _init_drive_mode)
{
  char _output[900];
  String _out_str = "";
  StaticJsonDocument<900> _doc;
  _doc["dev_id"] = DEV_NAME;
  JsonObject _gps_data = _doc.createNestedObject("gps_data");
  _gps_data["gpsd"] = _input_data->gps.date;
  _gps_data["gpst"] = _input_data->gps.timen;
  _gps_data["lat"] = _input_data->gps.lat;
  _gps_data["lon"] = _input_data->gps.lon;
  _gps_data["speed"] = _input_data->gps.speed;
  _gps_data["cousre"] = _input_data->gps.course;

  JsonObject _batt_data = _doc.createNestedObject("batt_data");
  _batt_data["voltage"] = String(_input_data->batt.v_batt);
  _batt_data["level"] = String(_input_data->batt.percent_batt);

  JsonObject _network = _doc.createNestedObject("network");
  _network["date"] = _input_data->network.date;
  _network["utc"] = _input_data->network.utc;
  _network["lat"] = String(_input_data->network.lat);
  _network["lon"] = String(_input_data->network.lon);
  _network["type"] = _input_data->network.type;
  _network["mode"] = _input_data->network.mode;
  _network["mcc"] = String(_input_data->network.mcc);
  _network["mnc"] = String(_input_data->network.mnc);
  _network["lac"] = String(_input_data->network.tac);
  _network["cid"] = String(_input_data->network.cid);
  _network["freq_b"] = _input_data->network.freq_b;
  _network["rsrq"] = String(_input_data->network.rsrq);
  _network["rsrp"] = String(_input_data->network.rsrp);
  _network["rssi"] = String(_input_data->network.rssi);
  _network["rssnr"] = String(_input_data->network.rssnr);
  _doc["key"] = _input_data->bike.key;
  _doc["drive_mode"] =  _init_drive_mode;
  if (_doc["drive_mode"] == "on")
  {
    _doc["park"] = "no";
  }
  else if(_doc["drive_mode"] == "off")
  {
    _doc["park"] = "yes";
  }
  _doc["buzzer_state"] = _input_data->bike.buzzer_state;
  _doc["routeID"] = _input_data->route_id;
  serializeJson(_doc, _output);
  _out_str = String(_output);
  return _out_str;
}

// MQTT Function
void callback(char* topic, byte* message, unsigned int length) // callback(char* _topic, byte* _message, unsigned int _len) //, JS_FW_CMD* _js_cmd_dev)
{
  // SerialMon.printf("Message arrived on topic: ");
  // SerialMon.printf(topic);
  // SerialMon.printf(". Message: ");
  String msg_sub;
  for (unsigned int i = 0; i < length; i++) 
  {
    msg_sub += (char)message[i];
  }
  // SerialMon.println("msg_sub is "+msg_sub);
  char _tmp_msg[length+1];
  memset(_tmp_msg,0,sizeof(_tmp_msg));
  memcpy(_tmp_msg,message,sizeof(_tmp_msg));
  _tmp_msg[length] = '\0';
  // Serial.println("_tmp_msg is "+String(_tmp_msg));
  if (xQueueSend(queue_cmd, _tmp_msg, 0) != pdPASS)
  {
    SerialMon.print(F("Failed to keep CMD in mqtt_queue.\r\n"));
  }
  else
  {
    xSemaphoreGive(sem_cmd_process);
  }
}

void mqttReconnect() 
{
  SerialMon.print(F("Attempting MQTT connection... \r\n"));
  mqtt.setServer(MQTT_SVR, MQTT_PRT);
  mqtt.setCallback(callback);
  mqtt.setBufferSize(900);
  if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASS)) 
  {
    Serial.println("MQTT_CMD_TOPIC is "+String(MQTT_CMD_TOPIC));
    mqtt.subscribe(MQTT_CMD_TOPIC, MQTT_QOS);
    SerialMon.print(F("connected \r\n"));
  } 
  else 
  {
    SerialMon.print("failed, rc = ");
    SerialMon.println(mqtt.state());
  }
}

// --------------------------- Firmware OTA Function ---------------------------------
void printPercent(uint32_t readLength, uint32_t contentLength) {
  // If we know the total length
  if (contentLength != -1) {
    SerialMon.print("\r ");
    SerialMon.print((100.0 * readLength) / contentLength);
    SerialMon.println('%');
  } else {
    SerialMon.println(readLength);
  }
}

void performUpdate(Stream &updateSource, size_t updateSize)
{
    if (Update.begin(updateSize))
    {
        size_t written = Update.writeStream(updateSource);
        if (written == updateSize)
        {
            Serial.println("writings : " + String(written) + " successfully");
        }
        else
        {
            Serial.println("only writing : " + String(written) + "/" + String(updateSize) + ". Retry?");
        }
        if (Update.end())
        {
            Serial.println("OTA accomplished!");
            if (Update.isFinished())
            {
                Serial.println("OTA ended. restarting!");
                ESP.restart();
            }
            else
            {
                Serial.println("OTA didn't finish? something went wrong!");
            }
        }
        else
        {
            Serial.println("Error occured #: " + String(Update.getError()));
        }
    }
    else
    {
        Serial.println("without enough space to do OTA");
    }
}

void updateFromFS(char* _fw_name)
{
  File updateBin = SPIFFS.open(_fw_name, "r");
  if (updateBin)
  {
    size_t updateSize = updateBin.size();

    if (updateSize > 0)
    {
        Serial.println("start of updating");
        performUpdate(updateBin, updateSize);
        SerialMon.println("updateSize is "+String(updateSize)+" bytes.");
    }
    else
    {
        Serial.println("Error, file is empty");
    }

    updateBin.close();
    if (!SPIFFS.remove(_fw_name)) SerialMon.print(F("Delete firmware Failed!!.\r\n"));
    // whe finished remove the binary from sd card to indicate end of the process
    // fs.remove("/firmware.bin");
  }
  else
  {
    Serial.println("can't open "+String(_fw_name));
  }
}
// OTA code from https://github.com/vshymanskyy/TinyGSM/issues/630
bool updateOTA(char* _fw_server, uint16_t _fw_prt, char* _src_fw_name, char* _des_fw_name, uint32_t _fw_crc)
{
  SerialMon.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return false;
  }
  SerialMon.println(" OK");
  SerialMon.print(F("Connecting to "));
  SerialMon.print(_fw_server);
  if (!client.connect(_fw_server, _fw_prt)) {
    SerialMon.println(" fail");
    delay(10000);
    return false;
  }
  SerialMon.println(" OK");

  // Make a HTTP GET request:
  client.print(String("GET ") + _src_fw_name + " HTTP/1.1\r\n");
  client.print(String("Host: ") + _fw_server + "\r\n\r\n\r\n\r\n");
  client.print("Connection: close\r\n\r\n\r\n\r\n");

  long timeout = millis();
  while (client.available() == 0) {
    if (millis() - timeout > 5000L) {
      SerialMon.println(F(">>> Client Timeout !"));
      client.stop();
      // delay(10000L);
      // return false;
    }
  }

  SerialMon.println(F("Reading response header"));
  uint32_t contentLength = 1024;
  
  if(!SPIFFS.begin())
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return false;
  }

  File file = SPIFFS.open(_des_fw_name, "w");

  while (client.connected()) {
    String line = client.readStringUntil('\n');
    line.trim();
    SerialMon.println(line);    // Uncomment this to show response header
    line.toLowerCase();
    if (line.startsWith("content-length:")) {
      contentLength = line.substring(line.lastIndexOf(':') + 1).toInt();
    } else if (line.length() == 0) {
      break;
    }
  }

  SerialMon.println(F("Reading response data"));
  timeout = millis();
  uint32_t readLength = 0;
  CRC32 crc;

  unsigned long timeElapsed = millis();
  printPercent(readLength, contentLength);
  while (readLength < contentLength && client.connected() && ((millis() - timeout) < 190000)) 
  {
    // (readLength < contentLength)? Serial.println("readLength < contentLength is true") : Serial.println("readLength < contentLength is false");
    // (client.connected())? Serial.println("client.connected() is true") : Serial.println("client.connected() is false");
    // (client.available())? Serial.println("client.available() is true") : Serial.println("client.available() is false");
    int i = 0;
    while (client.available()) 
    {
      // Serial.println("--- client has data ----");
      if (readLength > (contentLength-1))
      {
        break;
      }
      char c = client.read();
      crc.update(c);
      if (!file.print(c))
      {
          Serial.println("error writing character to SPIFFS");
      }
      readLength++;
      if (readLength % (contentLength / 13) == 0) {
        printPercent(readLength, contentLength);
      }
      timeout = millis();
    }
  }
  file.close();
  Serial.print(F("-------------\r\n"));
  printPercent(readLength, contentLength);
  timeElapsed = millis() - timeElapsed;
  SerialMon.println();

  client.stop();
  SerialMon.println(F("Server disconnected"));

  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));

  float duration = float(timeElapsed) / 1000;
  SerialMon.println();
  SerialMon.print("Content-Length: ");   SerialMon.println(contentLength);
  SerialMon.print("Actually read:  ");   SerialMon.println(readLength);
  SerialMon.print("Calc. CRC32:    0x"); SerialMon.println(crc.finalize(), HEX);
  SerialMon.print("Known CRC32:    0x"); SerialMon.println(_fw_crc, HEX);
  SerialMon.print("Duration:       ");   SerialMon.print(duration); SerialMon.println("s");
  updateFromFS(_des_fw_name);
}

bool cmdProcess(char* _js_msg, JS_EVB_SUB* _output)
{
  // SerialMon.print(F("----------- Process MQTT msg ------------ \r\n"));
  // SerialMon.printf("_js_msg = %s \r\n",_js_msg);
  StaticJsonDocument<256> _doc;
  DeserializationError err = deserializeJson(_doc, _js_msg);
  // SerialMon.printf("deserial err = %s \r\n",err.c_str());
  if ((err != err.Ok) || (_doc.containsKey("rsp")))
  {
    return false;
  }
  // Check the incoming dev_id
  if (strcmp(_doc["dev_id"],DEV_NAME) != 0)
  {
    return false;
  }
  memset(_output->dev_id,0,sizeof(_output->dev_id));
  strcpy(_output->dev_id,_doc["dev_id"]);
  memset(_output->drive_mode,0,sizeof(_output->drive_mode));
  memset(_output->drive_mode,0,sizeof(_output->drive_mode));
  memset(_output->buzzer_state,0,sizeof(_output->buzzer_state));
  memset(_output->system,0,sizeof(_output->system));
  memset(_output->params,0,sizeof(_output-> params));

  // Check the incoming field of the current dev_id
  // Copy data into _output
  if (_doc.containsKey("drive_mode"))
  {
    memset(_output->drive_mode,0,sizeof(_output->drive_mode));
    strcpy(_output->drive_mode,_doc["drive_mode"]);
  }
  if (_doc.containsKey("buzzer_state"))
  {
    memset(_output->buzzer_state,0,sizeof(_output->buzzer_state));
    strcpy(_output->buzzer_state,_doc["buzzer_state"]);
  }

  if (_doc.containsKey("system"))
  {
    memset(_output->system,0,sizeof(_output->system));
    strcpy(_output->system,_doc["system"]);
    if (_doc.containsKey("params"))
    {
	    memset(_output->params,0,sizeof(_output-> params));
      strcpy(_output->params,_doc["params"]);
    }
  }
  // SerialMon.printf("For dev_id %s, drive_mode is %s, buzzer_state is %s, system is %s, params is %s\r\n",_output->dev_id,_output->drive_mode, _output->buzzer_state, _output->system, _output->params);
  // just show
  return true;
}

// Task Function
void cmdReceiveTask(void* parameters)
{
  char _msg_receive[70];
  String _rsp_rl_msg = "";
  String _drive_mode_str = "";
  String _system_str = "";
  String _params_str = "";
  String _buzzer_str = "";
  String _drive_mode_js = "";
  // disableCore0WDT(); // Close watchdog
  while (true)
  {
    memset(_msg_receive,0,sizeof(_msg_receive));
    if (xSemaphoreTake(sem_cmd_process,100) == pdTRUE) // 1000
    {
      if( xQueueReceive(queue_cmd, &(_msg_receive),100) == pdPASS)
      {
        // SerialMon.print(F("-------- cmdReceiveTask --------\r\n"));
        _drive_mode_str = ""; _system_str = ""; _params_str = ""; _buzzer_str = ""; _drive_mode_js = ""; _rsp_rl_msg = "";
        if (cmdProcess(_msg_receive, &js_sub_dev))
        {
          // Serial.println("MQTT CMD OK");
          _rsp_rl_msg = "";
          _drive_mode_str = String(js_sub_dev.drive_mode);
          // Serial.println("_drive_mode_str = "+_drive_mode_str);
          if (_drive_mode_str != "")
          {
            _rsp_rl_msg = "{\"dev_id\":\"" + String(DEV_NAME) + "\",\"drive_mode\":\"";
            if (strcmp(js_sub_dev.drive_mode,"on") == 0)
            {
              _rsp_rl_msg += "on\",\"rsp\":\"success\"}";
              init_drive_mode = _drive_mode_str;
              _drive_mode_js = "{\"drive_mode\":\""+init_drive_mode+"\"}";
              espWriteFile("/","drive_state.log","w", _drive_mode_js);
            }
            else if (strcmp(js_sub_dev.drive_mode,"off") == 0)
            {
              _rsp_rl_msg += "off\",\"rsp\":\"success\"}";
              init_drive_mode = _drive_mode_str;
              _drive_mode_js = "{\"drive_state\":\""+init_drive_mode+"\"}";
              espWriteFile("/","drive_state.log","w", _drive_mode_js);
            }
            else
            {
              _rsp_rl_msg += "unkown\",\"rsp\":\"success\"}";
            }
            xSemaphoreGive(sem_side_stand); // send signal to sideStandOffTask
          }

          // Process system field : update, shutdown, reboot
          _system_str = String(js_sub_dev.system);
          if (_system_str != "")
          {
            _rsp_rl_msg = "{\"dev_id\":\"" + String(DEV_NAME) + "\",\"system\":\"";
            if (_system_str == "update")
            {
              _params_str = String(js_sub_dev.params);
              if (_params_str.startsWith("0x"))
              {
                knownCRC32 = strtoul(js_sub_dev.params,NULL,16);
                xSemaphoreGive(sem_ota_update);
              }
              else
              {
                _rsp_rl_msg += "unknown\",\"rsp\":\"success\"}";
              }
            }
            else if (_system_str == "reboot")
            {
              _rsp_rl_msg += "reboot\",\"rsp\":\"success\"}";
              mqtt.publish(MQTT_CMD_TOPIC, "rebooting … wait for 60 second to wakeup again ;)");
              ESP.restart();
            }
            else
            {
              _rsp_rl_msg += "unkown\",\"rsp\":\"success\"}";
            }
          }
          // Check buzzer_state
          _buzzer_str = String(js_sub_dev.buzzer_state);
          if (_buzzer_str != "")
          {
            _rsp_rl_msg = "{\"dev_id\":\"" + String(DEV_NAME) + "\",\"buzzer_state\":\"";
            if (xSemaphoreTake(sem_pub_data,100) == pdTRUE)
            {
              if (_buzzer_str == "on")
              {
                js_pub_dev.bike.buzzer_state = _buzzer_str;
                _rsp_rl_msg += "on\",\"rsp\":\"success\"}";
              }
              else if (_buzzer_str == "off")
              {
                js_pub_dev.bike.buzzer_state = _buzzer_str;
                _rsp_rl_msg += "off\",\"rsp\":\"success\"}";
              }
              else
              {
                _rsp_rl_msg += "unkown\",\"rsp\":\"success\"}";
              }
              xSemaphoreGive(sem_pub_data);
            }
            xSemaphoreGive(sem_buzzer);
          }
          if (xSemaphoreTake(sem_at, 100))
          {
            if(!mqtt.publish(MQTT_CMD_TOPIC,_rsp_rl_msg.c_str())) SerialMon.print(F("Response report failed!!\r\n"));
            xSemaphoreGive(sem_at);
          }
        }
      }
    }
    // Move Side Statnd Task & Buzzer Task here ?
    
    // SerialMon.printf("Remain stack mqtt Process: %d \r\n",uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(800 / portTICK_PERIOD_MS);
  }
  ESP.restart();
  vTaskDelete(NULL);
}

void buzzerTask(void* parameters)
{
  bool buzzer_play_logic = 0;
  while(true)
  {
    if (xSemaphoreTake(sem_buzzer,0) == pdTRUE)
    {
      SerialMon.print(F("---------- buzzerPlayTask ----------\r\n"));
      if (js_pub_dev.bike.buzzer_state == "on")
      {
        buzzer_play_logic = 1;
      }
      else if (js_pub_dev.bike.buzzer_state == "off")
      {
        buzzer_play_logic = 0;
      }
    }
    if (buzzer_play_logic)
    {
      buzzerBeep(600,3); // buzzerBeep(500,3);
    }
    // SerialMon.printf("Remain stack mqtt Process: %d \r\n",uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
  ESP.restart();
  vTaskDelete(NULL);
}

void sideStandTask(void* parameters) // for off side stand event only!!
{
  bool side_stand_doing = 1;
  uint8_t side_stand_logic = 0;
  while(true)
  {
    // 1. Check signal from mqttProcessTask
    if (xSemaphoreTake(sem_side_stand,100) == pdTRUE) //(xSemaphoreTake(sem_side_stand,0) == pdTRUE)
    {
      // Serial.println("------ sideStandTask ---------");
      (init_drive_mode == "off")? side_stand_logic=1 : side_stand_logic=0;
      side_stand_doing = 0;
    }
    // 2. Try to close side stand if itsn't done.
    if (!side_stand_doing)
    {
      // Take sem_pub_data to check GPS Speed
      if (xSemaphoreTake(sem_pub_data,1000) == pdTRUE)
      {
        if ((uint8_t)abs(js_pub_dev.gps.speed.toDouble()) <= STOP_MOTION_SPEED_KMH)
        {
          relayTrigger(STAND_PIN,side_stand_logic);
          side_stand_doing = 1;
          SerialMon.println(init_drive_mode+" "+String(DEV_NAME) + " Done.");
        }
        xSemaphoreGive(sem_pub_data);
      }
    }
    // SerialMon.printf("Remain stack mqtt Process: %d \r\n",uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(600 / portTICK_PERIOD_MS);
  }
  ESP.restart();    
  vTaskDelete(NULL);
}

void lteTask(void* parameteres)
{
  NETWORK_INFO _network_lbs_info;
  memset(&_network_lbs_info,0,sizeof(_network_lbs_info));
  long int _ts = millis();
  long int _duration_ms = 0;
  long int _wait_ms = 0;
  String _debug = "";
  char _debug_ch[100];
  while(true)
  {
    _ts = millis();
    if ( xSemaphoreTake(sem_at, 5000) == pdTRUE )
    {
      SerialMon.print(F("------ lteTask -------- \r\n"));
      String check_net = sendData("AT+NETOPEN?",1000,0);
      if (check_net.startsWith("+NETOPEN: 0",check_net.indexOf('+')))
      {
        sendData("AT+CRESET",5000,0);
        delay(10000);
        connectLTE();
        SerialMon.print(F(" --------- Reconncet Done --------- \r\n"));
        digitalWrite(LED_LTE_PIN,LOW);
      }
      else
      {
        digitalWrite(LED_LTE_PIN,HIGH);
      }
      // LTE info update only CPSI data
      memset(&_network_lbs_info,0,sizeof(_network_lbs_info));
      readNetworkInfo(&_network_lbs_info);
      if (_network_lbs_info.lat == 0)
      {
        _network_lbs_info.lat = 0.00;
      }
      if (_network_lbs_info.lon == 0)
      {
        _network_lbs_info.lon = 0.00;
      }
      xSemaphoreGive(sem_at);
    }
    if (xSemaphoreTake(sem_pub_data, 2000) == pdTRUE)
    {
      memset(&js_pub_dev.network,0,sizeof(js_pub_dev.network));
      js_pub_dev.network = _network_lbs_info;
      xSemaphoreGive(sem_pub_data);
    }
    _duration_ms = millis()-_ts;
    _wait_ms = LTE_WAIT - _duration_ms;
    // SerialMon.printf("High water mark (words): %d \r\n",uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(_wait_ms/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void otaUpdateTask(void* parameters)
{
  char _update_received[100];
  memset(&_update_received,0,sizeof(_update_received));
  sprintf(_update_received,"{\"dev_id\":\"%s\",\"cmd\":\"update\",\"rsp\":\"in progress\"}",DEV_NAME);
  while(true)
  {
    // SerialMon.print(F("---------- otaUpdateTask ----------- \r\n"));
    if (xSemaphoreTake(sem_ota_update,1000))
    {
      SerialMon.print(F("---------- otaUpdateTask ----------- \r\n"));
      
      if (!mqtt.publish(MQTT_CMD_TOPIC,_update_received)) SerialMon.print(F("Update notify failed!!\r\n"));
      if (xSemaphoreTake(sem_at,1000))
      {
        // vTaskSuspend(buzzerPlayTask_handle);
        SerialMon.println("The received CRC32 is "+String(knownCRC32));
        if (!updateOTA(FW_SERVER, FW_PRT, FW_SRC_FNAME, FW_DEST_FNAME, knownCRC32))
        {
          SerialMon.println("OTA Update Failed!!\r\n"); 
          mqtt.publish(MQTT_CMD_TOPIC, "Update firmware failed!!");
        }
        xSemaphoreGive(sem_at);
      }
    }
    // SerialMon.printf("High water mark (words): %d \r\n",uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  ESP.restart();
  vTaskDelete(NULL);
}

void statusUpdateTask(void* parameters)
{
  JS_EVB_PUB _evb_status_struct;
  String js_report_str = "";
  unsigned long start;
  unsigned long _wait_ms = 0;
  uint16_t _adc_avg = 0;
  uint8_t _key_state = 0;
  while(true)
  {
    Serial.println("---------- statusUpdateTask ------------");
    // 1. Read GPS
    start = millis();
    do
    {
      while (gpsAT.available()) gps_module.encode(gpsAT.read()); // Send it to the encode function
    } while (millis() - start < 1000);
    if (gps_module.date.isUpdated())
    {
      _evb_status_struct.gps.date = String(gps_module.date.year()) + "/" + String(gps_module.date.month()) + "/" + String(gps_module.date.day());
      // SerialMon.println(String(_evb_status_struct.gps.date));
    }
    if (gps_module.time.isUpdated())
    {
      // memset(_evb_status_struct.gps.timen,0,sizeof(_evb_status_struct.gps.timen));
      // sprintf(_evb_status_struct.gps.timen,"%d:%d:%d",gps_module.time.hour(),gps_module.time.minute(),gps_module.time.second());
      _evb_status_struct.gps.timen = String(gps_module.time.hour()) + ":" + String(gps_module.time.minute()) + ":" + String(gps_module.time.second());
      // SerialMon.println(String(_evb_status_struct.gps.timen));
    }
    if (gps_module.location.isUpdated())
    {
      _evb_status_struct.gps.lat = String(gps_module.location.lat(),6);
      _evb_status_struct.gps.lon = String(gps_module.location.lng(),6);
      // SerialMon.println(String(_evb_status_struct.gps.lat,6) + ", " + String(_evb_status_struct.gps.lon,6));
      digitalWrite(LED_GPS_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED_GPS_PIN, LOW);
    }
    if (gps_module.altitude.isUpdated())
    {
      _evb_status_struct.gps.alt = String(gps_module.altitude.meters());
    }
    if (gps_module.speed.isUpdated())
    {
      _evb_status_struct.gps.speed = String(gps_module.speed.kmph());
    }
    if (gps_module.course.isUpdated())
    {
      _evb_status_struct.gps.course = String(gps_module.course.deg());
    }
    // 2. Read Batt
    adcAvg(BATT_ADC,&_adc_avg,1000);
    _evb_status_struct.batt.v_batt = readVoltBattery(_adc_avg);
    _evb_status_struct.batt.percent_batt = percentBattery(_evb_status_struct.batt.v_batt); 
    // 3. Read Key
    _key_state = digitalRead(KEY_ADC);
    _evb_status_struct.bike.key="";
    if (_key_state == 1)
    {
      _evb_status_struct.bike.key="on";
    }
    else
    {
      _evb_status_struct.bike.key="off";
    }
    // Serial.println(String(_evb_status_struct.gps.date)+", "+String(_evb_status_struct.gps.timen)+", "+String(_evb_status_struct.gps.lat,6) + ", " + String(_evb_status_struct.gps.lon,6)+", "+String(_evb_status_struct.batt.v_batt,1)+", "+String(_evb_status_struct.batt.percent_batt)+", "+String(_evb_status_struct.bike.key));
    if (xSemaphoreTake(sem_pub_data,100) == pdTRUE)
    {
      js_pub_dev.gps = _evb_status_struct.gps;
      js_pub_dev.batt = _evb_status_struct.batt;
      if ((js_pub_dev.bike.key == "off") && (_evb_status_struct.bike.key == "on"))
      {
        js_pub_dev.route_id = "";
        js_pub_dev.route_id = String(DEV_NAME)+"-"+String(esp_random()/10000);
      }
      js_pub_dev.bike.key = _evb_status_struct.bike.key;
      // Serial.println("js_pub_dev.route_id is "+js_pub_dev.route_id);
      js_report_str = "";
      // Serial.println(String(js_pub_dev.gps.date)+", "+String(js_pub_dev.gps.timen)+", "+String(js_pub_dev.gps.lat,6) + ", " + String(js_pub_dev.gps.lon,6)+", "+String(js_pub_dev.batt.v_batt,1)+", "+String(js_pub_dev.batt.percent_batt)+", "+String(js_pub_dev.bike.key));
      js_report_str = reportJsonUpdater(&js_pub_dev, init_drive_mode);
      Serial.println("Take sem pub OK");
      xSemaphoreGive(sem_pub_data);
    }
    else
    {
      Serial.println("Take sem pub failed!!, wait for next 100 ms");
    }
    // Serial.println("js_report_str = "+js_report_str);
    // Publish to MQTT
    if (xSemaphoreTake(sem_at,1000) == pdTRUE)
    {
      mqtt.publish(MQTT_RSP_TOPIC,js_report_str.c_str());
      js_report_str = "";
      Serial.println("Take sem at OK");
      xSemaphoreGive(sem_at);
    }
    else
    {
      Serial.println("Take sem at failed!!, wait for next 1 s");
    }
    // Save to log file?
    _wait_ms = REPORT_WAIT - (millis()-start);
    // Serial.println("_wait_ms = "+String(_wait_ms));
    xSemaphoreGive(sem_pub_alive); // Sensd signal to pubAliveTask to check the current task still alive.
    // SerialMon.printf("High water mark (words): %d \r\n",uxTaskGetStackHighWaterMark(NULL));
    vTaskDelay(_wait_ms/portTICK_PERIOD_MS);
  }
  ESP.restart();
  vTaskDelete(NULL);
}

void pubAliveTask(void* parameters)
{
  uint8_t _pub_die_count = 0;
  uint8_t _max_die_count = 10; // 26;
  while(true)
  {
    // Serial.println("------------ pubAliveTask -------------");
    if (xSemaphoreTake(sem_pub_alive,1000))
    {
      _pub_die_count = 0;
    }
    else
    {
      _pub_die_count++;
      // Serial.println("_pub_die_count is "+String(_pub_die_count));
    }
    if (_pub_die_count >= _max_die_count) 
    {
      Serial.println("statusUpdateTask are died, begin to restart ESP!!");
      ESP.restart();
    }
    vTaskDelay(7000/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void imuProcessTask(void* parameters)
{
  double dt;
  float fusion_pitch = 0;
  float fusion_roll = 0;
  float mea_roll = 0;
  float mea_pitch = 0;
  MOTION_DATA tmp_motion;
  while(true)
  {
    // SerialMon.print(F("----------- imuProcessTask ----------\r\n"));
    memset(&tmp_motion,0,sizeof(tmp_motion));
    aValue = myMPU6500.getAccRawValues();
    // Print raw Acc
    Serial.println(String(aValue.x,3)+" "+String(aValue.y,3)+" "+String(aValue.z,3));
    gyr = myMPU6500.getGyrValues();
    dt = (double)(micros() - ts_rollpitch) / 1000000;
    ts_rollpitch = micros();

    mea_roll = rollCalculation(aValue.x,aValue.y,aValue.z);
    mea_pitch = pitchCalculation(aValue.x,aValue.y,aValue.z);

    // estimate roll and pitch angle with gyro signal
    fusion_roll = kalmanRoll.getAngle(mea_roll, gyr.x, dt);
    fusion_pitch = kalmanPitch.getAngle(mea_pitch, gyr.y, dt);
    // Print roll
    // Serial.println(String(mea_roll,3)+" "+String(fusion_roll,3));
    // Print pitch
    // Serial.println(String(mea_pitch,3)+" "+String(fusion_pitch,3));
    // update state
    kalmanRoll.setAngle(fusion_roll);
    kalmanPitch.setAngle(fusion_pitch);

    tmp_motion.roll = fusion_roll;
    tmp_motion.pitch = fusion_pitch;
    vTaskDelay(100/portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void setup()
{
  // Initial Serial port and ADC
  SerialMon.begin(115200);
  SerialAT.begin(UART_BAUD,SERIAL_8N1,MODEM_RX,MODEM_TX);
  Serial.println("Hello");
  gpsAT.begin(NEO6M_BAUD);
  // IO initial
  // pinMode(KEY_ADC,INPUT);  // INPUT_PULLUP
  pinMode(KEY_ADC,INPUT_PULLUP);
  pinMode(BATT_ADC,INPUT);
  pinMode(STAND_PIN,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(LED_GPS_PIN,OUTPUT);
  pinMode(LED_LTE_PIN,OUTPUT);
  // Close ESP Wifi
  esp_wifi_stop();
  // Read device name & drive_mode
  // memset(DEV_NAME,0,sizeof(DEV_NAME));
  espReadConfig("/","dev_config.conf",DEV_NAME);
  memset(MQTT_CMD_TOPIC,0,sizeof(MQTT_CMD_TOPIC));
  sprintf(MQTT_CMD_TOPIC,"/%s",DEV_NAME);
  init_drive_mode = espReadDriveState("/","drive_state.log"); 
  Serial.println("init_drive_mode is "+init_drive_mode);
  sprintf(js_pub_dev.drivemode,"%s",init_drive_mode.c_str());
  (init_drive_mode == "on")? relayTrigger(STAND_PIN,0) : relayTrigger(STAND_PIN,1);
  js_pub_dev.route_id=String(DEV_NAME)+"-"+String(esp_random()/10000);
  // LTE Connection
  modemOn();
  sendData("ATE0",100,0);
  connectLTE();
  // FreeRTOS Parameters
  sem_at = xSemaphoreCreateMutex();
  sem_pub_data = xSemaphoreCreateMutex();
  sem_cmd_process = xSemaphoreCreateBinary();
  sem_side_stand = xSemaphoreCreateBinary();
  sem_buzzer = xSemaphoreCreateBinary();
  sem_ota_update = xSemaphoreCreateBinary();
  sem_pub_alive = xSemaphoreCreateBinary();
  queue_cmd = xQueueCreate(2, 70*sizeof(char));

  // MQTT setup
  mqtt.setServer(MQTT_SVR, MQTT_PRT);
  mqtt.setCallback(callback);
  mqtt.setBufferSize(900); //https://github.com/knolleary/pubsubclient/issues/501
  memset(MQTT_NAME,0,sizeof(MQTT_NAME));
  sprintf(MQTT_NAME,"EVB_Proj%d",esp_random());
  SerialMon.println("MQTT_NAME is "+String(MQTT_NAME));
  while (!mqtt.connected())
  {
    SerialMon.println("Connecting to MQTT...");
    if (mqtt.connect(MQTT_NAME,MQTT_USERNAME,MQTT_PASS))
    {
      SerialMon.println("MQTT connected");
      SerialMon.printf("MQTT_CMD_TOPIC is %s\r\n", MQTT_CMD_TOPIC);
      mqtt.subscribe(MQTT_CMD_TOPIC, MQTT_QOS) ? SerialMon.println("Subscribe OK\r\n") : SerialMon.println("Subscribe Failed!!\r\n");
    } 
    else 
    {
      SerialMon.println("MQTT connect failed with state "+String(mqtt.state()));
      mqtt.disconnect();
      mqtt_failed_connect++;
      if (mqtt_failed_connect > 5) ESP.restart();
      delay(2000);
    }
  }
  delay(2000);
  char greeting[100];
  sprintf(greeting,"{\"msg\":\"device %s is wake up, fw-version 9!!\"}",DEV_NAME);
  mqtt.publish(MQTT_RSP_TOPIC,greeting)? SerialMon.printf("Greeting OK\r\n") : SerialMon.printf("Greeting Failed!!\r\n");

  // Task Create
  xTaskCreatePinnedToCore(lteTask,
                          "Check internet",
                          2000,
                          NULL,
                          1,
                          &lteTask_handle,
                          1);
  xTaskCreatePinnedToCore(cmdReceiveTask,
                          "cmdReceiveTask",
                          2500, // 3800
                          NULL,
                          1,
                          &cmdReceiveTask_handle,
                          0);
  xTaskCreatePinnedToCore(sideStandTask,
                          "sideStandTask",
                          2200,
                          NULL,
                          1,
                          &sideStandTask_handle,
                          1);
  xTaskCreatePinnedToCore(buzzerTask,
                          "buzzerTask",
                          2000,
                          NULL,
                          1,
                          &buzzerTask_handle,
                          1);
  xTaskCreatePinnedToCore(statusUpdateTask,
                          "statusUpdateTask",
                          4000, // 3800
                          NULL,
                          1,
                          &statusUpdateTask_handle,
                          1);
  // if (imuInitial() == 1)
  // {
  //   xTaskCreatePinnedToCore(imuProcessTask,
  //                           "imuProcessTask",
  //                           2500, NULL
  //                           , 1
  //                           , &imuProcessTask_handle
  //                           , 1);
  // }
  xTaskCreatePinnedToCore(otaUpdateTask,
                          "otaUpdateTask",
                          9000, // 3800
                          NULL,
                          1,
                          &otaUpdateTask_handle,
                          0);
  xTaskCreatePinnedToCore(pubAliveTask,
                          "pubAliveTask",
                          2000,
                          NULL,
                          1,
                          &pubAliveTask_handle,
                          1);
}

void loop()
{
  if (xSemaphoreTake(sem_at,0))
  {
    if (!mqtt.connected()) mqttReconnect();
    mqtt.loop();
    xSemaphoreGive(sem_at);
  }
  // if (xSemaphoreTake(sem_at,0))
  // {
  //   if (!mqtt.connected())
  //   {
  //     for (uint8_t _try_idx=3; _try_idx>0; _try_idx--)
  //     {
  //       mqttReconnect();
  //       if (mqtt.connected()) break;
  //     }
  //   }
  //   xSemaphoreGive(sem_at);
  // }
  // mqtt.loop();
  vTaskDelay(100/portTICK_PERIOD_MS);
}