// For PlatformIO we need to include the Arduino framework
#include <Arduino.h>
#include "esp_now.h"
#include "WiFi.h"
#include "BluetoothSerial.h"
#include <esp_wifi.h>
#include <future>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define TASK_SCHEDULER // enable adv task scheduler. Small memory save to be worth removing

// How large should the RX buffer for the GPS port - more than 512 creates issues
#define UART_BUFFER_SIZE_RX 256

// GPS port on UART2
#define gpsPort Serial2
// How much time before autobauding times out
#define GPS_UART_TIMEOUT 10000UL
#define GPS_STANDARD_BAUD_RATE 115200

// Define the serial monitor port
#define SerialMonitor Serial
#define LOG_BAUD_RATE 115200

uint8_t newBoardAddress[] = {0x94, 0x3C, 0xC6, 0x08, 0x35, 0x60};

#if !(defined(ESP32))
#error This code is designed to run only on the ESP32 board
#endif

#define RX2 RX // Standard label Rx2 on board
#define TX2 TX // Standard label Tx2 on board

int max_buffer = 0;
bool gps_powersave = false;

BluetoothSerial SerialBT;
bool bt_deviceConnected = false;
bool bt_spp_stop();
void bt_spp_start();
void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);

//for LED status and OTA updates
#ifdef TASK_SCHEDULER
#include <TaskScheduler.h>
// Scheduler for periodic tasks
Scheduler ts;
#endif

#define MAX_AP_NAME_SIZE 20
char blspp_device_id[MAX_AP_NAME_SIZE];

char ap_ssid[MAX_AP_NAME_SIZE];
uint16_t chip;
#define KARTGPS_AP "KartGPS_PCB"

esp_now_peer_info_t peerInfo;


/********************************
 * 
 * Declarations of prototypes required by VS Code or PlatformIO
 * 
********************************/
void gps_initialize_settings();
void restart_after_sleep();

void poweroff() {
  bt_spp_stop();

  // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html
  // rtc_gpio_isolate(GPIO_NUM_12);
  esp_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}
/********************************

   GPS Settings
   These sentences are used to configure a uBlox series 8 device

 * ******************************/
// set rate of GPS to 10Hz
const char UBLOX_INIT_10HZ[] PROGMEM = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};

// GLL OFF
const char UBLOX_GxGLL_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};

// VTG OFF
const char UBLOX_GxVTG_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46};

// Disable GxGSA
const char UBLOX_GxGSA_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31};

// Disable GxGSV
const char UBLOX_GxGSV_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38};

// use GP as Main talker and GSV Talker ID - this is needed for TrackAddict
const char UBLOX_INIT_MAINTALKER_GP[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x00, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x78};

// use GP as Main talker and GSV Talker ID and restrict to GPS SVs ony - this is needed for RaceChrono
const char UBLOX_INIT_MAINTALKER_GP_GPSONLY[] PROGMEM = {0xB5, 0x62, 0x06, 0x17, 0x14, 0x00, 0x10, 0x41, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88, 0xB8};

// set gps port BAUD rate
const char UBLOX_BAUD_57600[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xE1, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA, 0xA9};
const char UBLOX_BAUD_38400[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x70};
const char UBLOX_BAUD_115200[] PROGMEM = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x03, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E};

// power saving
// enable power saving mode for 1800 or 3600 seconds
// UBX-RXM-PMREQ request
const char UBLOX_PWR_OFF[] PROGMEM = { 0xB5, 0x62, 0x02, 0x41, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x5D, 0x4B};

//, 0xrestart UBX-CFG-RST with controlled GNSS only software, hotstart (<4 hrs) so that ephemeris still valid
const char UBLOX_WARMSTART[] PROGMEM = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68};

// Display precision in some apps
const char UBLOX_GxGBS_OFF[] PROGMEM = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x62};

// Poll GPGSA
const char UBLOX_GPGSA_POLL[] PROGMEM = "$EIGPQ,GSA*33\r\n";
// Poll GPGSV
const char UBLOX_GPGSV_POLL[] PROGMEM = "$EIGPQ,GSV*24\r\n";
bool gsaorgsv_turn = true;

void push_gps_message(const char message[], int message_size = 0)
{
  if (gps_powersave)
  {
    log_d("Disable powermode");
    gpsPort.end();
    delay(250);
    // assumes stored_preferences global
    gps_powersave = false;
    gps_initialize_settings();
  }
  for (int i = 0; i < message_size; i++)
  {
    gpsPort.write(message[i]);
  }
}

#ifdef TASK_SCHEDULER
void poll_GSA_GSV_info()
{
  if (gsaorgsv_turn)
  {
      // RaceChrono only understand GPGSA
      gpsPort.write(UBLOX_GPGSA_POLL);
  }
  else
  {
      // RaceChrono only understand GPGSV
      gpsPort.write(UBLOX_GPGSV_POLL);

  }
  gsaorgsv_turn = !gsaorgsv_turn;
}

Task tpoll_GSA_GSV_info(0, TASK_FOREVER, poll_GSA_GSV_info, &ts, false);
void control_poll_GSA_GSV(int frequency)
{
  if (frequency > 0)
  {
    push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
    push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
    tpoll_GSA_GSV_info.setInterval(frequency * 500);
    tpoll_GSA_GSV_info.enable();
  }
  else
  {
    tpoll_GSA_GSV_info.disable();
  }
}

// handle restart when config is lost
Task trestart_after_sleep(0, 1, restart_after_sleep, &ts, false);
void restart_after_sleep()
{
  gpsPort.end();
  delay(250);
  // assumes 'stored_preferences' is global
  gps_initialize_settings();
  trestart_after_sleep.disable();
}
#endif

void switch_baudrate(uint32_t newbaudrate)
{
  // stored_preferences assumed global
  log_d("Send UBX-CFG for rate %d", newbaudrate);
  switch (newbaudrate)
  {
  case 38400:
    push_gps_message(UBLOX_BAUD_38400, sizeof(UBLOX_BAUD_38400));
    break;
  case 57600:
    push_gps_message(UBLOX_BAUD_57600, sizeof(UBLOX_BAUD_57600));
    break;
  case 115200:
    push_gps_message(UBLOX_BAUD_115200, sizeof(UBLOX_BAUD_115200));
    break;
  default:
    push_gps_message(UBLOX_BAUD_38400, sizeof(UBLOX_BAUD_38400));
    break;
  }
  log_d("Flush UART");
  gpsPort.flush();
  delay(500);
  log_d("Changing baud on UART2 from %u to %u , pins RX2=%d TX2=%d", gpsPort.baudRate(), newbaudrate, RX2, TX2);
  gpsPort.updateBaudRate(newbaudrate);
}


void gps_enable_common()
{
  push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
  push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
  push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));
  push_gps_message(UBLOX_GxVTG_OFF, sizeof(UBLOX_GxVTG_OFF));
  push_gps_message(UBLOX_GxGLL_OFF, sizeof(UBLOX_GxGLL_OFF));
  push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(0);
#endif
}


void gps_enable_racechrono()
{
  log_i("Setting GPS to specific RaceChrono needs");
  gps_enable_common();
  push_gps_message(UBLOX_INIT_MAINTALKER_GP_GPSONLY, sizeof(UBLOX_INIT_MAINTALKER_GP_GPSONLY));
#ifdef TASK_SCHEDULER
  control_poll_GSA_GSV(5);
#endif
}  

void handle_racechrono_android()
{
  // /racechrono/android
  log_i("Setting optimal configuration for RaceChrono on Android: 10Hz, GSA+GSV+GBS Off, BT-SPP");
  gps_enable_racechrono();
  bt_spp_start();
}


void handle_deepsleep_execute()
{
  log_i("Powering off");
  control_poll_GSA_GSV(0);
  // https://portal.u-blox.com/s/question/0D52p00008HKCQxCAP/shutting-down-neom8-indefinitely
  push_gps_message(UBLOX_PWR_OFF, sizeof(UBLOX_PWR_OFF));
  delay(1000);
  poweroff();
}

/*********************************************************************

  Bluetooth Classic

*******************************************************************/
void bt_spp_start()
{
  SerialBT.begin(KARTGPS_AP); //Bluetooth device name
  SerialBT.register_callback(bt_callback);
  log_i("Started BT-SPP port");
}

void bt_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    log_d("BT-SPP Client Connected");
    bt_deviceConnected = true;
  }

  if (event == ESP_SPP_CLOSE_EVT)
  {
    log_d("BT-SPP Client closed connection");
    bt_deviceConnected = false;
  }
}

bool bt_spp_stop()
{
  // Prior to version 1.1, this was
  SerialBT.end();
}

/*********************************************************************

  SETUP

*******************************************************************/
void gps_initialize_settings()
{
  // GPS Connection
  log_d("Send a ping to start the GPS if it was powersaved");
  // BAUD matters almost nothing, we just need some data on UART
  gpsPort.begin(GPS_STANDARD_BAUD_RATE, SERIAL_8N1, RX2, TX2);

  push_gps_message(UBLOX_WARMSTART, sizeof(UBLOX_WARMSTART));
  gpsPort.flush();
  delay(250);
  gpsPort.end();
  delay(250);
  // if preferences have a value <> than the one stored in the GPS, connect+switch
  log_d("Start UART connection on RX pin %d TX pin %d and autobaudrate", RX2, TX2);
  gpsPort.begin(0, SERIAL_8N1, RX2, TX2, false, GPS_UART_TIMEOUT);
  if (gpsPort.baudRate() > 0)
  {
    log_d("Connected with autobaudrate at %u on RX pin %d TX pin %d ", gpsPort.baudRate(), RX2, TX2);
  }
  else
  {
    log_e("Can't auto find BAUD rate on RX pin %d TX pin %d , forcing %u", RX2, TX2, GPS_STANDARD_BAUD_RATE);
    // TODO: enable pulsing error on the LED to signal the user that something is bad
    gpsPort.begin(GPS_STANDARD_BAUD_RATE, SERIAL_8N1, RX2, TX2);
  }

  if (gpsPort.baudRate() != GPS_STANDARD_BAUD_RATE)
  {
    log_i("Re-Connecting to GPS at updated %u", GPS_STANDARD_BAUD_RATE);
    switch_baudrate(GPS_STANDARD_BAUD_RATE);
  }

  gpsPort.setRxBufferSize(UART_BUFFER_SIZE_RX);
  delay(50);

    push_gps_message(UBLOX_GxGSV_OFF, sizeof(UBLOX_GxGSV_OFF));
    push_gps_message(UBLOX_GxGSA_OFF, sizeof(UBLOX_GxGSA_OFF));
    push_gps_message(UBLOX_GxGBS_OFF, sizeof(UBLOX_GxGBS_OFF));

    gps_enable_racechrono();

    push_gps_message(UBLOX_INIT_10HZ, sizeof(UBLOX_INIT_10HZ));
}

void onESPNowDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\nLast packer send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}


bool espNow_Connected = false;

void setupESPNow() 
{
  WiFi.mode(WIFI_MODE_STA);
  if(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR) == ESP_OK) {
    Serial.println("WiFi Long range");
  } else {
    Serial.println("WiFi failed");
  }

  Serial.println("Wifi MAC:\n");
  Serial.println(WiFi.macAddress());
  Serial.println("\n");

  if(esp_now_init() != ESP_OK) {
    Serial.println("Error! Could not initialise ESP-NOW");
    return;
  }

  esp_now_register_send_cb(onESPNowDataSent);

  memcpy(peerInfo.peer_addr, newBoardAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error! Could not connect to ESP-NOW peer");
    return;
  }

  espNow_Connected = true;
}

bool newGPSData = false;
uint8_t *gpsBuf;
size_t gpsLen;

void writeESPNow() {
  // Write to esp-now
  if(espNow_Connected) {
    esp_err_t result = esp_now_send(newBoardAddress, gpsBuf, gpsLen);

    if(result == ESP_OK) {
      Serial.println("ESP-NOW sent");
    } else {
      Serial.println("ESP-NOW failed");
    }
  }

  free(gpsBuf);
  newGPSData = false;
}

// //create an RF24 object
#define CE_PIN  22
#define CSN_PIN 21

const byte sendToAddress[5] = {'R', 'x', 'A', 'B', 'A'};

RF24 radio(CE_PIN, CSN_PIN);
bool radioOn = false;

void setupNRF24L01() {
  if(radio.begin()) {
      radioOn = true;

      radio.setDataRate( RF24_250KBPS );
      radio.disableAckPayload();
      radio.openWritingPipe(sendToAddress);
      radio.printDetails();
        
      //Set module as transmitter
      radio.stopListening();

      Serial.println("Radio started");
   } else {
      radioOn = false;
      Serial.println("Radio failed");
   }
  
}

void writeNRF24L01() {
  //Send message to receiver
  Serial.print("Radio: ");
  Serial.println(radioOn);

  char gpsData[32];

  memcpy(gpsData, gpsBuf, sizeof(gpsData));

  if(radioOn) {
    bool result = radio.write(gpsBuf, sizeof(gpsBuf));

    Serial.print((char*)gpsBuf);
    Serial.print(": ");
    Serial.println(sizeof(gpsBuf));

    if(result == ESP_OK) {
      Serial.println("nRF24l01+ sent");
    } else {
      Serial.println("nRF24l01+ failed");
    }
  }

  free(gpsBuf);
  newGPSData = false;
}


TaskHandle_t Task1;

void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
      if(newGPSData) {
        // writeESPNow();
        writeNRF24L01();
      }

      delay(200);
  }
}

void setup()
{
  SerialMonitor.begin(LOG_BAUD_RATE); // Initialize the serial monitor
  delay(200);
  // setupESPNow();
  setupNRF24L01();
  log_d("ESP32 SDK: %s", ESP.getSdkVersion());

    pinMode(2, OUTPUT);
  // Generate device name
  chip = (uint16_t)((uint64_t)ESP.getEfuseMac() >> 32);
  sprintf(ap_ssid, "%s-%04X", KARTGPS_AP, chip);
  sprintf(blspp_device_id, "%s-%04X", KARTGPS_AP, chip);

  bt_spp_start();

  gps_initialize_settings();

  log_d("Total heap: %d", ESP.getHeapSize());
  log_d("Free heap: %d", ESP.getFreeHeap());
  log_d("Total PSRAM: %d", ESP.getPsramSize());
  log_d("Free PSRAM: %d", ESP.getFreePsram());

  // Start core 2
  if(radioOn) {
    xTaskCreatePinnedToCore(
      Task1code,   /* Task function. */
      "Task1",     /* name of task. */
      10000,       /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &Task1,      /* Task handle to keep track of created task */
      0
    );          /* pin task to core 0 */
  }
}

void loop()
{
  digitalWrite(2, HIGH);
  // check UART for data
  if (gpsPort.available())
  {
    size_t len = gpsPort.available();
    uint8_t sbuf[len];
    gpsPort.readBytes(sbuf, len);
    // log max buffer size
    if (len > max_buffer)
    {
      max_buffer = len;
      log_w("New max buffer: %d", max_buffer);
    }

    if(!newGPSData) {
        gpsBuf = (uint8_t *) malloc(len * sizeof(uint8_t));
        memcpy(gpsBuf, sbuf, len);
        gpsLen = len;
        newGPSData = true;
    }

    if (bt_deviceConnected)
    {
      // we have BT-SPP active
      SerialBT.write(sbuf, len);
      Serial.println("Wrote to bluetooth");
    }
  }

#ifdef TASK_SCHEDULER
  // run all periodic tasks
  ts.execute();
#endif // #ifdef TASK_SCHEDULER
}