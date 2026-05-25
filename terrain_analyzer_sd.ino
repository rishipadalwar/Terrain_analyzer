
#if defined(__AVR_ATmega2560__)          
  #define BOARD_MEGA
#elif defined(__AVR_ATmega328P__)       
  #define BOARD_UNO
#else
  #define BOARD_UNO                      
#endif


#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>

#ifdef BOARD_UNO
  #include <SoftwareSerial.h>
  SoftwareSerial gpsSerial(4, 3);        
  SoftwareSerial lidarSerial(6, 5);      
  #define GPS_SERIAL   gpsSerial
  #define LIDAR_SERIAL lidarSerial
#else
  #define GPS_SERIAL   Serial1           
  #define LIDAR_SERIAL Serial2          
#endif


#ifdef BOARD_MEGA
  #define SD_CS_PIN   53
#else
  #define SD_CS_PIN   10
#endif

#define LED_GPS   7
#define LED_SD    8
#define LED_ERR   9


#define SAMPLE_INTERVAL_MS   500    
#define LOG_FILENAME         "TERRAIN.CSV"
#define MAX_LIDAR_DISTANCE   1200   
#define MIN_LIDAR_STRENGTH   100    


TinyGPSPlus gps;


File logFile;
bool sdReady = false;


struct TerrainPoint {
  uint32_t timestamp_ms;
  double   latitude;
  double   longitude;
  double   altitude_m;
  uint16_t distance_cm;
  int16_t  signal_strength;
  int8_t   temperature_c;   
  float    slope_deg;
  bool     gps_valid;
  bool     lidar_valid;
};

TerrainPoint cur, prev;
bool hasPrev    = false;
uint32_t lastSampleMs = 0;
uint32_t pointIndex   = 0;


bool readLuna(uint16_t &dist_cm, int16_t &strength, int8_t &temp_c) {
  uint32_t timeout = millis() + 150;
  while (millis() < timeout) {
    if (!LIDAR_SERIAL.available()) continue;
    if (LIDAR_SERIAL.read() != 0x59) continue;

    
    uint32_t t2 = millis() + 20;
    while (!LIDAR_SERIAL.available() && millis() < t2);
    if (!LIDAR_SERIAL.available()) continue;
    if (LIDAR_SERIAL.read() != 0x59) continue;

    
    uint8_t buf[7];
    for (uint8_t i = 0; i < 7; ) {
      if (millis() > timeout) return false;
      if (LIDAR_SERIAL.available()) buf[i++] = LIDAR_SERIAL.read();
    }

    
    uint8_t ck = 0x59 + 0x59;
    for (uint8_t i = 0; i < 6; i++) ck += buf[i];
    if (ck != buf[6]) continue;   // bad frame, keep hunting

    dist_cm  = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    strength = (int16_t)buf[2]  | ((int16_t)buf[3]  << 8);
    int16_t rawTemp = (int16_t)buf[4] | ((int16_t)buf[5] << 8);
    temp_c   = (int8_t)(rawTemp / 8 - 256);

   
    if (dist_cm > MAX_LIDAR_DISTANCE) return false;
    if (strength < MIN_LIDAR_STRENGTH) return false;
    return true;
  }
  return false;
}


float calcSlopeDeg(const TerrainPoint &a, const TerrainPoint &b) {
  if (!a.gps_valid || !b.gps_valid)     return 0.0f;
  if (!a.lidar_valid || !b.lidar_valid) return 0.0f;

  double lat1 = radians(a.latitude), lat2 = radians(b.latitude);
  double dLat = lat2 - lat1;
  double dLon = radians(b.longitude - a.longitude);
  double h    = sin(dLat/2)*sin(dLat/2) +
                cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);
  double horizM = 6371000.0 * 2.0 * atan2(sqrt(h), sqrt(1.0-h));

  if (horizM < 0.05) return 0.0f;

  float dH_m = ((int32_t)b.distance_cm - (int32_t)a.distance_cm) / 100.0f;
  return degrees(atan2(fabs(dH_m), horizM));
}


void sdWriteHeader() {
  logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  if (!logFile) return;
  logFile.println(F("idx,timestamp_ms,latitude,longitude,alt_gps_m,"
                    "lidar_dist_cm,lidar_strength,lidar_temp_c,slope_deg,"
                    "gps_valid,lidar_valid"));
  logFile.flush();
  logFile.close();
}

void sdLogPoint(const TerrainPoint &p) {
  logFile = SD.open(LOG_FILENAME, FILE_WRITE);
  if (!logFile) {
    sdReady = false;
    digitalWrite(LED_ERR, HIGH);
    Serial.println(F("[SD] Open failed!"));
    return;
  }

  logFile.print(pointIndex);               logFile.print(',');
  logFile.print(p.timestamp_ms);           logFile.print(',');

  if (p.gps_valid) {
    logFile.print(p.latitude,  6);         logFile.print(',');
    logFile.print(p.longitude, 6);         logFile.print(',');
    logFile.print(p.altitude_m, 2);
  } else {
    logFile.print(F(",,"));
  }
  logFile.print(',');

  if (p.lidar_valid) {
    logFile.print(p.distance_cm);          logFile.print(',');
    logFile.print(p.signal_strength);      logFile.print(',');
    logFile.print(p.temperature_c);
  } else {
    logFile.print(F(",,"));
  }
  logFile.print(',');
  logFile.print(p.slope_deg, 2);           logFile.print(',');
  logFile.print(p.gps_valid   ? 1 : 0);   logFile.print(',');
  logFile.println(p.lidar_valid ? 1 : 0);

  logFile.flush();
  logFile.close();

  
  digitalWrite(LED_SD, HIGH);
  delay(30);
  digitalWrite(LED_SD, LOW);
}


void printPoint(const TerrainPoint &p) {
  Serial.print(F("[#"));
  Serial.print(pointIndex);
  Serial.print(F("] t="));
  Serial.print(p.timestamp_ms);
  Serial.print(F("ms | GPS: "));

  if (p.gps_valid) {
    Serial.print(p.latitude,  6); Serial.print(F(", "));
    Serial.print(p.longitude, 6); Serial.print(F(" | Alt: "));
    Serial.print(p.altitude_m, 1); Serial.print(F("m"));
  } else {
    Serial.print(F("waiting..."));
  }

  Serial.print(F(" | LiDAR: "));
  if (p.lidar_valid) {
    Serial.print(p.distance_cm);  Serial.print(F("cm  Sig:"));
    Serial.print(p.signal_strength); Serial.print(F("  T:"));
    Serial.print(p.temperature_c); Serial.print(F("°C"));
  } else {
    Serial.print(F("no data"));
  }

  if (p.slope_deg > 0.5f) {
    Serial.print(F(" | Slope: "));
    Serial.print(p.slope_deg, 1);
    Serial.print(F("°"));
  }

  Serial.println();
}


void setup() {
  Serial.begin(115200);

  
  pinMode(LED_GPS, OUTPUT);
  pinMode(LED_SD,  OUTPUT);
  pinMode(LED_ERR, OUTPUT);

  
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(LED_GPS, HIGH); digitalWrite(LED_SD, HIGH); digitalWrite(LED_ERR, HIGH);
    delay(150);
    digitalWrite(LED_GPS, LOW);  digitalWrite(LED_SD, LOW);  digitalWrite(LED_ERR, LOW);
    delay(150);
  }

  
  GPS_SERIAL.begin(9600);
  LIDAR_SERIAL.begin(115200);

 

 
  Serial.print(F("[SD] Initialising... "));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("FAILED — check wiring & card format (FAT32)"));
    digitalWrite(LED_ERR, HIGH);
    sdReady = false;
  } else {
    Serial.println(F("OK"));
    sdReady = true;
   
    if (!SD.exists(LOG_FILENAME)) {
      sdWriteHeader();
      Serial.println(F("[SD] New file created: " LOG_FILENAME));
    } else {
      Serial.println(F("[SD] Appending to existing: " LOG_FILENAME));
    }
    digitalWrite(LED_SD, HIGH); delay(500); digitalWrite(LED_SD, LOW);
  }

  Serial.println(F("[GPS] Waiting for fix..."));
  Serial.println(F("[LOG] idx,timestamp_ms,lat,lon,alt_m,dist_cm,strength,temp_c,slope_deg"));
}


void loop() {

#ifdef BOARD_UNO
  
  while (GPS_SERIAL.available()) gps.encode(GPS_SERIAL.read());
#else
  
  while (GPS_SERIAL.available()) gps.encode(GPS_SERIAL.read());
#endif

  if (millis() - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = millis();

  
  cur.timestamp_ms = millis();
  cur.gps_valid    = gps.location.isValid() && gps.location.age() < 2000;
  if (cur.gps_valid) {
    cur.latitude   = gps.location.lat();
    cur.longitude  = gps.location.lng();
    cur.altitude_m = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    digitalWrite(LED_GPS, HIGH);
  } else {
    cur.latitude = cur.longitude = cur.altitude_m = 0.0;
    digitalWrite(LED_GPS, LOW);
  }

  
#ifdef BOARD_UNO
  
  GPS_SERIAL.end();
  LIDAR_SERIAL.begin(115200);
  delay(5);
#endif

  cur.lidar_valid = readLuna(cur.distance_cm,
                              cur.signal_strength,
                              cur.temperature_c);
  if (!cur.lidar_valid) {
    cur.distance_cm    = 0;
    cur.signal_strength = 0;
    cur.temperature_c  = 0;
  }

#ifdef BOARD_UNO
  LIDAR_SERIAL.end();
  GPS_SERIAL.begin(9600);
#endif

  
  cur.slope_deg = hasPrev ? calcSlopeDeg(prev, cur) : 0.0f;

  
  printPoint(cur);
  if (sdReady) sdLogPoint(cur);

  
  prev    = cur;
  hasPrev = true;
  pointIndex++;
}
