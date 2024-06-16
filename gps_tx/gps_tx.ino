#include <Adafruit_GPS.h>
#include <SPI.h>
#include <RH_RF95.h>

// GPS Module setup
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
#define GPSECHO false

// LoRa Module setup
#define RFM95_CS 10
#define RFM95_RST 11
#define RFM95_INT 9
#define RF95_FREQ 434.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define PSK "fredraslab" // Pre-shared key for XOR encryption

// Function to encrypt or decrypt data
void xorEncrypt(uint8_t *data, size_t len, const char *key)
{
  size_t keyLen = strlen(key);
  for (size_t i = 0; i < len; i++)
  {
    data[i] ^= key[i % keyLen];
  }
}

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  // while (!Serial)
  //   ;  // Wait for the serial port to be ready

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);

  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    while (1)
      ;
  }
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  rf95.setTxPower(20, false);
  Serial.println("GPS & LoRa modules initialized.");
}

void loop()
{
  static uint32_t lastTime = 0;
  char c = GPS.read();
  if (GPSECHO)
  {
    if (c)
      Serial.print(c);
  }

  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  if (millis() - lastTime < 500)
    return;

  // if (GPS.fix) {
  // Compressed data format: lat (4 bytes), lon (4 bytes), alt (2 bytes), sats (1 byte), time (4 bytes)
  static uint8_t buffer[15];
  int32_t lat = GPS.latitude_fixed;                                  // Convert to float and reduce precision
  int32_t lon = GPS.longitude_fixed;                                 // Convert to float and reduce precision
  int16_t alt = (int16_t)GPS.altitude;                               // Convert altitude to integer
  uint8_t sats = GPS.satellites;                                     // Satellites are likely a small number
  uint32_t time = GPS.hour * 10000 + GPS.minute * 100 + GPS.seconds; // Compact time into HHMMSS format

  memcpy(buffer, &lat, 4);
  memcpy(buffer + 4, &lon, 4);
  memcpy(buffer + 8, &alt, 2);
  memcpy(buffer + 10, &sats, 1);
  memcpy(buffer + 11, &time, 4);

  // Serial.println("Sending compressed GPS data...");
  xorEncrypt(buffer, sizeof(buffer), PSK);
  rf95.send(buffer, sizeof(buffer));
  rf95.waitPacketSent();

  // static uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  // uint8_t len = sizeof(buf);
  // if (rf95.waitAvailableTimeout(3000))
  // {
  //   if (rf95.recv(buf, &len))
  //   {
  //     Serial.print("Got ack: ");
  //     Serial.println((char *)buf);
  //   }
  //   else
  //   {
  //     // Serial.println("Ack receive failed, resend...");
  //     return;
  //   }
  // }
  // else
  // {
  //   // Serial.println("No ack received, resend...");
  //   return;
  // }
  lastTime = millis();
  // }
}
