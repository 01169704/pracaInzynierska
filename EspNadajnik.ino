#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

// Ekran
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define PI 3.14159265359
#define AIN0D 12200
#define AIN1D 12050
#define AIN2D 12320
#define AIN3D 12540
#define ROZDZIELCZOSC 4000

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

// Zdefiniowanie joysticka
const byte Switchpin = 32;

int16_t adc0, adc1, adc2, adc3;

int chod = 0;
 
// MAC Address of responder - edit as required
uint8_t broadcastAddress[] = {0xec, 0x64, 0xc9, 0x86, 0xd0, 0x20};
 
// Define a data structure
typedef struct struct_message {
  float katRuchu;
  int wysokosc;
  int obrot;
  int ruch;
} struct_message;
 
// Create a structured object
struct_message myData;

 
// Peer info
esp_now_peer_info_t peerInfo;
 
// Callback function called when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.print("katRuchu: ");
  Serial.println(myData.katRuchu);
  Serial.print("Wysokosc: ");
  Serial.println(myData.wysokosc);
  Serial.print("Obrot: ");
  Serial.println(myData.obrot);
  Serial.print("Ruch: ");
  Serial.println(myData.ruch);
}
 
// Przeliczenie stopniów na radiany
float radiany(float stopnie){
  return stopnie*PI/180;
}

// Przeliczenie radianów na stopnie
float stopnie(float radiany){
  return radiany*(180.0/PI);
}

void setup() {
  myData.katRuchu = -1;
  // Set up Serial Monitor
  Serial.begin(115200);
  Wire.begin(21,22);
 
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS1115");
    while (1);
  }

  // Piny
  pinMode(33, INPUT);
  pinMode(32, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(33), przerwanie, RISING);

  // Ekran
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  // Display static text
  display.println("Hello, world!");
  display.display(); 
  Serial.println("Ekran zainicjowany");

  
}
 
void przerwanie() {
  Serial.println("Przycisk");
  chod = chod + 1;
}

void loop() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  adc2 = ads.readADC_SingleEnded(2);
  adc3 = ads.readADC_SingleEnded(3);
  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: "); Serial.println(adc0);
  Serial.print("AIN1: "); Serial.println(adc1);
  Serial.print("AIN2: "); Serial.println(adc2);
  Serial.print("AIN3: "); Serial.println(adc3);
  float promien, x, y, kat, sinA, cosA, wynik, tg;
  int obr, wys;
  if (adc0 >= (AIN0D + ROZDZIELCZOSC)){
    x = adc0-AIN0D;
    if (adc1 >= (AIN1D + ROZDZIELCZOSC)){
      y = adc1-AIN1D;
    }
    if (adc1 <= (AIN1D - ROZDZIELCZOSC)){
      y = adc1;
    }
    if ((adc1 > (AIN1D - ROZDZIELCZOSC)) && (adc1 < (AIN1D + ROZDZIELCZOSC))){
      y = AIN1D;
    }
  }

  if (adc0 <= (AIN0D - ROZDZIELCZOSC)){
    x = adc0;
    if (adc1 >= (AIN1D + ROZDZIELCZOSC)){
      y = adc1-AIN1D;
    }
    if (adc1 <= (AIN1D - ROZDZIELCZOSC)){
      y = adc1;
    }
    if ((adc1 > (AIN1D - ROZDZIELCZOSC)) && (adc1 < (AIN1D + ROZDZIELCZOSC))){
      y = AIN1D;
    }
  }

  if ((adc0 > (AIN0D - ROZDZIELCZOSC)) && (adc0 < (AIN0D + ROZDZIELCZOSC))){
    x = AIN0D;
    if (adc1 >= (AIN1D + ROZDZIELCZOSC)){
      y = adc1-AIN1D;
    }
    if (adc1 <= (AIN1D - ROZDZIELCZOSC)){
      y = adc1;
    }
    if ((adc1 > (AIN1D - ROZDZIELCZOSC)) && (adc1 < (AIN1D + ROZDZIELCZOSC))){
      y = AIN1D;
    }
  }
  
  if ((adc0 > (AIN0D - ROZDZIELCZOSC)) && (adc0 < (AIN0D + ROZDZIELCZOSC)) && (adc1 > (AIN1D - ROZDZIELCZOSC)) && (adc1 < (AIN1D + ROZDZIELCZOSC)) || (kat < 0)){
    wynik = -1;
  }
  if ((adc0 > (AIN0D - ROZDZIELCZOSC)) && (adc0 < (AIN0D + ROZDZIELCZOSC)) && (adc1 < (AIN1D - ROZDZIELCZOSC))){
    wynik = 90;
  }
  if ((adc0 > (AIN0D - ROZDZIELCZOSC)) && (adc0 < (AIN0D + ROZDZIELCZOSC)) && (adc1 > (AIN1D + ROZDZIELCZOSC))){
    wynik = 270;
  }
  if ((adc1 > (AIN1D - ROZDZIELCZOSC)) && (adc1 < (AIN1D + ROZDZIELCZOSC)) && (adc0 > (AIN0D + ROZDZIELCZOSC))){
    wynik = 0;
  }
  if ((adc1 > (AIN1D - ROZDZIELCZOSC)) && (adc1 < (AIN1D + ROZDZIELCZOSC)) && (adc0 < (AIN0D - ROZDZIELCZOSC))){
    wynik = 180;
  }
  if ((adc1 > (20000 - ROZDZIELCZOSC)) && (adc0 > (20000 - ROZDZIELCZOSC))){
    wynik = 315;
  }
  if ((adc1 < ROZDZIELCZOSC) && (adc0 > (20000 - ROZDZIELCZOSC))){
    wynik = 45;
  }
  if ((adc1 < ROZDZIELCZOSC) && (adc0 < ROZDZIELCZOSC)){
    wynik = 135;
  }
  if ((adc1 > (20000 - ROZDZIELCZOSC)) && (adc0 < ROZDZIELCZOSC)){
    wynik = 225;
  }



  if (adc2 > 16000){
    wys = 1;
  }
  if (adc2 < 8000){
    wys = -1;
  }
  if ((adc2 >= 8000) && (adc2 <= 16000)){
    wys = 0;
  }

  if (adc3 > 16000){
    obr = 1;
  }
  if (adc3 < 8000){
    obr = -1;
  }
  if ((adc3 <= 16000) && (adc3 >= 8000)){
    obr = 0;
  }

  if (chod >= 3){
    chod = 0;
  }

  myData.katRuchu = wynik;
  myData.obrot = obr;
  myData.wysokosc = wys;
  myData.ruch = chod;

  display.clearDisplay();
  display.setCursor(0, 10);
  display.print("Ruch: ");
  display.println(chod);
  display.print("Kat: ");
  display.println(wynik);
  display.print("Obrot: ");
  display.println(obr);
  display.print("Wysokosc: ");
  display.println(wys);
  display.display(); 
  Serial.print("Kat: "); Serial.println(kat);
  Serial.print("Wynik: "); Serial.println(wynik);
  if (result == ESP_OK) {
    Serial.println("Sending confirmed");
  }
  else {
    Serial.println("Sending error");
  }
  delay(250);
}