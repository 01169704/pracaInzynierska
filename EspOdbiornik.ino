#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define PI 3.14159265359
#define SERVOMIN  125
#define SERVOMAX  575
#define PREDKOSC 3.2 // 2.5 ms na 1 stopien
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) > (b) ? (b) : (a))
#define ZG 50

float L1 = 80.45;
float L2 = 87.9;
float L3 = 91.65;
float d1 = 90.5;

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // Prawa noga
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Lewa noga


// Tu przechowywane są wartości kątów w stopniach

// Wartości domowe kątów teta dla poszczególnych nóg
const float homeLewaNoga[3][3] = {{90,60,60},{90,60,60},{90,60,60}};
const float homePrawaNoga[3][3] = {{90,120,120},{90,120,120},{90,120,120}};

float lewaNoga[3][3] = {{homeLewaNoga[0][0],homeLewaNoga[0][1],homeLewaNoga[0][2]},{homeLewaNoga[1][0],homeLewaNoga[1][1],homeLewaNoga[1][2]},{homeLewaNoga[2][0],homeLewaNoga[2][1],homeLewaNoga[2][2]}};
float prawaNoga[3][3] = {{homePrawaNoga[0][0],homePrawaNoga[0][1],homePrawaNoga[0][2]},{homePrawaNoga[1][0],homePrawaNoga[1][1],homePrawaNoga[1][2]},{homePrawaNoga[2][0],homePrawaNoga[2][1],homePrawaNoga[2][2]}};

float staraLewaNoga[3][3] = {{homeLewaNoga[0][0],homeLewaNoga[0][1],homeLewaNoga[0][2]},{homeLewaNoga[1][0],homeLewaNoga[1][1],homeLewaNoga[1][2]},{homeLewaNoga[2][0],homeLewaNoga[2][1],homeLewaNoga[2][2]}};
float staraPrawaNoga[3][3] = {{homePrawaNoga[0][0],homePrawaNoga[0][1],homePrawaNoga[0][2]},{homePrawaNoga[1][0],homePrawaNoga[1][1],homePrawaNoga[1][2]},{homePrawaNoga[2][0],homePrawaNoga[2][1],homePrawaNoga[2][2]}};

// Tu podaję odległość w jakiej mają znajdować się nogi oraz wysokość
float cosKatRuchu, sinKatRuchu;
float suniecie;

float X = 160;
float Y = 0;
float Z = -50;

float deltaY;
float deltaX;

float wysokosc = -40;

float zerowanieLewa[3][3] = {{-225.3,335.6,90.5},{-403.2,0,90.5},{-225.3,-335.6,90.5}};
float zerowaniePrawa[3][3] = {{225.3,335.6,90.5},{403.2,0,90.5},{225.3,-335.6,90.5}};

float polozenieLewa[3][3] = {{zerowanieLewa[0][0],zerowanieLewa[0][1],zerowanieLewa[0][2]},{zerowanieLewa[1][0],zerowanieLewa[1][1],zerowanieLewa[1][2]},{zerowanieLewa[2][0],zerowanieLewa[2][1],zerowanieLewa[2][2]}};
float polozeniePrawa[3][3] = {{zerowaniePrawa[0][0],zerowaniePrawa[0][1],zerowaniePrawa[0][2]},{zerowaniePrawa[1][0],zerowaniePrawa[1][1],zerowaniePrawa[1][2]},{zerowaniePrawa[2][0],zerowaniePrawa[2][1],zerowaniePrawa[2][2]}};


// Współrzędne X,Y,Z gdy robot stoi
float doRuchuLewa[3][3] = {{-166.88,254.48,0},{-303.2,0,0},{-166.88,-254.48,0}};
float doRuchuPrawa[3][3] = {{166.88,254.48,0},{303.2,0,0},{166.88,-254.48,0}};



// Podniesienie
float ruchLewaNoga11[3][3];
float ruchPrawaNoga11[3][3];
// Przesuniecie w powietrzu
float ruchLewaNoga12[3][3];
float ruchPrawaNoga12[3][3];
// Opuszczenie
float ruchLewaNoga13[3][3];
float ruchPrawaNoga13[3][3];

// Podniesienie
float ruchLewaNoga21[3][3];
float ruchPrawaNoga21[3][3];
// Przesuniecie w powietrzu
float ruchLewaNoga22[3][3];
float ruchPrawaNoga22[3][3];
// Opuszczenie
float ruchLewaNoga23[3][3];
float ruchPrawaNoga23[3][3];

// Podniesienie
float ruchLewaNoga31[3][3];
float ruchPrawaNoga31[3][3];
// Przesuniecie w powietrzu
float ruchLewaNoga32[3][3];
float ruchPrawaNoga32[3][3];
// Opuszczenie
float ruchLewaNoga33[3][3];
float ruchPrawaNoga33[3][3];

// Podniesienie
float ruchLewaNoga41[3][3];
float ruchPrawaNoga41[3][3];
// Przesuniecie w powietrzu
float ruchLewaNoga42[3][3];
float ruchPrawaNoga42[3][3];
// Opuszczenie
float ruchLewaNoga43[3][3];
float ruchPrawaNoga43[3][3];

// Podniesienie
float ruchLewaNoga51[3][3];
float ruchPrawaNoga51[3][3];
// Przesuniecie w powietrzu
float ruchLewaNoga52[3][3];
float ruchPrawaNoga52[3][3];
// Opuszczenie
float ruchLewaNoga53[3][3];
float ruchPrawaNoga53[3][3];

// Podniesienie
float ruchLewaNoga61[3][3];
float ruchPrawaNoga61[3][3];
// Przesuniecie w powietrzu
float ruchLewaNoga62[3][3];
float ruchPrawaNoga62[3][3];
// Opuszczenie
float ruchLewaNoga63[3][3];
float ruchPrawaNoga63[3][3];


// Do bazowania nóg przy ruchu jedną nogą
float bazowanieLewaNoga11[3][3];
float bazowaniePrawaNoga11[3][3];
float bazowanieLewaNoga12[3][3];
float bazowaniePrawaNoga12[3][3];
float bazowanieLewaNoga13[3][3];
float bazowaniePrawaNoga13[3][3];

float bazowanieLewaNoga21[3][3];
float bazowaniePrawaNoga21[3][3];
float bazowanieLewaNoga22[3][3];
float bazowaniePrawaNoga22[3][3];
float bazowanieLewaNoga23[3][3];
float bazowaniePrawaNoga23[3][3];

float bazowanieLewaNoga31[3][3];
float bazowaniePrawaNoga31[3][3];
float bazowanieLewaNoga32[3][3];
float bazowaniePrawaNoga32[3][3];
float bazowanieLewaNoga33[3][3];
float bazowaniePrawaNoga33[3][3];

float bazowanieLewaNoga41[3][3];
float bazowaniePrawaNoga41[3][3];
float bazowanieLewaNoga42[3][3];
float bazowaniePrawaNoga42[3][3];
float bazowanieLewaNoga43[3][3];
float bazowaniePrawaNoga43[3][3];

float bazowanieLewaNoga51[3][3];
float bazowaniePrawaNoga51[3][3];
float bazowanieLewaNoga52[3][3];
float bazowaniePrawaNoga52[3][3];
float bazowanieLewaNoga53[3][3];
float bazowaniePrawaNoga53[3][3];

float bazowanieLewaNoga61[3][3];
float bazowaniePrawaNoga61[3][3];
float bazowanieLewaNoga62[3][3];
float bazowaniePrawaNoga62[3][3];
float bazowanieLewaNoga63[3][3];
float bazowaniePrawaNoga63[3][3];

float pierwszyTrojpodporowyLewa11[3][3];
float pierwszyTrojpodporowyLewa12[3][3];
float pierwszyTrojpodporowyLewa13[3][3];
float pierwszyTrojpodporowyPrawa11[3][3];
float pierwszyTrojpodporowyPrawa12[3][3];
float pierwszyTrojpodporowyPrawa13[3][3];

float trojpodporowyLewa11[3][3];
float trojpodporowyLewa12[3][3];
float trojpodporowyLewa13[3][3];
float trojpodporowyPrawa11[3][3];
float trojpodporowyPrawa12[3][3];
float trojpodporowyPrawa13[3][3];

float trojpodporowyLewa21[3][3];
float trojpodporowyLewa22[3][3];
float trojpodporowyLewa23[3][3];
float trojpodporowyPrawa21[3][3];
float trojpodporowyPrawa22[3][3];
float trojpodporowyPrawa23[3][3];

// Define a data structure
typedef struct struct_message {
  float katRuchu;
  int wysokosc;
  int obrot;
  int ruch;
} struct_message;
 
// Create a structured object
struct_message myData;

// Struktura zawierająca kąty teta
typedef struct {
  float teta1;
  float teta2;
  float teta3;
} teta;

struct Wynik {
  float x, y, z;
};

int ruch = -1;
int obrot = 0;
float katRuchu = -1;

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("katRuchu: ");
  Serial.println(myData.katRuchu);
  Serial.print("Wysokosc: ");
  Serial.println(myData.wysokosc);
  Serial.print("Obrot: ");
  Serial.println(myData.obrot);
  Serial.print("Ruch: ");
  Serial.println(myData.ruch);
  Serial.println();
  ruch = myData.ruch;
  obrot = myData.obrot;
  katRuchu = myData.katRuchu;
  Serial.print("ruch w loop: ");
  Serial.println(myData.ruch);
  Serial.print("katRuchu w loop: ");
  Serial.println(myData.katRuchu);
}

// Przeliczenie kąta na położenie serwomechanizmu
int angleToPulse(int ang){
  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);
  return pulse;
}

// Ustawienie danego kąta na danym sterowniku i wyjściu
void ustawSerwo(Adafruit_PWMServoDriver sterownik, int wejscie, int kat) {
  sterownik.setPWM(wejscie, 0, angleToPulse(kat));
}

// Ustawienie kątów serwomechanizmów w lewej nodze
void ustawLewaNoga(int numerNogi, int katUdo, int katGolen, int katPiszczel){ //Noga pierwsza - 0, Noga druga - 1, Noga trzecia - 2
  ustawSerwo(pwm2,2 + numerNogi*3, katUdo);
  ustawSerwo(pwm2,1 + numerNogi*3, katGolen);
  ustawSerwo(pwm2,0 + numerNogi*3, katPiszczel);
}

// Ustawienie kątów serwomechanizmów w prawej nodze
void ustawPrawaNoga(int numerNogi, int katUdo, int katGolen, int katPiszczel){ //Noga pierwsza - 0, Noga druga - 1, Noga trzecia - 2
  ustawSerwo(pwm1,13 - numerNogi*3, katUdo);
  ustawSerwo(pwm1,14 - numerNogi*3, katGolen);
  ustawSerwo(pwm1,15 - numerNogi*3, katPiszczel);
}

// Przeliczenie stopniów na radiany
float radiany(float stopnie){
  return stopnie*PI/180;
}

// Przeliczenie radianów na stopnie
float stopnie(float radiany){
  return radiany*(180.0/PI);
}

// Wyznaczenie sinusa danego kąta podanego w stopniach, a wynik w radianach
float sinus(float stopnie){
  float rad = radiany(stopnie);
  float sinus = sin(rad);
  return sinus;
}

// Wyznaczenie cosinusa danego kąta podanego w stopniach, a wynik w radianach
float cosinus(float stopnie){
  float rad = radiany(stopnie);
  float cosinus = cos(rad);
  return cosinus;
}

// Rozwiązanie zagadnienia odwrotnego podając współrzędne w układzie globalnym
void zagadnienieOdwrotne(float Px0, float Py0, float Pz0, char strona, int numerNogi){
  // Wyznaczenie położenia w ukłdzie nogi
  float qx,qy,Teta0;
  if (strona == 'L' && numerNogi == 0){
    qx = -73.41;
    qy = 124.6;
    Teta0 = 125.74;
  }
  if (strona == 'L' && numerNogi == 1){
    qx = -143.24;
    qy = 0;
    Teta0 = 180;
  }
  if (strona == 'L' && numerNogi == 2){
    qx = -73.41;
    qy = -124.6;
    Teta0 = 234.25;
  }
  if (strona == 'P' && numerNogi == 0){
    qx = 73.41;
    qy = 124.6;
    Teta0 = 54.25;
  }
  if (strona == 'P' && numerNogi == 1){
    qx = 143.24;
    qy = 0;
    Teta0 = 0;
  }
  if (strona == 'P' && numerNogi == 2){
    qx = 73.41;
    qy = -124.6;
    Teta0 = -54.25;
  }
  // Rozwiązanie zagadnienia odwrotnego
  float Px = cosinus(Teta0)*(Px0-qx)+sinus(Teta0)*(Py0-qy);
  float Py;
  if (strona == 'L'){
    Py = sinus(Teta0)*(qx-Px0)+cosinus(Teta0)*(Py0-qy);
  }
  if (strona == 'P'){
    Py = -(sinus(Teta0)*(qx-Px0)+cosinus(Teta0)*(Py0-qy));
  }
  float Pz = Pz0;
  float kwadrat = sqrt(Px*Px+Py*Py);
  float ciag = kwadrat - L1;
  float cosTeta3 = (ciag*ciag + (Pz - d1)*(Pz - d1) - L2*L2 - L3*L3)/(2*L2*L2);
  if (cosTeta3 > 1.0f) cosTeta3 = 1.0f;
  if (cosTeta3 < -1.0f) cosTeta3 = -1.0f;
  float sinTeta3 = -sqrt(1-cosTeta3*cosTeta3);
  float teta3 = stopnie(atan2(sinTeta3,cosTeta3));
  float k1 = L3*cosTeta3 + L2;
  float k2 = L3*sinTeta3;
  float r = sqrt(k1*k1+k2*k2);
  float teta2 = stopnie(atan2((Pz-d1)/r,ciag/r) - atan2(k2/r,k1/r));
  float teta1 = stopnie(atan2(Py/kwadrat, Px/kwadrat));
  // Uwzględnienie przesunięcia kątów
  if (strona == 'L'){
    lewaNoga[numerNogi][0] = 90 + teta1;
    lewaNoga[numerNogi][1] = 60 - teta2;
    lewaNoga[numerNogi][2] = 60 - teta3;
  }
  if (strona == 'P'){
    prawaNoga[numerNogi][0] = 90 - teta1;
    prawaNoga[numerNogi][1] = 120 + teta2;
    prawaNoga[numerNogi][2] = 120 + teta3;  
  }
  // Wypisanie w terminalu kątów nogi
  Serial.print("Zagadnienie odwrotne dla nogi ");
  Serial.print(numerNogi);
  Serial.print(strona);
  if (strona == 'L'){
    Serial.print(" Teta1: ");
    Serial.print(lewaNoga[numerNogi][0]);
    Serial.print(", ");
    Serial.print(" Teta2: ");
    Serial.print(lewaNoga[numerNogi][1]);
    Serial.print(", ");
    Serial.print(" Teta3: ");
    Serial.println(lewaNoga[numerNogi][2]);
  }
    if (strona == 'P'){
    Serial.print(" Teta1: ");
    Serial.print(prawaNoga[numerNogi][0]);
    Serial.print(", ");
    Serial.print(" Teta2: ");
    Serial.print(prawaNoga[numerNogi][1]);
    Serial.print(", ");
    Serial.print(" Teta3: ");
    Serial.println(prawaNoga[numerNogi][2]);
  }
}

// Sprawdzenie czy wyznaczone przesunięcie w funkcji promien() jest osiągalne
float sprawdzPromien(float r, float x1, float x2, float y1) {
    while (r >= 0) { // Pętla działa, dopóki r >= 0
        // Sprawdzam warunki dla x1, y1 + r
        zagadnienieOdwrotne(x1, y1 + r, doRuchuPrawa[0][2], 'P', 0);
        if ((prawaNoga[0][0] > 0 && prawaNoga[0][0] < 180) &&
            (prawaNoga[0][1] > 0 && prawaNoga[0][1] < 180) &&
            (prawaNoga[0][2] > 0 && prawaNoga[0][2] < 180) && 
            (prawaNoga[0][2] != 120)) {
            
            // Sprawdzam warunki dla x2, r
            zagadnienieOdwrotne(x2, r, doRuchuPrawa[0][2], 'P', 1);
            if ((prawaNoga[1][0] > 0 && prawaNoga[1][0] < 180) &&
                (prawaNoga[1][1] > 0 && prawaNoga[1][1] < 180) &&
                (prawaNoga[1][2] > 0 && prawaNoga[1][2] < 180) && 
                (prawaNoga[1][2] != 120)) {
                
                // Sprawdzam warunki dla x1, y1 - r
                zagadnienieOdwrotne(x1, y1 - r, doRuchuPrawa[0][2], 'P', 0);
                if ((prawaNoga[0][0] > 0 && prawaNoga[0][0] < 180) &&
                    (prawaNoga[0][1] > 0 && prawaNoga[0][1] < 180) &&
                    (prawaNoga[0][2] > 0 && prawaNoga[0][2] < 180) && 
                    (prawaNoga[0][2] != 120)) {
                    
                    // Sprawdzam warunki dla x2, -r
                    zagadnienieOdwrotne(x2, -r, doRuchuPrawa[0][2], 'P', 1);
                    if ((prawaNoga[1][0] > 0 && prawaNoga[1][0] < 180) &&
                        (prawaNoga[1][1] > 0 && prawaNoga[1][1] < 180) &&
                        (prawaNoga[1][2] > 0 && prawaNoga[1][2] < 180) && 
                        (prawaNoga[1][2] != 120)) {
                        
                        // Jeśli wszystkie warunki są spełnione, zwracam r
                        return r;
                    }
                }
            }
        }
        
        // Jeśli warunki nie są spełnione, zmniejszam r
        r -= 1;
    }
    
    // Jeśli pętla dojdzie tutaj, oznacza to, że warunki nigdy nie były spełnione
    return 1; // Wartość oznaczająca brak poprawnego rozwiązania
}

// Wyznaczenie przesunięcia dla nogi przy ruchu do przodu/tyłu
float promien(float x1, float x2, float y1){
  float Teta = 54.25;
  float Teta2 = 71.49;
  float cosT = cos(Teta);
  float cosT2 = cos(Teta2);
  // Wyznaczam r dla nogi 0 i 1
  float r = (sqrt(x1*x1+y1*y1+x2*x2-2*sqrt(x1*x1+y1*y1)*sqrt(x2*x2)*cosT)-80)/2;
  // Sprawdzam, czy robot może osiągnąć takie r
  float wynik = sprawdzPromien(r,x1,x2,y1);
  // Jeśli nie może to zmniejszam r
  return wynik;
}

// Wyznaczenie przesunięcia dla nogi przy ruchu w bok
float promienX(float r, float x1, float x2, float y1) {
    float maxR = r; // Przechowuje największe r, dla którego warunki są spełnione

    while (true) {
        // Sprawdzamy warunki dla x1 + r
        zagadnienieOdwrotne(x1 + r, y1, doRuchuPrawa[0][2], 'P', 0);
        if (!((prawaNoga[0][0] > 0 && prawaNoga[0][0] < 180) && 
              (prawaNoga[0][1] > 0 && prawaNoga[0][1] < 180) && 
              (prawaNoga[0][2] > 0 && prawaNoga[0][2] < 180) && 
              (prawaNoga[0][2] != 120))) {
            break; // Warunki nie są spełnione, przerywamy pętlę
        }

        // Sprawdzamy warunki dla x2 + r
        zagadnienieOdwrotne(x2 + r, 0, doRuchuPrawa[0][2], 'P', 1);
        if (!((prawaNoga[1][0] > 0 && prawaNoga[1][0] < 180) && 
              (prawaNoga[1][1] > 0 && prawaNoga[1][1] < 180) && 
              (prawaNoga[1][2] > 0 && prawaNoga[1][2] < 180) && 
              (prawaNoga[1][2] != 120))) {
            break; // Warunki nie są spełnione, przerywamy pętlę
        }

        // Sprawdzamy warunki dla x1 - r
        zagadnienieOdwrotne(x1 - r, y1, doRuchuPrawa[0][2], 'P', 0);
        if (!((prawaNoga[0][0] > 0 && prawaNoga[0][0] < 180) && 
              (prawaNoga[0][1] > 0 && prawaNoga[0][1] < 180) && 
              (prawaNoga[0][2] > 0 && prawaNoga[0][2] < 180) && 
              (prawaNoga[0][2] != 120))) {
            break; // Warunki nie są spełnione, przerywamy pętlę
        }

        // Sprawdzamy warunki dla x2 - r
        zagadnienieOdwrotne(x2 - r, 0, doRuchuPrawa[0][2], 'P', 1);
        if (!((prawaNoga[1][0] > 0 && prawaNoga[1][0] < 180) && 
              (prawaNoga[1][1] > 0 && prawaNoga[1][1] < 180) && 
              (prawaNoga[1][2] > 0 && prawaNoga[1][2] < 180) && 
              (prawaNoga[1][2] != 120))) {
            break; // Warunki nie są spełnione, przerywamy pętlę
        }

        // Wszystkie warunki są spełnione, zwiększamy r i aktualizujemy maxR
        maxR = r;
        r += 1;
    }

    return maxR; // Zwracamy największe r, dla którego warunki były spełnione
}

// Przejście współrzędnych z układu lokalnego nogi do układu globalnego robota
Wynik lokalnyDoBazowego(float Px, float Py, float Pz, char strona, int numerNogi){
  float qx,qy,Teta0;
  Wynik wynik;
  if (strona == 'L' && numerNogi == 0){
    qx = -73.41;
    qy = 124.6;
    Teta0 = 125.74;
  }
  if (strona == 'L' && numerNogi == 1){
    qx = -143.24;
    qy = 0;
    Teta0 = 180;
  }
  if (strona == 'L' && numerNogi == 2){
    qx = -73.41;
    qy = -124.6;
    Teta0 = 234.25;
  }
  if (strona == 'P' && numerNogi == 0){
    qx = 73.41;
    qy = 124.6;
    Teta0 = 54.25;
  }
  if (strona == 'P' && numerNogi == 1){
    qx = 143.24;
    qy = 0;
    Teta0 = 0;
  }
  if (strona == 'P' && numerNogi == 2){
    qx = 73.41;
    qy = -124.6;
    Teta0 = -54.25;
  }
  wynik.x = cosinus(Teta0)*Px-sinus(Teta0)*Py+qx;
  wynik.y = sinus(Teta0)*Px+cosinus(Teta0)*Py+qy;
  wynik.z = Pz;
  return wynik;
}


// Funkcja realizująca ruch liniowy
void ruchLiniowy(float wartosciKoncoweLewa[3][3], float wartosciKoncowePrawa[3][3], int podziel){
  // Wyznaczenie przesunięć w układzie współrzędnych
  float deltaX0L = (wartosciKoncoweLewa[0][0] - polozenieLewa[0][0])/podziel;
  float deltaY0L = (wartosciKoncoweLewa[0][1] - polozenieLewa[0][1])/podziel;
  float deltaZ0L = (wartosciKoncoweLewa[0][2] - polozenieLewa[0][2])/podziel;

  float deltaX1L = (wartosciKoncoweLewa[1][0] - polozenieLewa[1][0])/podziel;
  float deltaY1L = (wartosciKoncoweLewa[1][1] - polozenieLewa[1][1])/podziel;
  float deltaZ1L = (wartosciKoncoweLewa[1][2] - polozenieLewa[1][2])/podziel;

  float deltaX2L = (wartosciKoncoweLewa[2][0] - polozenieLewa[2][0])/podziel;
  float deltaY2L = (wartosciKoncoweLewa[2][1] - polozenieLewa[2][1])/podziel;
  float deltaZ2L = (wartosciKoncoweLewa[2][2] - polozenieLewa[2][2])/podziel;

  float deltaX0P = (wartosciKoncowePrawa[0][0] - polozeniePrawa[0][0])/podziel;
  float deltaY0P = (wartosciKoncowePrawa[0][1] - polozeniePrawa[0][1])/podziel;
  float deltaZ0P = (wartosciKoncowePrawa[0][2] - polozeniePrawa[0][2])/podziel;

  float deltaX1P = (wartosciKoncowePrawa[1][0] - polozeniePrawa[1][0])/podziel;
  float deltaY1P = (wartosciKoncowePrawa[1][1] - polozeniePrawa[1][1])/podziel;
  float deltaZ1P = (wartosciKoncowePrawa[1][2] - polozeniePrawa[1][2])/podziel;

  float deltaX2P = (wartosciKoncowePrawa[2][0] - polozeniePrawa[2][0])/podziel;
  float deltaY2P = (wartosciKoncowePrawa[2][1] - polozeniePrawa[2][1])/podziel;
  float deltaZ2P = (wartosciKoncowePrawa[2][2] - polozeniePrawa[2][2])/podziel;



  for(int i = 1; podziel >= i; i++){
    zagadnienieOdwrotne(polozenieLewa[0][0] + deltaX0L*i, polozenieLewa[0][1] + deltaY0L*i, polozenieLewa[0][2] + deltaZ0L*i, 'L', 0);
    zagadnienieOdwrotne(polozenieLewa[1][0] + deltaX1L*i, polozenieLewa[1][1] + deltaY1L*i, polozenieLewa[1][2] + deltaZ1L*i, 'L', 1);
    zagadnienieOdwrotne(polozenieLewa[2][0] + deltaX2L*i, polozenieLewa[2][1] + deltaY2L*i, polozenieLewa[2][2] + deltaZ2L*i, 'L', 2);

    zagadnienieOdwrotne(polozeniePrawa[0][0] + deltaX0P*i, polozeniePrawa[0][1] + deltaY0P*i, polozeniePrawa[0][2] + deltaZ0P*i, 'P', 0);
    zagadnienieOdwrotne(polozeniePrawa[1][0] + deltaX1P*i, polozeniePrawa[1][1] + deltaY1P*i, polozeniePrawa[1][2] + deltaZ1P*i, 'P', 1);
    zagadnienieOdwrotne(polozeniePrawa[2][0] + deltaX2P*i, polozeniePrawa[2][1] + deltaY2P*i, polozeniePrawa[2][2] + deltaZ2P*i, 'P', 2);

    float przesuniecie0T1L = lewaNoga[0][0] - staraLewaNoga[0][0];
    float przesuniecie0T2L = lewaNoga[0][1] - staraLewaNoga[0][1];
    float przesuniecie0T3L = lewaNoga[0][2] - staraLewaNoga[0][2];

    float przesuniecie1T1L = lewaNoga[1][0] - staraLewaNoga[1][0];
    float przesuniecie1T2L = lewaNoga[1][1] - staraLewaNoga[1][1];
    float przesuniecie1T3L = lewaNoga[1][2] - staraLewaNoga[1][2];

    float przesuniecie2T1L = lewaNoga[2][0] - staraLewaNoga[2][0];
    float przesuniecie2T2L = lewaNoga[2][1] - staraLewaNoga[2][1];
    float przesuniecie2T3L = lewaNoga[2][2] - staraLewaNoga[2][2];

    float przesuniecie0T1P = prawaNoga[0][0] - staraPrawaNoga[0][0];
    float przesuniecie0T2P = prawaNoga[0][1] - staraPrawaNoga[0][1];
    float przesuniecie0T3P = prawaNoga[0][2] - staraPrawaNoga[0][2];

    float przesuniecie1T1P = prawaNoga[1][0] - staraPrawaNoga[1][0];
    float przesuniecie1T2P = prawaNoga[1][1] - staraPrawaNoga[1][1];
    float przesuniecie1T3P = prawaNoga[1][2] - staraPrawaNoga[1][2];

    float przesuniecie2T1P = prawaNoga[2][0] - staraPrawaNoga[2][0];
    float przesuniecie2T2P = prawaNoga[2][1] - staraPrawaNoga[2][1];
    float przesuniecie2T3P = prawaNoga[2][2] - staraPrawaNoga[2][2];

    float max0L = MAX(MAX(abs(przesuniecie0T1L),abs(przesuniecie0T2L)),abs(przesuniecie0T3L));
    float max1L = MAX(MAX(abs(przesuniecie1T1L),abs(przesuniecie1T2L)),abs(przesuniecie1T3L));
    float max2L = MAX(MAX(abs(przesuniecie2T1L),abs(przesuniecie2T2L)),abs(przesuniecie2T3L));

    float max0P = MAX(MAX(abs(przesuniecie0T1P),abs(przesuniecie0T2P)),abs(przesuniecie0T3P));
    float max1P = MAX(MAX(abs(przesuniecie1T1P),abs(przesuniecie1T2P)),abs(przesuniecie1T3P));
    float max2P = MAX(MAX(abs(przesuniecie2T1P),abs(przesuniecie2T2P)),abs(przesuniecie2T3P));

    float maxL = MAX(MAX(max0L,max1L),max2L);
    float maxP = MAX(MAX(max0P,max1P),max2P);
    float max = MAX(maxL,maxP);
    float opoznienie = PREDKOSC*max;

    if (lewaNoga[0][0] < 0){
      lewaNoga[0][0] = 0;
    }
    if (lewaNoga[0][0] > 180){
      lewaNoga[0][0] = 180;
    }
    if (lewaNoga[0][1] < 0){
      lewaNoga[0][1] = 0;
    }
    if (lewaNoga[0][1] > 180){
      lewaNoga[0][1] = 180;
    }
    if (lewaNoga[0][2] < 0){
      lewaNoga[0][2] = 0;
    }
    if (lewaNoga[0][2] > 180){
      lewaNoga[0][2] = 180;
    }

    if (lewaNoga[1][0] < 0){
      lewaNoga[0][0] = 0;
    }
    if (lewaNoga[1][0] > 180){
      lewaNoga[0][0] = 180;
    }
    if (lewaNoga[1][1] < 0){
      lewaNoga[1][1] = 0;
    }
    if (lewaNoga[1][1] > 180){
      lewaNoga[1][1] = 180;
    }
    if (lewaNoga[1][2] < 0){
      lewaNoga[1][2] = 0;
    }
    if (lewaNoga[1][2] > 180){
      lewaNoga[1][2] = 180;
    }

    if (lewaNoga[2][0] < 0){
      lewaNoga[2][0] = 0;
    }
    if (lewaNoga[2][0] > 180){
      lewaNoga[2][0] = 180;
    }
    if (lewaNoga[2][1] < 0){
      lewaNoga[2][1] = 0;
    }
    if (lewaNoga[2][1] > 180){
      lewaNoga[2][1] = 180;
    }
    if (lewaNoga[2][2] < 0){
      lewaNoga[2][2] = 0;
    }
    if (lewaNoga[2][2] > 180){
      lewaNoga[2][2] = 180;
    }


    if (prawaNoga[0][0] < 0){
      prawaNoga[0][0] = 0;
    }
    if (prawaNoga[0][0] > 180){
      prawaNoga[0][0] = 180;
    }
    if (prawaNoga[0][1] < 0){
      prawaNoga[0][1] = 0;
    }
    if (prawaNoga[0][1] > 180){
      prawaNoga[0][1] = 180;
    }
    if (prawaNoga[0][2] < 0){
      prawaNoga[0][2] = 0;
    }
    if (prawaNoga[0][2] > 180){
      prawaNoga[0][2] = 180;
    }

    if (prawaNoga[1][0] < 0){
      prawaNoga[1][0] = 0;
    }
    if (prawaNoga[1][0] > 180){
      prawaNoga[1][0] = 180;
    }
    if (prawaNoga[1][1] < 0){
      prawaNoga[1][1] = 0;
    }
    if (prawaNoga[1][1] > 180){
      prawaNoga[1][1] = 180;
    }
    if (prawaNoga[1][2] < 0){
      prawaNoga[1][2] = 0;
    }
    if (prawaNoga[1][2] > 180){
      prawaNoga[1][2] = 180;
    }

    if (prawaNoga[2][0] < 0){
      prawaNoga[2][0] = 0;
    }
    if (prawaNoga[2][0] > 180){
      prawaNoga[2][0] = 180;
    }
    if (prawaNoga[2][1] < 0){
      prawaNoga[2][1] = 0;
    }
    if (prawaNoga[2][1] > 180){
      prawaNoga[2][1] = 180;
    }
    if (prawaNoga[2][2] < 0){
      prawaNoga[2][2] = 0;
    }
    if (prawaNoga[2][2] > 180){
      prawaNoga[2][2] = 180;
    }


    ustawLewaNoga(0, lewaNoga[0][0], lewaNoga[0][1], lewaNoga[0][2]);
    ustawLewaNoga(1, lewaNoga[1][0], lewaNoga[1][1], lewaNoga[1][2]);
    ustawLewaNoga(2, lewaNoga[2][0], lewaNoga[2][1], lewaNoga[2][2]);

    ustawPrawaNoga(0, prawaNoga[0][0], prawaNoga[0][1], prawaNoga[0][2]);
    ustawPrawaNoga(1, prawaNoga[1][0], prawaNoga[1][1], prawaNoga[1][2]);
    ustawPrawaNoga(2, prawaNoga[2][0], prawaNoga[2][1], prawaNoga[2][2]);

    staraLewaNoga[0][0] = lewaNoga[0][0];
    staraLewaNoga[0][1] = lewaNoga[0][1];
    staraLewaNoga[0][2] = lewaNoga[0][2];

    staraLewaNoga[1][0] = lewaNoga[1][0];
    staraLewaNoga[1][1] = lewaNoga[1][1];
    staraLewaNoga[1][2] = lewaNoga[1][2];

    staraLewaNoga[2][0] = lewaNoga[2][0];
    staraLewaNoga[2][1] = lewaNoga[2][1];
    staraLewaNoga[2][2] = lewaNoga[2][2];

    staraPrawaNoga[0][0] = prawaNoga[0][0];
    staraPrawaNoga[0][1] = prawaNoga[0][1];
    staraPrawaNoga[0][2] = prawaNoga[0][2];

    staraPrawaNoga[1][0] = prawaNoga[1][0];
    staraPrawaNoga[1][1] = prawaNoga[1][1];
    staraPrawaNoga[1][2] = prawaNoga[1][2];

    staraPrawaNoga[2][0] = prawaNoga[2][0];
    staraPrawaNoga[2][1] = prawaNoga[2][1];
    staraPrawaNoga[2][2] = prawaNoga[2][2];

    delay(opoznienie);
  }
  polozenieLewa[0][0] = wartosciKoncoweLewa[0][0];
  polozenieLewa[0][1] = wartosciKoncoweLewa[0][1];
  polozenieLewa[0][2] = wartosciKoncoweLewa[0][2];

  polozenieLewa[1][0] = wartosciKoncoweLewa[1][0];
  polozenieLewa[1][1] = wartosciKoncoweLewa[1][1];
  polozenieLewa[1][2] = wartosciKoncoweLewa[1][2];

  polozenieLewa[2][0] = wartosciKoncoweLewa[2][0];
  polozenieLewa[2][1] = wartosciKoncoweLewa[2][1];
  polozenieLewa[2][2] = wartosciKoncoweLewa[2][2];
  
  polozeniePrawa[0][0] = wartosciKoncowePrawa[0][0];
  polozeniePrawa[0][1] = wartosciKoncowePrawa[0][1];
  polozeniePrawa[0][2] = wartosciKoncowePrawa[0][2];

  polozeniePrawa[1][0] = wartosciKoncowePrawa[1][0];
  polozeniePrawa[1][1] = wartosciKoncowePrawa[1][1];
  polozeniePrawa[1][2] = wartosciKoncowePrawa[1][2];

  polozeniePrawa[2][0] = wartosciKoncowePrawa[2][0];
  polozeniePrawa[2][1] = wartosciKoncowePrawa[2][1];
  polozeniePrawa[2][2] = wartosciKoncowePrawa[2][2];
}

// Funkcja wyznaczająca długość kroku robota
void dlugoscKroku(float X, float Y, float Z){
  Wynik w1 = lokalnyDoBazowego(X, Y, Z, 'L', 0);
  doRuchuLewa[0][0] = w1.x;
  doRuchuLewa[0][1] = w1.y; 
  doRuchuLewa[0][2] = w1.z;

  Wynik w2 = lokalnyDoBazowego(X, Y, Z, 'L', 1);
  doRuchuLewa[1][0] = w2.x;
  doRuchuLewa[1][1] = w2.y;
  doRuchuLewa[1][2] = w2.z;

  Wynik w3 = lokalnyDoBazowego(X, Y, Z, 'L', 2);
  doRuchuLewa[2][0] = w3.x;
  doRuchuLewa[2][1] = w3.y;
  doRuchuLewa[2][2] = w3.z;

  Wynik w4 = lokalnyDoBazowego(X, Y, Z, 'P', 0);
  doRuchuPrawa[0][0] = w4.x;
  doRuchuPrawa[0][1] = w4.y;
  doRuchuPrawa[0][2] = w4.z;

  Wynik w5 = lokalnyDoBazowego(X, Y, Z, 'P', 1);
  doRuchuPrawa[1][0] = w5.x;
  doRuchuPrawa[1][1] = w5.y;
  doRuchuPrawa[1][2] = w5.z;

  Wynik w6 = lokalnyDoBazowego(X, Y, Z, 'P', 2);
  doRuchuPrawa[2][0] = w6.x;
  doRuchuPrawa[2][1] = w6.y;
  doRuchuPrawa[2][2] = w6.z;

  deltaY = promien(doRuchuPrawa[0][0],doRuchuPrawa[1][0],doRuchuPrawa[0][1]);
  Serial.print("deltaY: ");
  Serial.println(deltaY);
  //delay(1000);
  deltaX = promienX(1, doRuchuPrawa[0][0],doRuchuPrawa[1][0], doRuchuPrawa[0][1]);
  Serial.print("deltaX: ");
  Serial.println(deltaX);
  //delay(1000);
}

// Funkcja wyznaczająca przemieszczenie i kierunek robota w zależności od odczytanego kąta
void wyznaczKatRuchu(float kat){
  float cosGamma = deltaY/(sqrt(deltaY*deltaY+deltaX*deltaX));
  float sinGamma = deltaX/(sqrt(deltaX*deltaX+deltaY*deltaY));
  float gamma = stopnie(atan2(sinGamma,cosGamma));
  sinKatRuchu = sinus(kat);
  cosKatRuchu = cosinus(kat);
  if (kat > 270){
    kat = kat - 270;
  }
  if (kat > 180){
    kat = kat - 180;
  }
  if (kat > 90){
    kat = kat - 90;
  }
  float ro = 90 + kat - gamma;
  suniecie = abs(deltaY*sinus(gamma)/sinus(90+kat-gamma));
  Serial.print("Suniecie: ");
  Serial.println(suniecie);
  Serial.print("Kat: ");
  Serial.println(kat);
}

// Funkcja realizująca wstępne ustawienie pozycji nóg, przy poruszaniu tylko 1 nogą na raz
void wstepnyRuchPojedynczejNogi(){
  float suniecieX = sinKatRuchu*suniecie;
  float suniecieY = cosKatRuchu*suniecie;

  // Podniesienie L0
  bazowanieLewaNoga11[0][0] = doRuchuLewa[0][0];
  bazowanieLewaNoga11[0][1] = doRuchuLewa[0][1];
  bazowanieLewaNoga11[0][2] = doRuchuLewa[0][2] + ZG;
  bazowanieLewaNoga11[1][0] = doRuchuLewa[1][0];
  bazowanieLewaNoga11[1][1] = doRuchuLewa[1][1];
  bazowanieLewaNoga11[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga11[2][0] = doRuchuLewa[2][0];
  bazowanieLewaNoga11[2][1] = doRuchuLewa[2][1];
  bazowanieLewaNoga11[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga11[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga11[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga11[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga11[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga11[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga11[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga11[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga11[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga11[2][2] = doRuchuPrawa[2][2];

  // Przesunięcie w powietrzu L0
  bazowanieLewaNoga12[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga12[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga12[0][2] = doRuchuLewa[0][2] + ZG;
  bazowanieLewaNoga12[1][0] = doRuchuLewa[1][0];
  bazowanieLewaNoga12[1][1] = doRuchuLewa[1][1];
  bazowanieLewaNoga12[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga12[2][0] = doRuchuLewa[2][0];
  bazowanieLewaNoga12[2][1] = doRuchuLewa[2][1];
  bazowanieLewaNoga12[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga12[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga12[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga12[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga12[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga12[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga12[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga12[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga12[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga12[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L0
  bazowanieLewaNoga13[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga13[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga13[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga13[1][0] = doRuchuLewa[1][0];
  bazowanieLewaNoga13[1][1] = doRuchuLewa[1][1];
  bazowanieLewaNoga13[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga13[2][0] = doRuchuLewa[2][0];
  bazowanieLewaNoga13[2][1] = doRuchuLewa[2][1];
  bazowanieLewaNoga13[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga13[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga13[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga13[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga13[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga13[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga13[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga13[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga13[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga13[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L1
  bazowanieLewaNoga21[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga21[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga21[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga21[1][0] = doRuchuLewa[1][0];
  bazowanieLewaNoga21[1][1] = doRuchuLewa[1][1];
  bazowanieLewaNoga21[1][2] = doRuchuLewa[1][2] + ZG;
  bazowanieLewaNoga21[2][0] = doRuchuLewa[2][0];
  bazowanieLewaNoga21[2][1] = doRuchuLewa[2][1];
  bazowanieLewaNoga21[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga21[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga21[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga21[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga21[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga21[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga21[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga21[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga21[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga21[2][2] = doRuchuPrawa[2][2];

  // Przesunięcie w powietrzu L1
  bazowanieLewaNoga22[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga22[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga22[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga22[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga22[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga22[1][2] = doRuchuLewa[1][2] + ZG;
  bazowanieLewaNoga22[2][0] = doRuchuLewa[2][0];
  bazowanieLewaNoga22[2][1] = doRuchuLewa[2][1];
  bazowanieLewaNoga22[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga22[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga22[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga22[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga22[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga22[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga22[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga22[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga22[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga22[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L1
  bazowanieLewaNoga23[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga23[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga23[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga23[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga23[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga23[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga23[2][0] = doRuchuLewa[2][0];
  bazowanieLewaNoga23[2][1] = doRuchuLewa[2][1];
  bazowanieLewaNoga23[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga23[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga23[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga23[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga23[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga23[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga23[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga23[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga23[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga23[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L2
  bazowanieLewaNoga31[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga31[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga31[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga31[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga31[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga31[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga31[2][0] = doRuchuLewa[2][0];
  bazowanieLewaNoga31[2][1] = doRuchuLewa[2][1];
  bazowanieLewaNoga31[2][2] = doRuchuLewa[2][2] + ZG;

  bazowaniePrawaNoga31[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga31[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga31[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga31[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga31[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga31[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga31[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga31[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga31[2][2] = doRuchuPrawa[2][2];

  // Przesunięcie w powietrzu L2
  bazowanieLewaNoga32[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga32[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga32[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga32[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga32[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga32[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga32[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga32[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga32[2][2] = doRuchuLewa[2][2] + ZG;

  bazowaniePrawaNoga32[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga32[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga32[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga32[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga32[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga32[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga32[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga32[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga32[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L2
  bazowanieLewaNoga33[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga33[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga33[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga33[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga33[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga33[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga33[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga33[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga33[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga33[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga33[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga33[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga33[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga33[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga33[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga33[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga33[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga33[2][2] = doRuchuPrawa[2][2];

  // Podniesienie P0
  bazowanieLewaNoga41[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga41[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga41[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga41[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga41[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga41[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga41[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga41[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga41[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga41[0][0] = doRuchuPrawa[0][0];
  bazowaniePrawaNoga41[0][1] = doRuchuPrawa[0][1];
  bazowaniePrawaNoga41[0][2] = doRuchuPrawa[0][2] + ZG;
  bazowaniePrawaNoga41[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga41[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga41[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga41[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga41[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga41[2][2] = doRuchuPrawa[2][2];

  // Przesunięcie w powietrzu P0
  bazowanieLewaNoga42[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga42[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga42[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga42[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga42[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga42[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga42[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga42[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga42[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga42[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga42[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga42[0][2] = doRuchuPrawa[0][2] + ZG;
  bazowaniePrawaNoga42[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga42[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga42[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga42[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga42[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga42[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie P0
  bazowanieLewaNoga43[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga43[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga43[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga43[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga43[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga43[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga43[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga43[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga43[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga43[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga43[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga43[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga43[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga43[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga43[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga43[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga43[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga43[2][2] = doRuchuPrawa[2][2];

  // Podniesienie P1
  bazowanieLewaNoga51[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga51[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga51[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga51[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga51[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga51[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga51[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga51[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga51[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga51[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga51[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga51[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga51[1][0] = doRuchuPrawa[1][0];
  bazowaniePrawaNoga51[1][1] = doRuchuPrawa[1][1];
  bazowaniePrawaNoga51[1][2] = doRuchuPrawa[1][2] + ZG;
  bazowaniePrawaNoga51[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga51[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga51[2][2] = doRuchuPrawa[2][2];

  // Przesunięcie w powietrzu P1
  bazowanieLewaNoga52[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga52[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga52[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga52[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga52[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga52[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga52[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga52[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga52[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga52[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga52[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga52[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga52[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  bazowaniePrawaNoga52[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  bazowaniePrawaNoga52[1][2] = doRuchuPrawa[1][2] + ZG;
  bazowaniePrawaNoga52[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga52[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga52[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie P1
  bazowanieLewaNoga53[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga53[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga53[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga53[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga53[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga53[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga53[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga53[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga53[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga53[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga53[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga53[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga53[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  bazowaniePrawaNoga53[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  bazowaniePrawaNoga53[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga53[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga53[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga53[2][2] = doRuchuPrawa[2][2];

  // Podniesienie P2
  bazowanieLewaNoga61[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga61[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga61[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga61[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga61[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga61[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga61[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga61[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga61[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga61[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga61[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga61[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga61[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  bazowaniePrawaNoga61[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  bazowaniePrawaNoga61[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga61[2][0] = doRuchuPrawa[2][0];
  bazowaniePrawaNoga61[2][1] = doRuchuPrawa[2][1];
  bazowaniePrawaNoga61[2][2] = doRuchuPrawa[2][2] + ZG;

  // Przesunięcie w powietrzu P2
  bazowanieLewaNoga62[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga62[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga62[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga62[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga62[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga62[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga62[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga62[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga62[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga62[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga62[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga62[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga62[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  bazowaniePrawaNoga62[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  bazowaniePrawaNoga62[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga62[2][0] = doRuchuPrawa[2][0] - suniecieX;
  bazowaniePrawaNoga62[2][1] = doRuchuPrawa[2][1] + suniecieY;
  bazowaniePrawaNoga62[2][2] = doRuchuPrawa[2][2] + ZG;

  // Opuszczenie P2
  bazowanieLewaNoga63[0][0] = doRuchuLewa[0][0] + suniecieX;
  bazowanieLewaNoga63[0][1] = doRuchuLewa[0][1] - suniecieY;
  bazowanieLewaNoga63[0][2] = doRuchuLewa[0][2];
  bazowanieLewaNoga63[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  bazowanieLewaNoga63[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  bazowanieLewaNoga63[1][2] = doRuchuLewa[1][2];
  bazowanieLewaNoga63[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  bazowanieLewaNoga63[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  bazowanieLewaNoga63[2][2] = doRuchuLewa[2][2];

  bazowaniePrawaNoga63[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  bazowaniePrawaNoga63[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  bazowaniePrawaNoga63[0][2] = doRuchuPrawa[0][2];
  bazowaniePrawaNoga63[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  bazowaniePrawaNoga63[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  bazowaniePrawaNoga63[1][2] = doRuchuPrawa[1][2];
  bazowaniePrawaNoga63[2][0] = doRuchuPrawa[2][0] - suniecieX;
  bazowaniePrawaNoga63[2][1] = doRuchuPrawa[2][1] + suniecieY;
  bazowaniePrawaNoga63[2][2] = doRuchuPrawa[2][2];
}

// Funkcja realizjąca ruch robota, przy poruszaniu tylko 1 z nóg na raz
void ruchPojedynczejNogi(){
  float suniecieX = sinKatRuchu*suniecie;
  float suniecieY = cosKatRuchu*suniecie;

  // Podniesienie L0
  ruchLewaNoga11[0][0] = doRuchuLewa[0][0] + suniecieX;
  ruchLewaNoga11[0][1] = doRuchuLewa[0][1] - suniecieY;
  ruchLewaNoga11[0][2] = doRuchuLewa[0][2] + ZG;
  ruchLewaNoga11[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  ruchLewaNoga11[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  ruchLewaNoga11[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga11[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  ruchLewaNoga11[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  ruchLewaNoga11[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga11[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  ruchPrawaNoga11[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  ruchPrawaNoga11[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga11[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  ruchPrawaNoga11[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  ruchPrawaNoga11[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga11[2][0] = doRuchuPrawa[2][0] - suniecieX;
  ruchPrawaNoga11[2][1] = doRuchuPrawa[2][1] + suniecieY;
  ruchPrawaNoga11[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu L0
  ruchLewaNoga12[0][0] = doRuchuLewa[0][0] - suniecieX;
  ruchLewaNoga12[0][1] = doRuchuLewa[0][1] + suniecieY;
  ruchLewaNoga12[0][2] = doRuchuLewa[0][2] + ZG;
  ruchLewaNoga12[1][0] = doRuchuLewa[1][0] + suniecieX;
  ruchLewaNoga12[1][1] = doRuchuLewa[1][1] - suniecieY;
  ruchLewaNoga12[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga12[2][0] = doRuchuLewa[2][0] + suniecieX*3/5;
  ruchLewaNoga12[2][1] = doRuchuLewa[2][1] - suniecieY*3/5;
  ruchLewaNoga12[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga12[0][0] = doRuchuPrawa[0][0] + suniecieX/5;
  ruchPrawaNoga12[0][1] = doRuchuPrawa[0][1] - suniecieY/5;
  ruchPrawaNoga12[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga12[1][0] = doRuchuPrawa[1][0] - suniecieX/5;
  ruchPrawaNoga12[1][1] = doRuchuPrawa[1][1] + suniecieY/5;
  ruchPrawaNoga12[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga12[2][0] = doRuchuPrawa[2][0] - suniecieX*3/5;
  ruchPrawaNoga12[2][1] = doRuchuPrawa[2][1] + suniecieY*3/5;
  ruchPrawaNoga12[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L0
  ruchLewaNoga13[0][0] = doRuchuLewa[0][0] - suniecieX;
  ruchLewaNoga13[0][1] = doRuchuLewa[0][1] + suniecieY;
  ruchLewaNoga13[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga13[1][0] = doRuchuLewa[1][0] + suniecieX;
  ruchLewaNoga13[1][1] = doRuchuLewa[1][1] - suniecieY;
  ruchLewaNoga13[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga13[2][0] = doRuchuLewa[2][0] + suniecieX*3/5;
  ruchLewaNoga13[2][1] = doRuchuLewa[2][1] - suniecieY*3/5;
  ruchLewaNoga13[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga13[0][0] = doRuchuPrawa[0][0] + suniecieX/5;
  ruchPrawaNoga13[0][1] = doRuchuPrawa[0][1] - suniecieY/5;
  ruchPrawaNoga13[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga13[1][0] = doRuchuPrawa[1][0] - suniecieX/5;
  ruchPrawaNoga13[1][1] = doRuchuPrawa[1][1] + suniecieY/5;
  ruchPrawaNoga13[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga13[2][0] = doRuchuPrawa[2][0] - suniecieX*3/5;
  ruchPrawaNoga13[2][1] = doRuchuPrawa[2][1] + suniecieY*3/5;
  ruchPrawaNoga13[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L1
  ruchLewaNoga21[0][0] = doRuchuLewa[0][0] - suniecieX;
  ruchLewaNoga21[0][1] = doRuchuLewa[0][1] + suniecieY;
  ruchLewaNoga21[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga21[1][0] = doRuchuLewa[1][0] + suniecieX;
  ruchLewaNoga21[1][1] = doRuchuLewa[1][1] - suniecieY;
  ruchLewaNoga21[1][2] = doRuchuLewa[1][2] + ZG;
  ruchLewaNoga21[2][0] = doRuchuLewa[2][0] + suniecieX*3/5;
  ruchLewaNoga21[2][1] = doRuchuLewa[2][1] - suniecieX*3/5;
  ruchLewaNoga21[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga21[0][0] = doRuchuPrawa[0][0] + suniecieX/5;
  ruchPrawaNoga21[0][1] = doRuchuPrawa[0][1] - suniecieY/5;
  ruchPrawaNoga21[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga21[1][0] = doRuchuPrawa[1][0] - suniecieX/5;
  ruchPrawaNoga21[1][1] = doRuchuPrawa[1][1] + suniecieY/5;
  ruchPrawaNoga21[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga21[2][0] = doRuchuPrawa[2][0] - suniecieX*3/5;
  ruchPrawaNoga21[2][1] = doRuchuPrawa[2][1] + suniecieY*3/5;
  ruchPrawaNoga21[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu L1
  ruchLewaNoga22[0][0] = doRuchuLewa[0][0] - suniecieX*3/5;
  ruchLewaNoga22[0][1] = doRuchuLewa[0][1] + suniecieY*3/5;
  ruchLewaNoga22[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga22[1][0] = doRuchuLewa[1][0] - suniecieX;
  ruchLewaNoga22[1][1] = doRuchuLewa[1][1] + suniecieY;
  ruchLewaNoga22[1][2] = doRuchuLewa[1][2] + ZG;
  ruchLewaNoga22[2][0] = doRuchuLewa[2][0] + suniecieX;
  ruchLewaNoga22[2][1] = doRuchuLewa[2][1] - suniecieY;
  ruchLewaNoga22[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga22[0][0] = doRuchuPrawa[0][0] + suniecieX*3/5;
  ruchPrawaNoga22[0][1] = doRuchuPrawa[0][1] - suniecieY*3/5;
  ruchPrawaNoga22[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga22[1][0] = doRuchuPrawa[1][0] + suniecieX/5;
  ruchPrawaNoga22[1][1] = doRuchuPrawa[1][1] - suniecieY/5;
  ruchPrawaNoga22[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga22[2][0] = doRuchuPrawa[2][0] - suniecieX/5;
  ruchPrawaNoga22[2][1] = doRuchuPrawa[2][1] + suniecieY/5;
  ruchPrawaNoga22[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L1
  ruchLewaNoga23[0][0] = doRuchuLewa[0][0] - suniecieX*3/5;
  ruchLewaNoga23[0][1] = doRuchuLewa[0][1] + suniecieY*3/5;
  ruchLewaNoga23[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga23[1][0] = doRuchuLewa[1][0] - suniecieX;
  ruchLewaNoga23[1][1] = doRuchuLewa[1][1] + suniecieY;
  ruchLewaNoga23[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga23[2][0] = doRuchuLewa[2][0] + suniecieX;
  ruchLewaNoga23[2][1] = doRuchuLewa[2][1] - suniecieY;
  ruchLewaNoga23[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga23[0][0] = doRuchuPrawa[0][0] + suniecieX*3/5;
  ruchPrawaNoga23[0][1] = doRuchuPrawa[0][1] - suniecieY*3/5;
  ruchPrawaNoga23[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga23[1][0] = doRuchuPrawa[1][0] + suniecieX/5;
  ruchPrawaNoga23[1][1] = doRuchuPrawa[1][1] - suniecieY/5;
  ruchPrawaNoga23[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga23[2][0] = doRuchuPrawa[2][0] - suniecieX/5;
  ruchPrawaNoga23[2][1] = doRuchuPrawa[2][1] + suniecieY/5;
  ruchPrawaNoga23[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L2
  ruchLewaNoga31[0][0] = doRuchuLewa[0][0] - suniecieX*3/5;
  ruchLewaNoga31[0][1] = doRuchuLewa[0][1] + suniecieY*3/5;
  ruchLewaNoga31[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga31[1][0] = doRuchuLewa[1][0] - suniecieX;
  ruchLewaNoga31[1][1] = doRuchuLewa[1][1] + suniecieY;
  ruchLewaNoga31[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga31[2][0] = doRuchuLewa[2][0] + suniecieX;
  ruchLewaNoga31[2][1] = doRuchuLewa[2][1] - suniecieY;
  ruchLewaNoga31[2][2] = doRuchuLewa[2][2] + ZG;

  ruchPrawaNoga31[0][0] = doRuchuPrawa[0][0] + suniecieX*3/5;
  ruchPrawaNoga31[0][1] = doRuchuPrawa[0][1] - suniecieY*3/5;
  ruchPrawaNoga31[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga31[1][0] = doRuchuPrawa[1][0] + suniecieX/5;
  ruchPrawaNoga31[1][1] = doRuchuPrawa[1][1] - suniecieY/5;
  ruchPrawaNoga31[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga31[2][0] = doRuchuPrawa[2][0] - suniecieX/5;
  ruchPrawaNoga31[2][1] = doRuchuPrawa[2][1] + suniecieY/5;
  ruchPrawaNoga31[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu L2
  ruchLewaNoga32[0][0] = doRuchuLewa[0][0] - suniecieX/5;
  ruchLewaNoga32[0][1] = doRuchuLewa[0][1] + suniecieY/5;
  ruchLewaNoga32[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga32[1][0] = doRuchuLewa[1][0] - suniecieX*3/5;
  ruchLewaNoga32[1][1] = doRuchuLewa[1][1] + suniecieY*3/5;
  ruchLewaNoga32[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga32[2][0] = doRuchuLewa[2][0] - suniecieX;
  ruchLewaNoga32[2][1] = doRuchuLewa[2][1] + suniecieY;
  ruchLewaNoga32[2][2] = doRuchuLewa[2][2] + ZG;

  ruchPrawaNoga32[0][0] = doRuchuPrawa[0][0] + suniecieX;
  ruchPrawaNoga32[0][1] = doRuchuPrawa[0][1] - suniecieY;
  ruchPrawaNoga32[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga32[1][0] = doRuchuPrawa[1][0] + suniecieX*3/5;
  ruchPrawaNoga32[1][1] = doRuchuPrawa[1][1] - suniecieY*3/5;
  ruchPrawaNoga32[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga32[2][0] = doRuchuPrawa[2][0] + suniecieX/5;
  ruchPrawaNoga32[2][1] = doRuchuPrawa[2][1] - suniecieY/5;
  ruchPrawaNoga32[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L2
  ruchLewaNoga33[0][0] = doRuchuLewa[0][0] - suniecieX/5;
  ruchLewaNoga33[0][1] = doRuchuLewa[0][1] + suniecieY/5;
  ruchLewaNoga33[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga33[1][0] = doRuchuLewa[1][0] - suniecieX*3/5;
  ruchLewaNoga33[1][1] = doRuchuLewa[1][1] + suniecieY*3/5;
  ruchLewaNoga33[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga33[2][0] = doRuchuLewa[2][0] - suniecieX;
  ruchLewaNoga33[2][1] = doRuchuLewa[2][1] + suniecieY;
  ruchLewaNoga33[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga33[0][0] = doRuchuPrawa[0][0] + suniecieX;
  ruchPrawaNoga33[0][1] = doRuchuPrawa[0][1] - suniecieY;
  ruchPrawaNoga33[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga33[1][0] = doRuchuPrawa[1][0] + suniecieX*3/5;
  ruchPrawaNoga33[1][1] = doRuchuPrawa[1][1] - suniecieY*3/5;
  ruchPrawaNoga33[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga33[2][0] = doRuchuPrawa[2][0] + suniecieX/5;
  ruchPrawaNoga33[2][1] = doRuchuPrawa[2][1] - suniecieY/5;
  ruchPrawaNoga33[2][2] = doRuchuPrawa[2][2];

  // Podniesienie P0
  ruchLewaNoga41[0][0] = doRuchuLewa[0][0] - suniecieX/5;
  ruchLewaNoga41[0][1] = doRuchuLewa[0][1] + suniecieY/5;
  ruchLewaNoga41[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga41[1][0] = doRuchuLewa[1][0] - suniecieX*3/5;
  ruchLewaNoga41[1][1] = doRuchuLewa[1][1] + suniecieY*3/5;
  ruchLewaNoga41[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga41[2][0] = doRuchuLewa[2][0] - suniecieX;
  ruchLewaNoga41[2][1] = doRuchuLewa[2][1] + suniecieY;
  ruchLewaNoga41[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga41[0][0] = doRuchuPrawa[0][0] + suniecieX;
  ruchPrawaNoga41[0][1] = doRuchuPrawa[0][1] - suniecieY;
  ruchPrawaNoga41[0][2] = doRuchuPrawa[0][2] + ZG;
  ruchPrawaNoga41[1][0] = doRuchuPrawa[1][0] + suniecieX*3/5;
  ruchPrawaNoga41[1][1] = doRuchuPrawa[1][1] - suniecieY*3/5;
  ruchPrawaNoga41[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga41[2][0] = doRuchuPrawa[2][0] + suniecieX/5;
  ruchPrawaNoga41[2][1] = doRuchuPrawa[2][1] - suniecieY/5;
  ruchPrawaNoga41[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu P0
  ruchLewaNoga42[0][0] = doRuchuLewa[0][0] + suniecieX/5;
  ruchLewaNoga42[0][1] = doRuchuLewa[0][1] - suniecieY/5;
  ruchLewaNoga42[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga42[1][0] = doRuchuLewa[1][0] - suniecieX/5;
  ruchLewaNoga42[1][1] = doRuchuLewa[1][1] + suniecieY/5;
  ruchLewaNoga42[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga42[2][0] = doRuchuLewa[2][0] - suniecieX*3/5;
  ruchLewaNoga42[2][1] = doRuchuLewa[2][1] + suniecieY*3/5;
  ruchLewaNoga42[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga42[0][0] = doRuchuPrawa[0][0] - suniecieX;
  ruchPrawaNoga42[0][1] = doRuchuPrawa[0][1] + suniecieY;
  ruchPrawaNoga42[0][2] = doRuchuPrawa[0][2] + ZG;
  ruchPrawaNoga42[1][0] = doRuchuPrawa[1][0] + suniecieX;
  ruchPrawaNoga42[1][1] = doRuchuPrawa[1][1] - suniecieY;
  ruchPrawaNoga42[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga42[2][0] = doRuchuPrawa[2][0] + suniecieX*3/5;
  ruchPrawaNoga42[2][1] = doRuchuPrawa[2][1] - suniecieY*3/5;
  ruchPrawaNoga42[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie P0
  ruchLewaNoga43[0][0] = doRuchuLewa[0][0] + suniecieX/5;
  ruchLewaNoga43[0][1] = doRuchuLewa[0][1] - suniecieY/5;
  ruchLewaNoga43[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga43[1][0] = doRuchuLewa[1][0] - suniecieX/5;
  ruchLewaNoga43[1][1] = doRuchuLewa[1][1] + suniecieY/5;
  ruchLewaNoga43[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga43[2][0] = doRuchuLewa[2][0] - suniecieX*3/5;
  ruchLewaNoga43[2][1] = doRuchuLewa[2][1] + suniecieY*3/5;
  ruchLewaNoga43[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga43[0][0] = doRuchuPrawa[0][0] - suniecieX;
  ruchPrawaNoga43[0][1] = doRuchuPrawa[0][1] + suniecieY;
  ruchPrawaNoga43[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga43[1][0] = doRuchuPrawa[1][0] + suniecieX;
  ruchPrawaNoga43[1][1] = doRuchuPrawa[1][1] - suniecieY;
  ruchPrawaNoga43[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga43[2][0] = doRuchuPrawa[2][0] + suniecieX*3/5;
  ruchPrawaNoga43[2][1] = doRuchuPrawa[2][1] - suniecieY*3/5;
  ruchPrawaNoga43[2][2] = doRuchuPrawa[2][2];

  // Podniesienie P1
  ruchLewaNoga51[0][0] = doRuchuLewa[0][0] + suniecieX/5;
  ruchLewaNoga51[0][1] = doRuchuLewa[0][1] - suniecieY/5;
  ruchLewaNoga51[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga51[1][0] = doRuchuLewa[1][0] - suniecieX/5;
  ruchLewaNoga51[1][1] = doRuchuLewa[1][1] + suniecieY/5;
  ruchLewaNoga51[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga51[2][0] = doRuchuLewa[2][0] - suniecieX*3/5;
  ruchLewaNoga51[2][1] = doRuchuLewa[2][1] + suniecieY*3/5;
  ruchLewaNoga51[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga51[0][0] = doRuchuPrawa[0][0] - suniecieX;
  ruchPrawaNoga51[0][1] = doRuchuPrawa[0][1] + suniecieY;
  ruchPrawaNoga51[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga51[1][0] = doRuchuPrawa[1][0] + suniecieX;
  ruchPrawaNoga51[1][1] = doRuchuPrawa[1][1] - suniecieY;
  ruchPrawaNoga51[1][2] = doRuchuPrawa[1][2] + ZG;
  ruchPrawaNoga51[2][0] = doRuchuPrawa[2][0] + suniecieX*3/5;
  ruchPrawaNoga51[2][1] = doRuchuPrawa[2][1] - suniecieY*3/5;
  ruchPrawaNoga51[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu P1
  ruchLewaNoga52[0][0] = doRuchuLewa[0][0] + suniecieX*3/5;
  ruchLewaNoga52[0][1] = doRuchuLewa[0][1] - suniecieY*3/5;
  ruchLewaNoga52[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga52[1][0] = doRuchuLewa[1][0] + suniecieX/5;
  ruchLewaNoga52[1][1] = doRuchuLewa[1][1] - suniecieY/5;
  ruchLewaNoga52[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga52[2][0] = doRuchuLewa[2][0] - suniecieX/5;
  ruchLewaNoga52[2][1] = doRuchuLewa[2][1] + suniecieY/5;
  ruchLewaNoga52[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga52[0][0] = doRuchuPrawa[0][0] - suniecieX*3/5;
  ruchPrawaNoga52[0][1] = doRuchuPrawa[0][1] + suniecieY*3/5;
  ruchPrawaNoga52[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga52[1][0] = doRuchuPrawa[1][0] - suniecieX;
  ruchPrawaNoga52[1][1] = doRuchuPrawa[1][1] + suniecieY;
  ruchPrawaNoga52[1][2] = doRuchuPrawa[1][2] + ZG;
  ruchPrawaNoga52[2][0] = doRuchuPrawa[2][0] + suniecieX;
  ruchPrawaNoga52[2][1] = doRuchuPrawa[2][1] - suniecieY;
  ruchPrawaNoga52[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie P1
  ruchLewaNoga53[0][0] = doRuchuLewa[0][0] + suniecieX*3/5;
  ruchLewaNoga53[0][1] = doRuchuLewa[0][1] - suniecieY*3/5;
  ruchLewaNoga53[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga53[1][0] = doRuchuLewa[1][0] + suniecieX/5;
  ruchLewaNoga53[1][1] = doRuchuLewa[1][1] - suniecieY/5;
  ruchLewaNoga53[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga53[2][0] = doRuchuLewa[2][0] - suniecieX/5;
  ruchLewaNoga53[2][1] = doRuchuLewa[2][1] + suniecieY/5;
  ruchLewaNoga53[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga53[0][0] = doRuchuPrawa[0][0] - suniecieX*3/5;
  ruchPrawaNoga53[0][1] = doRuchuPrawa[0][1] + suniecieY*3/5;
  ruchPrawaNoga53[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga53[1][0] = doRuchuPrawa[1][0] - suniecieX;
  ruchPrawaNoga53[1][1] = doRuchuPrawa[1][1] + suniecieY;
  ruchPrawaNoga53[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga53[2][0] = doRuchuPrawa[2][0] + suniecieX;
  ruchPrawaNoga53[2][1] = doRuchuPrawa[2][1] - suniecieY;
  ruchPrawaNoga53[2][2] = doRuchuPrawa[2][2];

  // Podniesienie P2
  ruchLewaNoga61[0][0] = doRuchuLewa[0][0] + suniecieX*3/5;
  ruchLewaNoga61[0][1] = doRuchuLewa[0][1] - suniecieY*3/5;
  ruchLewaNoga61[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga61[1][0] = doRuchuLewa[1][0] + suniecieX/5;
  ruchLewaNoga61[1][1] = doRuchuLewa[1][1] - suniecieY/5;
  ruchLewaNoga61[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga61[2][0] = doRuchuLewa[2][0] - suniecieX/5;
  ruchLewaNoga61[2][1] = doRuchuLewa[2][1] + suniecieY/5;
  ruchLewaNoga61[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga61[0][0] = doRuchuPrawa[0][0] - suniecieX*3/5;
  ruchPrawaNoga61[0][1] = doRuchuPrawa[0][1] + suniecieY*3/5;
  ruchPrawaNoga61[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga61[1][0] = doRuchuPrawa[1][0] - suniecieX;
  ruchPrawaNoga61[1][1] = doRuchuPrawa[1][1] + suniecieY;
  ruchPrawaNoga61[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga61[2][0] = doRuchuPrawa[2][0] + suniecieX;
  ruchPrawaNoga61[2][1] = doRuchuPrawa[2][1] - suniecieY;
  ruchPrawaNoga61[2][2] = doRuchuPrawa[2][2] + ZG;

  // Przesuniecie w powietrzu P2
  ruchLewaNoga62[0][0] = doRuchuLewa[0][0] + suniecieX;
  ruchLewaNoga62[0][1] = doRuchuLewa[0][1] - suniecieY;
  ruchLewaNoga62[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga62[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  ruchLewaNoga62[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  ruchLewaNoga62[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga62[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  ruchLewaNoga62[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  ruchLewaNoga62[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga62[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  ruchPrawaNoga62[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  ruchPrawaNoga62[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga62[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  ruchPrawaNoga62[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  ruchPrawaNoga62[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga62[2][0] = doRuchuPrawa[2][0] - suniecieX;
  ruchPrawaNoga62[2][1] = doRuchuPrawa[2][1] + suniecieY;
  ruchPrawaNoga62[2][2] = doRuchuPrawa[2][2] + ZG;

  // Opuszczenie P2
  ruchLewaNoga63[0][0] = doRuchuLewa[0][0] + suniecieX;
  ruchLewaNoga63[0][1] = doRuchuLewa[0][1] - suniecieY;
  ruchLewaNoga63[0][2] = doRuchuLewa[0][2];
  ruchLewaNoga63[1][0] = doRuchuLewa[1][0] + suniecieX*3/5;
  ruchLewaNoga63[1][1] = doRuchuLewa[1][1] - suniecieY*3/5;
  ruchLewaNoga63[1][2] = doRuchuLewa[1][2];
  ruchLewaNoga63[2][0] = doRuchuLewa[2][0] + suniecieX/5;
  ruchLewaNoga63[2][1] = doRuchuLewa[2][1] - suniecieY/5;
  ruchLewaNoga63[2][2] = doRuchuLewa[2][2];

  ruchPrawaNoga63[0][0] = doRuchuPrawa[0][0] - suniecieX/5;
  ruchPrawaNoga63[0][1] = doRuchuPrawa[0][1] + suniecieY/5;
  ruchPrawaNoga63[0][2] = doRuchuPrawa[0][2];
  ruchPrawaNoga63[1][0] = doRuchuPrawa[1][0] - suniecieX*3/5;
  ruchPrawaNoga63[1][1] = doRuchuPrawa[1][1] + suniecieY*3/5;
  ruchPrawaNoga63[1][2] = doRuchuPrawa[1][2];
  ruchPrawaNoga63[2][0] = doRuchuPrawa[2][0] - suniecieX;
  ruchPrawaNoga63[2][1] = doRuchuPrawa[2][1] + suniecieY;
  ruchPrawaNoga63[2][2] = doRuchuPrawa[2][2];
}

void polozenieNogi(char strona, int numerNogi){
  Serial.print("Polozenie ");
  Serial.print(strona);
  Serial.print(numerNogi);
  Serial.print(" X:");
  if (strona == 'L'){
    Serial.print(polozenieLewa[numerNogi][0]);
    Serial.print(", Y:");
    Serial.print(polozenieLewa[numerNogi][1]);
    Serial.print(", Z:");
    Serial.println(polozenieLewa[numerNogi][2]);
  }

  if (strona == 'P'){
    Serial.print(polozeniePrawa[numerNogi][0]);
    Serial.print(", Y:");
    Serial.print(polozeniePrawa[numerNogi][1]);
    Serial.print(", Z:");
    Serial.println(polozeniePrawa[numerNogi][2]);
  }
}

void ruchTrojpodporowy(){
  float suniecieX = sinKatRuchu*suniecie;
  float suniecieY = cosKatRuchu*suniecie;
  Serial.print("SuniecieX: ");
  Serial.println(suniecieX);
  Serial.print("SuniecieY: ");
  Serial.println(suniecieY);

  // Chyba niepotrzebne te podniesienie
  // Podniesienie L0 P1 L2
  trojpodporowyLewa11[0][0] = polozenieLewa[0][0];
  trojpodporowyLewa11[0][1] = polozenieLewa[0][1];
  trojpodporowyLewa11[0][2] = polozenieLewa[0][2] + ZG;
  trojpodporowyLewa11[1][0] = polozenieLewa[1][0];
  trojpodporowyLewa11[1][1] = polozenieLewa[1][1];
  trojpodporowyLewa11[1][2] = polozenieLewa[1][2];
  trojpodporowyLewa11[2][0] = polozenieLewa[2][0];
  trojpodporowyLewa11[2][1] = polozenieLewa[2][1];
  trojpodporowyLewa11[2][2] = polozenieLewa[2][2] + ZG;

  trojpodporowyPrawa11[0][0] = polozeniePrawa[0][0];
  trojpodporowyPrawa11[0][1] = polozeniePrawa[0][1];
  trojpodporowyPrawa11[0][2] = polozeniePrawa[0][2];
  trojpodporowyPrawa11[1][0] = polozeniePrawa[1][0];
  trojpodporowyPrawa11[1][1] = polozeniePrawa[1][1]; 
  trojpodporowyPrawa11[1][2] = polozeniePrawa[1][2] + ZG;
  trojpodporowyPrawa11[2][0] = polozeniePrawa[2][0];
  trojpodporowyPrawa11[2][1] = polozeniePrawa[2][1];
  trojpodporowyPrawa11[2][2] = polozeniePrawa[2][2];

  // Przesuniecie L0 P1 L2
  trojpodporowyLewa12[0][0] = doRuchuLewa[0][0] - suniecieX;
  trojpodporowyLewa12[0][1] = doRuchuLewa[0][1] + suniecieY;
  trojpodporowyLewa12[0][2] = doRuchuLewa[0][2] + ZG;
  trojpodporowyLewa12[1][0] = doRuchuLewa[1][0] + suniecieX;
  trojpodporowyLewa12[1][1] = doRuchuLewa[1][1] - suniecieY;
  trojpodporowyLewa12[1][2] = doRuchuLewa[1][2];
  trojpodporowyLewa12[2][0] = doRuchuLewa[2][0] - suniecieX;
  trojpodporowyLewa12[2][1] = doRuchuLewa[2][1] + suniecieY;
  trojpodporowyLewa12[2][2] = doRuchuLewa[2][2] + ZG;

  trojpodporowyPrawa12[0][0] = doRuchuPrawa[0][0] + suniecieX;
  trojpodporowyPrawa12[0][1] = doRuchuPrawa[0][1] - suniecieY;
  trojpodporowyPrawa12[0][2] = doRuchuPrawa[0][2];
  trojpodporowyPrawa12[1][0] = doRuchuPrawa[1][0] - suniecieX;
  trojpodporowyPrawa12[1][1] = doRuchuPrawa[1][1] + suniecieY; 
  trojpodporowyPrawa12[1][2] = doRuchuPrawa[1][2] + ZG;
  trojpodporowyPrawa12[2][0] = doRuchuPrawa[2][0] + suniecieX;
  trojpodporowyPrawa12[2][1] = doRuchuPrawa[2][1] - suniecieY;
  trojpodporowyPrawa12[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L0 P1 L2
  trojpodporowyLewa13[0][0] = doRuchuLewa[0][0] - suniecieX;
  trojpodporowyLewa13[0][1] = doRuchuLewa[0][1] + suniecieY;
  trojpodporowyLewa13[0][2] = doRuchuLewa[0][2];
  trojpodporowyLewa13[1][0] = doRuchuLewa[1][0] + suniecieX;
  trojpodporowyLewa13[1][1] = doRuchuLewa[1][1] - suniecieY;
  trojpodporowyLewa13[1][2] = doRuchuLewa[1][2];
  trojpodporowyLewa13[2][0] = doRuchuLewa[2][0] - suniecieX;
  trojpodporowyLewa13[2][1] = doRuchuLewa[2][1] + suniecieY;
  trojpodporowyLewa13[2][2] = doRuchuLewa[2][2];

  trojpodporowyPrawa13[0][0] = doRuchuPrawa[0][0] + suniecieX;
  trojpodporowyPrawa13[0][1] = doRuchuPrawa[0][1] - suniecieY;
  trojpodporowyPrawa13[0][2] = doRuchuPrawa[0][2];
  trojpodporowyPrawa13[1][0] = doRuchuPrawa[1][0] - suniecieX;
  trojpodporowyPrawa13[1][1] = doRuchuPrawa[1][1] + suniecieY; 
  trojpodporowyPrawa13[1][2] = doRuchuPrawa[1][2];
  trojpodporowyPrawa13[2][0] = doRuchuPrawa[2][0] + suniecieX;
  trojpodporowyPrawa13[2][1] = doRuchuPrawa[2][1] - suniecieY;
  trojpodporowyPrawa13[2][2] = doRuchuPrawa[2][2];

  // Podniesienie P0 L1 P2
  trojpodporowyLewa21[0][0] = doRuchuLewa[0][0] - suniecieX;
  trojpodporowyLewa21[0][1] = doRuchuLewa[0][1] + suniecieY;
  trojpodporowyLewa21[0][2] = doRuchuLewa[0][2];
  trojpodporowyLewa21[1][0] = doRuchuLewa[1][0] + suniecieX;
  trojpodporowyLewa21[1][1] = doRuchuLewa[1][1] - suniecieY;
  trojpodporowyLewa21[1][2] = doRuchuLewa[1][2] + ZG;
  trojpodporowyLewa21[2][0] = doRuchuLewa[2][0] - suniecieX;
  trojpodporowyLewa21[2][1] = doRuchuLewa[2][1] + suniecieY;
  trojpodporowyLewa21[2][2] = doRuchuLewa[2][2];

  trojpodporowyPrawa21[0][0] = doRuchuPrawa[0][0] + suniecieX;
  trojpodporowyPrawa21[0][1] = doRuchuPrawa[0][1] - suniecieY;
  trojpodporowyPrawa21[0][2] = doRuchuPrawa[0][2] + ZG;
  trojpodporowyPrawa21[1][0] = doRuchuPrawa[1][0] - suniecieX;
  trojpodporowyPrawa21[1][1] = doRuchuPrawa[1][1] + suniecieY; 
  trojpodporowyPrawa21[1][2] = doRuchuPrawa[1][2];
  trojpodporowyPrawa21[2][0] = doRuchuPrawa[2][0] + suniecieX;
  trojpodporowyPrawa21[2][1] = doRuchuPrawa[2][1] - suniecieY;
  trojpodporowyPrawa21[2][2] = doRuchuPrawa[2][2] + ZG;

  // Przesuniecie P0 L1 P2
  trojpodporowyLewa22[0][0] = doRuchuLewa[0][0] + suniecieX;
  trojpodporowyLewa22[0][1] = doRuchuLewa[0][1] - suniecieY;
  trojpodporowyLewa22[0][2] = doRuchuLewa[0][2];
  trojpodporowyLewa22[1][0] = doRuchuLewa[1][0] - suniecieX;
  trojpodporowyLewa22[1][1] = doRuchuLewa[1][1] + suniecieY;
  trojpodporowyLewa22[1][2] = doRuchuLewa[1][2] + ZG;
  trojpodporowyLewa22[2][0] = doRuchuLewa[2][0] + suniecieX;
  trojpodporowyLewa22[2][1] = doRuchuLewa[2][1] - suniecieY;
  trojpodporowyLewa22[2][2] = doRuchuLewa[2][2];

  trojpodporowyPrawa22[0][0] = doRuchuPrawa[0][0] - suniecieX;
  trojpodporowyPrawa22[0][1] = doRuchuPrawa[0][1] + suniecieY;
  trojpodporowyPrawa22[0][2] = doRuchuPrawa[0][2] + ZG;
  trojpodporowyPrawa22[1][0] = doRuchuPrawa[1][0] + suniecieX;
  trojpodporowyPrawa22[1][1] = doRuchuPrawa[1][1] - suniecieY; 
  trojpodporowyPrawa22[1][2] = doRuchuPrawa[1][2];
  trojpodporowyPrawa22[2][0] = doRuchuPrawa[2][0] - suniecieX;
  trojpodporowyPrawa22[2][1] = doRuchuPrawa[2][1] + suniecieY;
  trojpodporowyPrawa22[2][2] = doRuchuPrawa[2][2] + ZG;

  // Opuszczenie P0 L1 P2
  trojpodporowyLewa23[0][0] = doRuchuLewa[0][0] + suniecieX;
  trojpodporowyLewa23[0][1] = doRuchuLewa[0][1] - suniecieY;
  trojpodporowyLewa23[0][2] = doRuchuLewa[0][2];
  trojpodporowyLewa23[1][0] = doRuchuLewa[1][0] - suniecieX;
  trojpodporowyLewa23[1][1] = doRuchuLewa[1][1] + suniecieY;
  trojpodporowyLewa23[1][2] = doRuchuLewa[1][2];
  trojpodporowyLewa23[2][0] = doRuchuLewa[2][0] + suniecieX;
  trojpodporowyLewa23[2][1] = doRuchuLewa[2][1] - suniecieY;
  trojpodporowyLewa23[2][2] = doRuchuLewa[2][2];

  trojpodporowyPrawa23[0][0] = doRuchuPrawa[0][0] - suniecieX;
  trojpodporowyPrawa23[0][1] = doRuchuPrawa[0][1] + suniecieY;
  trojpodporowyPrawa23[0][2] = doRuchuPrawa[0][2];
  trojpodporowyPrawa23[1][0] = doRuchuPrawa[1][0] + suniecieX;
  trojpodporowyPrawa23[1][1] = doRuchuPrawa[1][1] - suniecieY; 
  trojpodporowyPrawa23[1][2] = doRuchuPrawa[1][2];
  trojpodporowyPrawa23[2][0] = doRuchuPrawa[2][0] - suniecieX;
  trojpodporowyPrawa23[2][1] = doRuchuPrawa[2][1] + suniecieY;
  trojpodporowyPrawa23[2][2] = doRuchuPrawa[2][2];
}

float wstepnyCzteropodporowyLewa11[3][3];
float wstepnyCzteropodporowyLewa12[3][3];
float wstepnyCzteropodporowyLewa13[3][3];
float wstepnyCzteropodporowyLewa21[3][3];
float wstepnyCzteropodporowyLewa22[3][3];
float wstepnyCzteropodporowyLewa23[3][3];
float wstepnyCzteropodporowyLewa31[3][3];
float wstepnyCzteropodporowyLewa32[3][3];
float wstepnyCzteropodporowyLewa33[3][3];

float wstepnyCzteropodporowyPrawa11[3][3];
float wstepnyCzteropodporowyPrawa12[3][3];
float wstepnyCzteropodporowyPrawa13[3][3];
float wstepnyCzteropodporowyPrawa21[3][3];
float wstepnyCzteropodporowyPrawa22[3][3];
float wstepnyCzteropodporowyPrawa23[3][3];
float wstepnyCzteropodporowyPrawa31[3][3];
float wstepnyCzteropodporowyPrawa32[3][3];
float wstepnyCzteropodporowyPrawa33[3][3];

float czteropodporowyLewa11[3][3];
float czteropodporowyLewa12[3][3];
float czteropodporowyLewa13[3][3];
float czteropodporowyLewa21[3][3];
float czteropodporowyLewa22[3][3];
float czteropodporowyLewa23[3][3];
float czteropodporowyLewa31[3][3];
float czteropodporowyLewa32[3][3];
float czteropodporowyLewa33[3][3];

float czteropodporowyPrawa11[3][3];
float czteropodporowyPrawa12[3][3];
float czteropodporowyPrawa13[3][3];
float czteropodporowyPrawa21[3][3];
float czteropodporowyPrawa22[3][3];
float czteropodporowyPrawa23[3][3];
float czteropodporowyPrawa31[3][3];
float czteropodporowyPrawa32[3][3];
float czteropodporowyPrawa33[3][3];

void wstepnyCzteropodporowy(){
  float suniecieX = sinKatRuchu*suniecie;
  float suniecieY = cosKatRuchu*suniecie;

  // Podniesienie L0 P2
  wstepnyCzteropodporowyLewa11[0][0] = polozenieLewa[0][0];
  wstepnyCzteropodporowyLewa11[0][1] = polozenieLewa[0][1];
  wstepnyCzteropodporowyLewa11[0][2] = doRuchuLewa[0][2] + ZG;
  wstepnyCzteropodporowyLewa11[1][0] = polozenieLewa[1][0];
  wstepnyCzteropodporowyLewa11[1][1] = polozenieLewa[1][1];
  wstepnyCzteropodporowyLewa11[1][2] = polozenieLewa[1][2];
  wstepnyCzteropodporowyLewa11[2][0] = polozenieLewa[2][0];
  wstepnyCzteropodporowyLewa11[2][1] = polozenieLewa[2][1];
  wstepnyCzteropodporowyLewa11[2][2] = polozenieLewa[2][2];

  wstepnyCzteropodporowyPrawa11[0][0] = polozeniePrawa[0][0];
  wstepnyCzteropodporowyPrawa11[0][1] = polozeniePrawa[0][1];
  wstepnyCzteropodporowyPrawa11[0][2] = polozeniePrawa[0][2];
  wstepnyCzteropodporowyPrawa11[1][0] = polozeniePrawa[1][0];
  wstepnyCzteropodporowyPrawa11[1][1] = polozeniePrawa[1][1];
  wstepnyCzteropodporowyPrawa11[1][2] = polozeniePrawa[1][2];
  wstepnyCzteropodporowyPrawa11[2][0] = polozeniePrawa[2][0];
  wstepnyCzteropodporowyPrawa11[2][1] = polozeniePrawa[2][1];
  wstepnyCzteropodporowyPrawa11[2][2] = doRuchuPrawa[2][2] + ZG;

  // Przesuniecie w powietrzu L0 P2
  wstepnyCzteropodporowyLewa12[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa12[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa12[0][2] = doRuchuLewa[0][2] + ZG;
  wstepnyCzteropodporowyLewa12[1][0] = polozenieLewa[1][0];
  wstepnyCzteropodporowyLewa12[1][1] = polozenieLewa[1][1];
  wstepnyCzteropodporowyLewa12[1][2] = polozenieLewa[1][2];
  wstepnyCzteropodporowyLewa12[2][0] = polozenieLewa[2][0];
  wstepnyCzteropodporowyLewa12[2][1] = polozenieLewa[2][1];
  wstepnyCzteropodporowyLewa12[2][2] = polozenieLewa[2][2];

  wstepnyCzteropodporowyPrawa12[0][0] = polozeniePrawa[0][0];
  wstepnyCzteropodporowyPrawa12[0][1] = polozeniePrawa[0][1];
  wstepnyCzteropodporowyPrawa12[0][2] = polozeniePrawa[0][2];
  wstepnyCzteropodporowyPrawa12[1][0] = polozeniePrawa[1][0];
  wstepnyCzteropodporowyPrawa12[1][1] = polozeniePrawa[1][1];
  wstepnyCzteropodporowyPrawa12[1][2] = polozeniePrawa[1][2];
  wstepnyCzteropodporowyPrawa12[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa12[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa12[2][2] = doRuchuPrawa[2][2] + ZG;

  // Opuszczenie L0 P2
  wstepnyCzteropodporowyLewa13[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa13[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa13[0][2] = doRuchuLewa[0][2];
  wstepnyCzteropodporowyLewa13[1][0] = polozenieLewa[1][0];
  wstepnyCzteropodporowyLewa13[1][1] = polozenieLewa[1][1];
  wstepnyCzteropodporowyLewa13[1][2] = polozenieLewa[1][2];
  wstepnyCzteropodporowyLewa13[2][0] = polozenieLewa[2][0];
  wstepnyCzteropodporowyLewa13[2][1] = polozenieLewa[2][1];
  wstepnyCzteropodporowyLewa13[2][2] = polozenieLewa[2][2];

  wstepnyCzteropodporowyPrawa13[0][0] = polozeniePrawa[0][0];
  wstepnyCzteropodporowyPrawa13[0][1] = polozeniePrawa[0][1];
  wstepnyCzteropodporowyPrawa13[0][2] = polozeniePrawa[0][2];
  wstepnyCzteropodporowyPrawa13[1][0] = polozeniePrawa[1][0];
  wstepnyCzteropodporowyPrawa13[1][1] = polozeniePrawa[1][1];
  wstepnyCzteropodporowyPrawa13[1][2] = polozeniePrawa[1][2];
  wstepnyCzteropodporowyPrawa13[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa13[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa13[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L1 P1
  wstepnyCzteropodporowyLewa21[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa21[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa21[0][2] = doRuchuLewa[0][2];
  wstepnyCzteropodporowyLewa21[1][0] = polozenieLewa[1][0];
  wstepnyCzteropodporowyLewa21[1][1] = polozenieLewa[1][1];
  wstepnyCzteropodporowyLewa21[1][2] = doRuchuLewa[1][2] + ZG;
  wstepnyCzteropodporowyLewa21[2][0] = polozenieLewa[2][0];
  wstepnyCzteropodporowyLewa21[2][1] = polozenieLewa[2][1];
  wstepnyCzteropodporowyLewa21[2][2] = polozenieLewa[2][2];

  wstepnyCzteropodporowyPrawa21[0][0] = polozeniePrawa[0][0];
  wstepnyCzteropodporowyPrawa21[0][1] = polozeniePrawa[0][1];
  wstepnyCzteropodporowyPrawa21[0][2] = polozeniePrawa[0][2];
  wstepnyCzteropodporowyPrawa21[1][0] = polozeniePrawa[1][0];
  wstepnyCzteropodporowyPrawa21[1][1] = polozeniePrawa[1][1];
  wstepnyCzteropodporowyPrawa21[1][2] = doRuchuPrawa[1][2] + ZG;
  wstepnyCzteropodporowyPrawa21[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa21[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa21[2][2] = doRuchuPrawa[2][2];

   // Przesuniecie w powietrzu L1 P1
  wstepnyCzteropodporowyLewa22[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa22[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa22[0][2] = doRuchuLewa[0][2];
  wstepnyCzteropodporowyLewa22[1][0] = doRuchuLewa[1][0];
  wstepnyCzteropodporowyLewa22[1][1] = doRuchuLewa[1][1];
  wstepnyCzteropodporowyLewa22[1][2] = doRuchuLewa[1][2] + ZG;
  wstepnyCzteropodporowyLewa22[2][0] = polozenieLewa[2][0];
  wstepnyCzteropodporowyLewa22[2][1] = polozenieLewa[2][1];
  wstepnyCzteropodporowyLewa22[2][2] = polozenieLewa[2][2];

  wstepnyCzteropodporowyPrawa22[0][0] = polozeniePrawa[0][0];
  wstepnyCzteropodporowyPrawa22[0][1] = polozeniePrawa[0][1];
  wstepnyCzteropodporowyPrawa22[0][2] = polozeniePrawa[0][2];
  wstepnyCzteropodporowyPrawa22[1][0] = doRuchuPrawa[1][0];
  wstepnyCzteropodporowyPrawa22[1][1] = doRuchuPrawa[1][1];
  wstepnyCzteropodporowyPrawa22[1][2] = doRuchuPrawa[1][2] + ZG;
  wstepnyCzteropodporowyPrawa22[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa22[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa22[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L1 P1
  wstepnyCzteropodporowyLewa23[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa23[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa23[0][2] = doRuchuLewa[0][2];
  wstepnyCzteropodporowyLewa23[1][0] = doRuchuLewa[1][0];
  wstepnyCzteropodporowyLewa23[1][1] = doRuchuLewa[1][1];
  wstepnyCzteropodporowyLewa23[1][2] = doRuchuLewa[1][2];
  wstepnyCzteropodporowyLewa23[2][0] = polozenieLewa[2][0];
  wstepnyCzteropodporowyLewa23[2][1] = polozenieLewa[2][1];
  wstepnyCzteropodporowyLewa23[2][2] = polozenieLewa[2][2];

  wstepnyCzteropodporowyPrawa23[0][0] = polozeniePrawa[0][0];
  wstepnyCzteropodporowyPrawa23[0][1] = polozeniePrawa[0][1];
  wstepnyCzteropodporowyPrawa23[0][2] = polozeniePrawa[0][2];
  wstepnyCzteropodporowyPrawa23[1][0] = doRuchuPrawa[1][0];
  wstepnyCzteropodporowyPrawa23[1][1] = doRuchuPrawa[1][1];
  wstepnyCzteropodporowyPrawa23[1][2] = doRuchuPrawa[1][2];
  wstepnyCzteropodporowyPrawa23[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa23[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa23[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L2 P0
  wstepnyCzteropodporowyLewa31[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa31[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa31[0][2] = doRuchuLewa[0][2];
  wstepnyCzteropodporowyLewa31[1][0] = doRuchuLewa[1][0];
  wstepnyCzteropodporowyLewa31[1][1] = doRuchuLewa[1][1];
  wstepnyCzteropodporowyLewa31[1][2] = doRuchuLewa[1][2];
  wstepnyCzteropodporowyLewa31[2][0] = polozenieLewa[2][0];
  wstepnyCzteropodporowyLewa31[2][1] = polozenieLewa[2][1];
  wstepnyCzteropodporowyLewa31[2][2] = doRuchuLewa[1][2] + ZG;

  wstepnyCzteropodporowyPrawa31[0][0] = polozeniePrawa[0][0];
  wstepnyCzteropodporowyPrawa31[0][1] = polozeniePrawa[0][1];
  wstepnyCzteropodporowyPrawa31[0][2] = doRuchuPrawa[0][2] + ZG;
  wstepnyCzteropodporowyPrawa31[1][0] = doRuchuPrawa[1][0];
  wstepnyCzteropodporowyPrawa31[1][1] = doRuchuPrawa[1][1];
  wstepnyCzteropodporowyPrawa31[1][2] = doRuchuPrawa[1][2];
  wstepnyCzteropodporowyPrawa31[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa31[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa31[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu L2 P0
  wstepnyCzteropodporowyLewa32[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa32[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa32[0][2] = doRuchuLewa[0][2];
  wstepnyCzteropodporowyLewa32[1][0] = doRuchuLewa[1][0];
  wstepnyCzteropodporowyLewa32[1][1] = doRuchuLewa[1][1];
  wstepnyCzteropodporowyLewa32[1][2] = doRuchuLewa[1][2];
  wstepnyCzteropodporowyLewa32[2][0] = doRuchuLewa[2][0] - suniecieX;
  wstepnyCzteropodporowyLewa32[2][1] = doRuchuLewa[2][1] + suniecieY;
  wstepnyCzteropodporowyLewa32[2][2] = doRuchuLewa[1][2] + ZG;

  wstepnyCzteropodporowyPrawa32[0][0] = doRuchuPrawa[0][0] - suniecieX;
  wstepnyCzteropodporowyPrawa32[0][1] = doRuchuPrawa[0][1] + suniecieY;
  wstepnyCzteropodporowyPrawa32[0][2] = doRuchuPrawa[0][2] + ZG;
  wstepnyCzteropodporowyPrawa32[1][0] = doRuchuPrawa[1][0];
  wstepnyCzteropodporowyPrawa32[1][1] = doRuchuPrawa[1][1];
  wstepnyCzteropodporowyPrawa32[1][2] = doRuchuPrawa[1][2];
  wstepnyCzteropodporowyPrawa32[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa32[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa32[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L2 P0
  wstepnyCzteropodporowyLewa33[0][0] = doRuchuLewa[0][0] + suniecieX;
  wstepnyCzteropodporowyLewa33[0][1] = doRuchuLewa[0][1] - suniecieY;
  wstepnyCzteropodporowyLewa33[0][2] = doRuchuLewa[0][2];
  wstepnyCzteropodporowyLewa33[1][0] = doRuchuLewa[1][0];
  wstepnyCzteropodporowyLewa33[1][1] = doRuchuLewa[1][1];
  wstepnyCzteropodporowyLewa33[1][2] = doRuchuLewa[1][2];
  wstepnyCzteropodporowyLewa33[2][0] = doRuchuLewa[2][0] - suniecieX;
  wstepnyCzteropodporowyLewa33[2][1] = doRuchuLewa[2][1] + suniecieY;
  wstepnyCzteropodporowyLewa33[2][2] = doRuchuLewa[1][2];

  wstepnyCzteropodporowyPrawa33[0][0] = doRuchuPrawa[0][0] - suniecieX;
  wstepnyCzteropodporowyPrawa33[0][1] = doRuchuPrawa[0][1] + suniecieY;
  wstepnyCzteropodporowyPrawa33[0][2] = doRuchuPrawa[0][2];
  wstepnyCzteropodporowyPrawa33[1][0] = doRuchuPrawa[1][0];
  wstepnyCzteropodporowyPrawa33[1][1] = doRuchuPrawa[1][1];
  wstepnyCzteropodporowyPrawa33[1][2] = doRuchuPrawa[1][2];
  wstepnyCzteropodporowyPrawa33[2][0] = doRuchuPrawa[2][0] + suniecieX;
  wstepnyCzteropodporowyPrawa33[2][1] = doRuchuPrawa[2][1] - suniecieY;
  wstepnyCzteropodporowyPrawa33[2][2] = doRuchuPrawa[2][2];
}
void ruchCzteropodporowy(){
  float suniecieX = sinKatRuchu*suniecie;
  float suniecieY = cosKatRuchu*suniecie;
  Serial.print("SuniecieX: ");
  Serial.println(suniecieX);
  Serial.print("SuniecieY: ");
  Serial.println(suniecieY);

  // Podniesienie L0 P2
  czteropodporowyLewa11[0][0] = polozenieLewa[0][0];
  czteropodporowyLewa11[0][1] = polozenieLewa[0][1];
  czteropodporowyLewa11[0][2] = polozenieLewa[0][2] + ZG;
  czteropodporowyLewa11[1][0] = polozenieLewa[1][0];
  czteropodporowyLewa11[1][1] = polozenieLewa[1][1];
  czteropodporowyLewa11[1][2] = polozenieLewa[1][2];
  czteropodporowyLewa11[2][0] = polozenieLewa[2][0];
  czteropodporowyLewa11[2][1] = polozenieLewa[2][1];
  czteropodporowyLewa11[2][2] = polozenieLewa[2][2];

  czteropodporowyPrawa11[0][0] = polozeniePrawa[0][0];
  czteropodporowyPrawa11[0][1] = polozeniePrawa[0][1];
  czteropodporowyPrawa11[0][2] = polozeniePrawa[0][2];
  czteropodporowyPrawa11[1][0] = polozeniePrawa[1][0];
  czteropodporowyPrawa11[1][1] = polozeniePrawa[1][1];
  czteropodporowyPrawa11[1][2] = polozeniePrawa[1][2];
  czteropodporowyPrawa11[2][0] = polozeniePrawa[2][0];
  czteropodporowyPrawa11[2][1] = polozeniePrawa[2][1];
  czteropodporowyPrawa11[2][2] = polozeniePrawa[2][2] + ZG;

  // Przesunięcie w powietrzu L0 P2
  czteropodporowyLewa12[0][0] = doRuchuLewa[0][0] - suniecieX;
  czteropodporowyLewa12[0][1] = doRuchuLewa[0][1] + suniecieY;
  czteropodporowyLewa12[0][2] = doRuchuLewa[0][2] + ZG;
  czteropodporowyLewa12[1][0] = doRuchuLewa[1][0] + suniecieX;
  czteropodporowyLewa12[1][1] = doRuchuLewa[1][1] - suniecieY;
  czteropodporowyLewa12[1][2] = doRuchuLewa[1][2];
  czteropodporowyLewa12[2][0] = doRuchuLewa[2][0];
  czteropodporowyLewa12[2][1] = doRuchuLewa[2][1];
  czteropodporowyLewa12[2][2] = doRuchuLewa[2][2];

  czteropodporowyPrawa12[0][0] = doRuchuPrawa[0][0];
  czteropodporowyPrawa12[0][1] = doRuchuPrawa[0][1];
  czteropodporowyPrawa12[0][2] = doRuchuPrawa[0][2];
  czteropodporowyPrawa12[1][0] = doRuchuPrawa[1][0] + suniecieX;
  czteropodporowyPrawa12[1][1] = doRuchuPrawa[1][1] - suniecieY;
  czteropodporowyPrawa12[1][2] = doRuchuPrawa[1][2];
  czteropodporowyPrawa12[2][0] = doRuchuPrawa[2][0] - suniecieX;
  czteropodporowyPrawa12[2][1] = doRuchuPrawa[2][1] + suniecieY;
  czteropodporowyPrawa12[2][2] = doRuchuPrawa[2][2] + ZG;

  // Opuszczenie L0 P2
  czteropodporowyLewa13[0][0] = doRuchuLewa[0][0] - suniecieX;
  czteropodporowyLewa13[0][1] = doRuchuLewa[0][1] + suniecieY;
  czteropodporowyLewa13[0][2] = doRuchuLewa[0][2];
  czteropodporowyLewa13[1][0] = doRuchuLewa[1][0] + suniecieX;
  czteropodporowyLewa13[1][1] = doRuchuLewa[1][1] - suniecieY;
  czteropodporowyLewa13[1][2] = doRuchuLewa[1][2];
  czteropodporowyLewa13[2][0] = doRuchuLewa[2][0];
  czteropodporowyLewa13[2][1] = doRuchuLewa[2][1];
  czteropodporowyLewa13[2][2] = doRuchuLewa[2][2];

  czteropodporowyPrawa13[0][0] = doRuchuPrawa[0][0];
  czteropodporowyPrawa13[0][1] = doRuchuPrawa[0][1];
  czteropodporowyPrawa13[0][2] = doRuchuPrawa[0][2];
  czteropodporowyPrawa13[1][0] = doRuchuPrawa[1][0] + suniecieX;
  czteropodporowyPrawa13[1][1] = doRuchuPrawa[1][1] - suniecieY;
  czteropodporowyPrawa13[1][2] = doRuchuPrawa[1][2];
  czteropodporowyPrawa13[2][0] = doRuchuPrawa[2][0] - suniecieX;
  czteropodporowyPrawa13[2][1] = doRuchuPrawa[2][1] + suniecieY;
  czteropodporowyPrawa13[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L1 P1
  czteropodporowyLewa21[0][0] = doRuchuLewa[0][0] - suniecieX;
  czteropodporowyLewa21[0][1] = doRuchuLewa[0][1] + suniecieY;
  czteropodporowyLewa21[0][2] = doRuchuLewa[0][2];
  czteropodporowyLewa21[1][0] = doRuchuLewa[1][0] + suniecieX;
  czteropodporowyLewa21[1][1] = doRuchuLewa[1][1] - suniecieY;
  czteropodporowyLewa21[1][2] = doRuchuLewa[1][2] + ZG;
  czteropodporowyLewa21[2][0] = doRuchuLewa[2][0];
  czteropodporowyLewa21[2][1] = doRuchuLewa[2][1];
  czteropodporowyLewa21[2][2] = doRuchuLewa[2][2];

  czteropodporowyPrawa21[0][0] = doRuchuPrawa[0][0];
  czteropodporowyPrawa21[0][1] = doRuchuPrawa[0][1];
  czteropodporowyPrawa21[0][2] = doRuchuPrawa[0][2];
  czteropodporowyPrawa21[1][0] = doRuchuPrawa[1][0] + suniecieX;
  czteropodporowyPrawa21[1][1] = doRuchuPrawa[1][1] - suniecieY;
  czteropodporowyPrawa21[1][2] = doRuchuPrawa[1][2] + ZG;
  czteropodporowyPrawa21[2][0] = doRuchuPrawa[2][0] - suniecieX;
  czteropodporowyPrawa21[2][1] = doRuchuPrawa[2][1] + suniecieY;
  czteropodporowyPrawa21[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu L1 P1
  czteropodporowyLewa22[0][0] = doRuchuLewa[0][0];
  czteropodporowyLewa22[0][1] = doRuchuLewa[0][1];
  czteropodporowyLewa22[0][2] = doRuchuLewa[0][2];
  czteropodporowyLewa22[1][0] = doRuchuLewa[1][0] - suniecieX;
  czteropodporowyLewa22[1][1] = doRuchuLewa[1][1] + suniecieY;
  czteropodporowyLewa22[1][2] = doRuchuLewa[1][2] + ZG;
  czteropodporowyLewa22[2][0] = doRuchuLewa[2][0] + suniecieX;
  czteropodporowyLewa22[2][1] = doRuchuLewa[2][1] - suniecieY;
  czteropodporowyLewa22[2][2] = doRuchuLewa[2][2];

  czteropodporowyPrawa22[0][0] = doRuchuPrawa[0][0] + suniecieX;
  czteropodporowyPrawa22[0][1] = doRuchuPrawa[0][1] - suniecieY;
  czteropodporowyPrawa22[0][2] = doRuchuPrawa[0][2];
  czteropodporowyPrawa22[1][0] = doRuchuPrawa[1][0] - suniecieX;
  czteropodporowyPrawa22[1][1] = doRuchuPrawa[1][1] + suniecieY;
  czteropodporowyPrawa22[1][2] = doRuchuPrawa[1][2] + ZG;
  czteropodporowyPrawa22[2][0] = doRuchuPrawa[2][0];
  czteropodporowyPrawa22[2][1] = doRuchuPrawa[2][1];
  czteropodporowyPrawa22[2][2] = doRuchuPrawa[2][2];

  // Opuszczenie L1 P1
  czteropodporowyLewa23[0][0] = doRuchuLewa[0][0];
  czteropodporowyLewa23[0][1] = doRuchuLewa[0][1];
  czteropodporowyLewa23[0][2] = doRuchuLewa[0][2];
  czteropodporowyLewa23[1][0] = doRuchuLewa[1][0] - suniecieX;
  czteropodporowyLewa23[1][1] = doRuchuLewa[1][1] + suniecieY;
  czteropodporowyLewa23[1][2] = doRuchuLewa[1][2];
  czteropodporowyLewa23[2][0] = doRuchuLewa[2][0] + suniecieX;
  czteropodporowyLewa23[2][1] = doRuchuLewa[2][1] - suniecieY;
  czteropodporowyLewa23[2][2] = doRuchuLewa[2][2];

  czteropodporowyPrawa23[0][0] = doRuchuPrawa[0][0] + suniecieX;
  czteropodporowyPrawa23[0][1] = doRuchuPrawa[0][1] - suniecieY;
  czteropodporowyPrawa23[0][2] = doRuchuPrawa[0][2];
  czteropodporowyPrawa23[1][0] = doRuchuPrawa[1][0] - suniecieX;
  czteropodporowyPrawa23[1][1] = doRuchuPrawa[1][1] + suniecieY;
  czteropodporowyPrawa23[1][2] = doRuchuPrawa[1][2];
  czteropodporowyPrawa23[2][0] = doRuchuPrawa[2][0];
  czteropodporowyPrawa23[2][1] = doRuchuPrawa[2][1];
  czteropodporowyPrawa23[2][2] = doRuchuPrawa[2][2];

  // Podniesienie L2 P0
  czteropodporowyLewa31[0][0] = doRuchuLewa[0][0];
  czteropodporowyLewa31[0][1] = doRuchuLewa[0][1];
  czteropodporowyLewa31[0][2] = doRuchuLewa[0][2];
  czteropodporowyLewa31[1][0] = doRuchuLewa[1][0] - suniecieX;
  czteropodporowyLewa31[1][1] = doRuchuLewa[1][1] + suniecieY;
  czteropodporowyLewa31[1][2] = doRuchuLewa[1][2];
  czteropodporowyLewa31[2][0] = doRuchuLewa[2][0] + suniecieX;
  czteropodporowyLewa31[2][1] = doRuchuLewa[2][1] - suniecieY;
  czteropodporowyLewa31[2][2] = doRuchuLewa[2][2] + ZG;

  czteropodporowyPrawa31[0][0] = doRuchuPrawa[0][0] + suniecieX;
  czteropodporowyPrawa31[0][1] = doRuchuPrawa[0][1] - suniecieY;
  czteropodporowyPrawa31[0][2] = doRuchuPrawa[0][2] + ZG;
  czteropodporowyPrawa31[1][0] = doRuchuPrawa[1][0] - suniecieX;
  czteropodporowyPrawa31[1][1] = doRuchuPrawa[1][1] + suniecieY;
  czteropodporowyPrawa31[1][2] = doRuchuPrawa[1][2];
  czteropodporowyPrawa31[2][0] = doRuchuPrawa[2][0];
  czteropodporowyPrawa31[2][1] = doRuchuPrawa[2][1];
  czteropodporowyPrawa31[2][2] = doRuchuPrawa[2][2];

  // Przesuniecie w powietrzu L2 P0
  czteropodporowyLewa32[0][0] = doRuchuLewa[0][0] + suniecieX;
  czteropodporowyLewa32[0][1] = doRuchuLewa[0][1] - suniecieY;
  czteropodporowyLewa32[0][2] = doRuchuLewa[0][2];
  czteropodporowyLewa32[1][0] = doRuchuLewa[1][0];
  czteropodporowyLewa32[1][1] = doRuchuLewa[1][1];
  czteropodporowyLewa32[1][2] = doRuchuLewa[1][2];
  czteropodporowyLewa32[2][0] = doRuchuLewa[2][0] - suniecieX;
  czteropodporowyLewa32[2][1] = doRuchuLewa[2][1] + suniecieY;
  czteropodporowyLewa32[2][2] = doRuchuLewa[2][2] + ZG;

  czteropodporowyPrawa32[0][0] = doRuchuPrawa[0][0] - suniecieX;
  czteropodporowyPrawa32[0][1] = doRuchuPrawa[0][1] + suniecieY;
  czteropodporowyPrawa32[0][2] = doRuchuPrawa[0][2] + ZG;
  czteropodporowyPrawa32[1][0] = doRuchuPrawa[1][0];
  czteropodporowyPrawa32[1][1] = doRuchuPrawa[1][1];
  czteropodporowyPrawa32[1][2] = doRuchuPrawa[1][2];
  czteropodporowyPrawa32[2][0] = doRuchuPrawa[2][0] + suniecieX;
  czteropodporowyPrawa32[2][1] = doRuchuPrawa[2][1] - suniecieY;
  czteropodporowyPrawa32[2][2] = doRuchuPrawa[2][2];

  //Opuszczenie L2 P0
  czteropodporowyLewa33[0][0] = doRuchuLewa[0][0] + suniecieX;
  czteropodporowyLewa33[0][1] = doRuchuLewa[0][1] - suniecieY;
  czteropodporowyLewa33[0][2] = doRuchuLewa[0][2];
  czteropodporowyLewa33[1][0] = doRuchuLewa[1][0];
  czteropodporowyLewa33[1][1] = doRuchuLewa[1][1];
  czteropodporowyLewa33[1][2] = doRuchuLewa[1][2];
  czteropodporowyLewa33[2][0] = doRuchuLewa[2][0] - suniecieX;
  czteropodporowyLewa33[2][1] = doRuchuLewa[2][1] + suniecieY;
  czteropodporowyLewa33[2][2] = doRuchuLewa[2][2];

  czteropodporowyPrawa33[0][0] = doRuchuPrawa[0][0] - suniecieX;
  czteropodporowyPrawa33[0][1] = doRuchuPrawa[0][1] + suniecieY;
  czteropodporowyPrawa33[0][2] = doRuchuPrawa[0][2];
  czteropodporowyPrawa33[1][0] = doRuchuPrawa[1][0];
  czteropodporowyPrawa33[1][1] = doRuchuPrawa[1][1];
  czteropodporowyPrawa33[1][2] = doRuchuPrawa[1][2];
  czteropodporowyPrawa33[2][0] = doRuchuPrawa[2][0] + suniecieX;
  czteropodporowyPrawa33[2][1] = doRuchuPrawa[2][1] - suniecieY;
  czteropodporowyPrawa33[2][2] = doRuchuPrawa[2][2];

}

float zmianaLewa11[3][3];
float zmianaLewa12[3][3];
float zmianaLewa13[3][3];
float zmianaLewa21[3][3];
float zmianaLewa22[3][3];
float zmianaLewa23[3][3];

float zmianaPrawa11[3][3];
float zmianaPrawa12[3][3];
float zmianaPrawa13[3][3];
float zmianaPrawa21[3][3];
float zmianaPrawa22[3][3];
float zmianaPrawa23[3][3];


void zmianaRuchu(){
  // Podniesienie L0 P1 L2
  zmianaLewa11[0][0] = polozenieLewa[0][0];
  zmianaLewa11[0][1] = polozenieLewa[0][1];
  zmianaLewa11[0][2] = polozenieLewa[0][2] + ZG;
  zmianaLewa11[1][0] = polozenieLewa[1][0];
  zmianaLewa11[1][1] = polozenieLewa[1][1];
  zmianaLewa11[1][2] = polozenieLewa[1][2];
  zmianaLewa11[2][0] = polozenieLewa[2][0];
  zmianaLewa11[2][1] = polozenieLewa[2][1];
  zmianaLewa11[2][2] = polozenieLewa[2][2] + ZG;

  zmianaPrawa11[0][0] = polozeniePrawa[0][0];
  zmianaPrawa11[0][1] = polozeniePrawa[0][1];
  zmianaPrawa11[0][2] = polozeniePrawa[0][2];
  zmianaPrawa11[1][0] = polozeniePrawa[1][0];
  zmianaPrawa11[1][1] = polozeniePrawa[1][1];
  zmianaPrawa11[1][2] = polozeniePrawa[1][2] + ZG;
  zmianaPrawa11[2][0] = polozeniePrawa[2][0];
  zmianaPrawa11[2][1] = polozeniePrawa[2][1];
  zmianaPrawa11[2][2] = polozeniePrawa[2][2];

  // Przesuniecie w powietrzu L0 P1 L2
  zmianaLewa12[0][0] = doRuchuLewa[0][0];
  zmianaLewa12[0][1] = doRuchuLewa[0][1];
  zmianaLewa12[0][2] = doRuchuLewa[0][2] + ZG;
  zmianaLewa12[1][0] = polozenieLewa[1][0];
  zmianaLewa12[1][1] = polozenieLewa[1][1];
  zmianaLewa12[1][2] = polozenieLewa[1][2];
  zmianaLewa12[2][0] = doRuchuLewa[2][0];
  zmianaLewa12[2][1] = doRuchuLewa[2][1];
  zmianaLewa12[2][2] = doRuchuLewa[2][2] + ZG;

  zmianaPrawa12[0][0] = polozeniePrawa[0][0];
  zmianaPrawa12[0][1] = polozeniePrawa[0][1];
  zmianaPrawa12[0][2] = polozeniePrawa[0][2];
  zmianaPrawa12[1][0] = doRuchuPrawa[1][0];
  zmianaPrawa12[1][1] = doRuchuPrawa[1][1];
  zmianaPrawa12[1][2] = doRuchuPrawa[1][2] + ZG;
  zmianaPrawa12[2][0] = polozeniePrawa[2][0];
  zmianaPrawa12[2][1] = polozeniePrawa[2][1];
  zmianaPrawa12[2][2] = polozeniePrawa[2][2];

  // Opuszczenie L0 P1 L2
  zmianaLewa13[0][0] = doRuchuLewa[0][0];
  zmianaLewa13[0][1] = doRuchuLewa[0][1];
  zmianaLewa13[0][2] = doRuchuLewa[0][2];
  zmianaLewa13[1][0] = polozenieLewa[1][0];
  zmianaLewa13[1][1] = polozenieLewa[1][1];
  zmianaLewa13[1][2] = polozenieLewa[1][2];
  zmianaLewa13[2][0] = doRuchuLewa[2][0];
  zmianaLewa13[2][1] = doRuchuLewa[2][1];
  zmianaLewa13[2][2] = doRuchuLewa[2][2];

  zmianaPrawa13[0][0] = polozeniePrawa[0][0];
  zmianaPrawa13[0][1] = polozeniePrawa[0][1];
  zmianaPrawa13[0][2] = polozeniePrawa[0][2];
  zmianaPrawa13[1][0] = doRuchuPrawa[1][0];
  zmianaPrawa13[1][1] = doRuchuPrawa[1][1];
  zmianaPrawa13[1][2] = doRuchuPrawa[1][2];
  zmianaPrawa13[2][0] = polozeniePrawa[2][0];
  zmianaPrawa13[2][1] = polozeniePrawa[2][1];
  zmianaPrawa13[2][2] = polozeniePrawa[2][2];

  // Podniesienie P0 L1 P2
  zmianaLewa21[0][0] = doRuchuLewa[0][0];
  zmianaLewa21[0][1] = doRuchuLewa[0][1];
  zmianaLewa21[0][2] = doRuchuLewa[0][2];
  zmianaLewa21[1][0] = polozenieLewa[1][0];
  zmianaLewa21[1][1] = polozenieLewa[1][1];
  zmianaLewa21[1][2] = polozenieLewa[1][2] + ZG;
  zmianaLewa21[2][0] = doRuchuLewa[2][0];
  zmianaLewa21[2][1] = doRuchuLewa[2][1];
  zmianaLewa21[2][2] = doRuchuLewa[2][2];

  zmianaPrawa21[0][0] = polozeniePrawa[0][0];
  zmianaPrawa21[0][1] = polozeniePrawa[0][1];
  zmianaPrawa21[0][2] = polozeniePrawa[0][2] + ZG;
  zmianaPrawa21[1][0] = doRuchuPrawa[1][0];
  zmianaPrawa21[1][1] = doRuchuPrawa[1][1];
  zmianaPrawa21[1][2] = doRuchuPrawa[1][2];
  zmianaPrawa21[2][0] = polozeniePrawa[2][0];
  zmianaPrawa21[2][1] = polozeniePrawa[2][1];
  zmianaPrawa21[2][2] = polozeniePrawa[2][2] + ZG;

  // Przesuniecie w powietrzu P0 L1 P2
  zmianaLewa22[0][0] = doRuchuLewa[0][0];
  zmianaLewa22[0][1] = doRuchuLewa[0][1];
  zmianaLewa22[0][2] = doRuchuLewa[0][2];
  zmianaLewa22[1][0] = doRuchuLewa[1][0];
  zmianaLewa22[1][1] = doRuchuLewa[1][1];
  zmianaLewa22[1][2] = doRuchuLewa[1][2] + ZG;
  zmianaLewa22[2][0] = doRuchuLewa[2][0];
  zmianaLewa22[2][1] = doRuchuLewa[2][1];
  zmianaLewa22[2][2] = doRuchuLewa[2][2];

  zmianaPrawa22[0][0] = doRuchuPrawa[0][0];
  zmianaPrawa22[0][1] = doRuchuPrawa[0][1];
  zmianaPrawa22[0][2] = doRuchuPrawa[0][2] + ZG;
  zmianaPrawa22[1][0] = doRuchuPrawa[1][0];
  zmianaPrawa22[1][1] = doRuchuPrawa[1][1];
  zmianaPrawa22[1][2] = doRuchuPrawa[1][2];
  zmianaPrawa22[2][0] = doRuchuPrawa[2][0];
  zmianaPrawa22[2][1] = doRuchuPrawa[2][1];
  zmianaPrawa22[2][2] = doRuchuPrawa[2][2] + ZG;

  // Opuszczenie P0 L1 P2
  zmianaLewa23[0][0] = doRuchuLewa[0][0];
  zmianaLewa23[0][1] = doRuchuLewa[0][1];
  zmianaLewa23[0][2] = doRuchuLewa[0][2];
  zmianaLewa23[1][0] = doRuchuLewa[1][0];
  zmianaLewa23[1][1] = doRuchuLewa[1][1];
  zmianaLewa23[1][2] = doRuchuLewa[1][2];
  zmianaLewa23[2][0] = doRuchuLewa[2][0];
  zmianaLewa23[2][1] = doRuchuLewa[2][1];
  zmianaLewa23[2][2] = doRuchuLewa[2][2];

  zmianaPrawa23[0][0] = doRuchuPrawa[0][0];
  zmianaPrawa23[0][1] = doRuchuPrawa[0][1];
  zmianaPrawa23[0][2] = doRuchuPrawa[0][2];
  zmianaPrawa23[1][0] = doRuchuPrawa[1][0];
  zmianaPrawa23[1][1] = doRuchuPrawa[1][1];
  zmianaPrawa23[1][2] = doRuchuPrawa[1][2];
  zmianaPrawa23[2][0] = doRuchuPrawa[2][0];
  zmianaPrawa23[2][1] = doRuchuPrawa[2][1];
  zmianaPrawa23[2][2] = doRuchuPrawa[2][2];
}

float obrotLewa11[3][3];
float obrotLewa12[3][3];
float obrotLewa13[3][3];
float obrotPrawa11[3][3];
float obrotPrawa12[3][3];
float obrotPrawa13[3][3];

float obrotLewa21[3][3];
float obrotLewa22[3][3];
float obrotLewa23[3][3];
float obrotPrawa21[3][3];
float obrotPrawa22[3][3];
float obrotPrawa23[3][3];

// Funkcja realizująca obrót robota o zadany kąt
void obrotKat(float katObrotu){
  float tetaL0 = 125.74;
  float tetaL1 = 180.0;
  float tetaL2 = 234.25;
  float tetaP0 = 54.25;
  float tetaP1 = 0.0;
  float tetaP2 = -54.25;
  float rMale = sqrt(73.41*73.41+124.6*124.6) + X;
  float rDuze = 143.24 + X;

  // Podniesienie L0 P1 L2
  obrotLewa11[0][0] = polozenieLewa[0][0];
  obrotLewa11[0][1] = polozenieLewa[0][1];
  obrotLewa11[0][2] = polozenieLewa[0][2] + ZG;
  obrotLewa11[1][0] = polozenieLewa[1][0];
  obrotLewa11[1][1] = polozenieLewa[1][1];
  obrotLewa11[1][2] = polozenieLewa[1][2];
  obrotLewa11[2][0] = polozenieLewa[2][0];
  obrotLewa11[2][1] = polozenieLewa[2][1];
  obrotLewa11[2][2] = polozenieLewa[2][2] + ZG;

  obrotPrawa11[0][0] = polozeniePrawa[0][0];
  obrotPrawa11[0][1] = polozeniePrawa[0][1];
  obrotPrawa11[0][2] = polozeniePrawa[0][2];
  obrotPrawa11[1][0] = polozeniePrawa[1][0];
  obrotPrawa11[1][1] = polozeniePrawa[1][1];
  obrotPrawa11[1][2] = polozeniePrawa[1][2] + ZG;
  obrotPrawa11[2][0] = polozeniePrawa[2][0];
  obrotPrawa11[2][1] = polozeniePrawa[2][1];
  obrotPrawa11[2][2] = polozeniePrawa[2][2];

  // Przesuniecie w powietrzu L0 P1 L2
  obrotLewa12[0][0] = rMale*cosinus(tetaL0 + katObrotu);
  obrotLewa12[0][1] = rMale*sinus(tetaL0 + katObrotu);
  obrotLewa12[0][2] = polozenieLewa[0][2] + ZG;
  obrotLewa12[1][0] = polozenieLewa[1][0];
  obrotLewa12[1][1] = polozenieLewa[1][1];
  obrotLewa12[1][2] = polozenieLewa[1][2];
  obrotLewa12[2][0] = rMale*cosinus(tetaL2 + katObrotu);
  obrotLewa12[2][1] = rMale*sinus(tetaL2 + katObrotu);
  obrotLewa12[2][2] = polozenieLewa[2][2] + ZG;

  obrotPrawa12[0][0] = polozeniePrawa[0][0];
  obrotPrawa12[0][1] = polozeniePrawa[0][1];
  obrotPrawa12[0][2] = polozeniePrawa[0][2];
  obrotPrawa12[1][0] = rDuze*cosinus(tetaP1 + katObrotu);
  obrotPrawa12[1][1] = rDuze*sinus(tetaP1 + katObrotu);
  obrotPrawa12[1][2] = polozeniePrawa[1][2] + ZG;
  obrotPrawa12[2][0] = polozeniePrawa[2][0];
  obrotPrawa12[2][1] = polozeniePrawa[2][1];
  obrotPrawa12[2][2] = polozeniePrawa[2][2];

  // Opuszczenie L0 P1 L2
  obrotLewa13[0][0] = rMale*cosinus(tetaL0 + katObrotu);
  obrotLewa13[0][1] = rMale*sinus(tetaL0 + katObrotu);
  obrotLewa13[0][2] = polozenieLewa[0][2];
  obrotLewa13[1][0] = polozenieLewa[1][0];
  obrotLewa13[1][1] = polozenieLewa[1][1];
  obrotLewa13[1][2] = polozenieLewa[1][2];
  obrotLewa13[2][0] = rMale*cosinus(tetaL2 + katObrotu);
  obrotLewa13[2][1] = rMale*sinus(tetaL2 + katObrotu);
  obrotLewa13[2][2] = polozenieLewa[2][2];

  obrotPrawa13[0][0] = polozeniePrawa[0][0];
  obrotPrawa13[0][1] = polozeniePrawa[0][1];
  obrotPrawa13[0][2] = polozeniePrawa[0][2];
  obrotPrawa13[1][0] = rDuze*cosinus(tetaP1 + katObrotu);
  obrotPrawa13[1][1] = rDuze*sinus(tetaP1 + katObrotu);
  obrotPrawa13[1][2] = polozeniePrawa[1][2];
  obrotPrawa13[2][0] = polozeniePrawa[2][0];
  obrotPrawa13[2][1] = polozeniePrawa[2][1];
  obrotPrawa13[2][2] = polozeniePrawa[2][2];

  // Podniesienie P0 L1 P2
  obrotLewa21[0][0] = polozenieLewa[0][0];
  obrotLewa21[0][1] = polozenieLewa[0][1];
  obrotLewa21[0][2] = polozenieLewa[0][2];
  obrotLewa21[1][0] = polozenieLewa[1][0];
  obrotLewa21[1][1] = polozenieLewa[1][1];
  obrotLewa21[1][2] = polozenieLewa[1][2] + ZG;
  obrotLewa21[2][0] = polozenieLewa[2][0];
  obrotLewa21[2][1] = polozenieLewa[2][1];
  obrotLewa21[2][2] = polozenieLewa[2][2];

  obrotPrawa21[0][0] = polozeniePrawa[0][0];
  obrotPrawa21[0][1] = polozeniePrawa[0][1];
  obrotPrawa21[0][2] = polozeniePrawa[0][2] + ZG;
  obrotPrawa21[1][0] = polozeniePrawa[1][0];
  obrotPrawa21[1][1] = polozeniePrawa[1][1];
  obrotPrawa21[1][2] = polozeniePrawa[1][2];
  obrotPrawa21[2][0] = polozeniePrawa[2][0];
  obrotPrawa21[2][1] = polozeniePrawa[2][1];
  obrotPrawa21[2][2] = polozeniePrawa[2][2] + ZG;

  // Przesuniecie w powietrzu P0 L1 P2
  obrotLewa22[0][0] = doRuchuLewa[0][0];
  obrotLewa22[0][1] = doRuchuLewa[0][1];
  obrotLewa22[0][2] = doRuchuLewa[0][2];
  obrotLewa22[1][0] = doRuchuLewa[1][0];
  obrotLewa22[1][1] = doRuchuLewa[1][1];
  obrotLewa22[1][2] = doRuchuLewa[1][2] + ZG;
  obrotLewa22[2][0] = doRuchuLewa[2][0];
  obrotLewa22[2][1] = doRuchuLewa[2][1];
  obrotLewa22[2][2] = doRuchuLewa[2][2];

  obrotPrawa22[0][0] = doRuchuPrawa[0][0];
  obrotPrawa22[0][1] = doRuchuPrawa[0][1];
  obrotPrawa22[0][2] = doRuchuPrawa[0][2] + ZG;
  obrotPrawa22[1][0] = doRuchuPrawa[1][0];
  obrotPrawa22[1][1] = doRuchuPrawa[1][1];
  obrotPrawa22[1][2] = doRuchuPrawa[1][2];
  obrotPrawa22[2][0] = doRuchuPrawa[2][0];
  obrotPrawa22[2][1] = doRuchuPrawa[2][1];
  obrotPrawa22[2][2] = doRuchuPrawa[2][2] + ZG;

  // Opuszczenie P0 L1 P2
  obrotLewa23[0][0] = doRuchuLewa[0][0];
  obrotLewa23[0][1] = doRuchuLewa[0][1];
  obrotLewa23[0][2] = doRuchuLewa[0][2];
  obrotLewa23[1][0] = doRuchuLewa[1][0];
  obrotLewa23[1][1] = doRuchuLewa[1][1];
  obrotLewa23[1][2] = doRuchuLewa[1][2];
  obrotLewa23[2][0] = doRuchuLewa[2][0];
  obrotLewa23[2][1] = doRuchuLewa[2][1];
  obrotLewa23[2][2] = doRuchuLewa[2][2];

  obrotPrawa23[0][0] = doRuchuPrawa[0][0];
  obrotPrawa23[0][1] = doRuchuPrawa[0][1];
  obrotPrawa23[0][2] = doRuchuPrawa[0][2];
  obrotPrawa23[1][0] = doRuchuPrawa[1][0];
  obrotPrawa23[1][1] = doRuchuPrawa[1][1];
  obrotPrawa23[1][2] = doRuchuPrawa[1][2];
  obrotPrawa23[2][0] = doRuchuPrawa[2][0];
  obrotPrawa23[2][1] = doRuchuPrawa[2][1];
  obrotPrawa23[2][2] = doRuchuPrawa[2][2];
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Wire.begin(21,22);
  pwm2.begin();
  pwm2.setPWMFreq(50);
  pwm1.begin();
  pwm1.setPWMFreq(50);

  ustawLewaNoga(0,homeLewaNoga[0][0],homeLewaNoga[0][1],homeLewaNoga[0][2]);
  ustawLewaNoga(1,homeLewaNoga[1][0],homeLewaNoga[1][1],homeLewaNoga[1][2]);
  ustawLewaNoga(2,homeLewaNoga[2][0],homeLewaNoga[2][1],homeLewaNoga[2][2]);

  ustawPrawaNoga(0,homePrawaNoga[0][0],homePrawaNoga[0][1],homePrawaNoga[0][2]);
  ustawPrawaNoga(1,homePrawaNoga[1][0],homePrawaNoga[1][1],homePrawaNoga[1][2]);
  ustawPrawaNoga(2,homePrawaNoga[2][0],homePrawaNoga[2][1],homePrawaNoga[2][2]);
  //delay(1000);
  Serial.println("Rozpoczynam szukanie.");
  
  dlugoscKroku(X,Y,Z);
  ruchLiniowy(doRuchuLewa,doRuchuPrawa,10);
  //wyznaczKatRuchu(0);
  //wstepnyRuchPojedynczejNogi();
  //ruchPojedynczejNogi();
  //ruchTrojpodporowy();
  
}

// Ta część odpowiada ze testy do doboru czasu opóźnienia wymaganego do ruchu serwomechanizmów
float testPredkosciL1[3][3];
float testPredkosciL2[3][3];
float testPredkosciP1[3][3];
float testPredkosciP2[3][3];

void testPredkosci(){
  for (int i = 0; i < 3; ++i) {
    Wynik wynik = lokalnyDoBazowego(260, 0, 90, 'L', i);
    testPredkosciL1[i][0] = wynik.x;
    testPredkosciL1[i][1] = wynik.y;
    testPredkosciL1[i][2] = wynik.z;
  }

  for (int i = 0; i < 3; ++i) {
    Wynik wynik = lokalnyDoBazowego(160, 0, 20, 'L', i);
    testPredkosciL2[i][0] = wynik.x;
    testPredkosciL2[i][1] = wynik.y;
    testPredkosciL2[i][2] = wynik.z;
  }

  for (int i = 0; i < 3; ++i) {
    Wynik wynik = lokalnyDoBazowego(260, 0, 90, 'P', i);
    testPredkosciP1[i][0] = wynik.x;
    testPredkosciP1[i][1] = wynik.y;
    testPredkosciP1[i][2] = wynik.z;
  }

  for (int i = 0; i < 3; ++i) {
    Wynik wynik = lokalnyDoBazowego(160, 0, 20, 'P', i);
    testPredkosciP2[i][0] = wynik.x;
    testPredkosciP2[i][1] = wynik.y;
    testPredkosciP2[i][2] = wynik.z;
  }
}

int ustawienie = 0;

float nowyKatRuchu = -1;
float staryKatRuchu = -1;
int ruchStary = -1;

void loop() {
  int ruchLoop = ruch;
  float katRuchuLoop = katRuchu;
  int obrotLoop = obrot;
  if ((ruchStary != ruchLoop) && (ruchStary != -1)){
    zmianaRuchu();
    ruchLiniowy(zmianaLewa11,zmianaPrawa11,2);
    ruchLiniowy(zmianaLewa12,zmianaPrawa12,4);
    ruchLiniowy(zmianaLewa13,zmianaPrawa13,2);
    ruchLiniowy(zmianaLewa21,zmianaPrawa21,2);
    ruchLiniowy(zmianaLewa22,zmianaPrawa22,4);
    ruchLiniowy(zmianaLewa23,zmianaPrawa23,2);
    ustawienie = 0;
  }
  if ((staryKatRuchu == -1) || (staryKatRuchu + 5 > nowyKatRuchu) || (staryKatRuchu - 5 < nowyKatRuchu)){
    nowyKatRuchu = katRuchuLoop;
  }
  if ((ruchLoop == 0) && (katRuchuLoop >= 0)){
    if ((katRuchuLoop != staryKatRuchu)){
      wyznaczKatRuchu(katRuchuLoop);
      ruchTrojpodporowy();
    }
    ruchLiniowy(trojpodporowyLewa11,trojpodporowyPrawa11,2);
    ruchLiniowy(trojpodporowyLewa12,trojpodporowyPrawa12,4);
    ruchLiniowy(trojpodporowyLewa13,trojpodporowyPrawa13,2);
    ruchLiniowy(trojpodporowyLewa21,trojpodporowyPrawa21,2);
    ruchLiniowy(trojpodporowyLewa22,trojpodporowyPrawa22,4);
    ruchLiniowy(trojpodporowyLewa23,trojpodporowyPrawa23,2);
  }
  if ((ruchLoop == 1) && (katRuchuLoop >= 0)){
    if ((katRuchuLoop != staryKatRuchu)){
      wyznaczKatRuchu(katRuchuLoop);
      wstepnyRuchPojedynczejNogi();
      ruchPojedynczejNogi();
      ustawienie = 0;
    }
    if (ustawienie == 0){
      ruchLiniowy(bazowanieLewaNoga11,bazowaniePrawaNoga11,2);
      ruchLiniowy(bazowanieLewaNoga12,bazowaniePrawaNoga12,4);
      ruchLiniowy(bazowanieLewaNoga13,bazowaniePrawaNoga13,2);
      ruchLiniowy(bazowanieLewaNoga21,bazowaniePrawaNoga21,2);
      ruchLiniowy(bazowanieLewaNoga22,bazowaniePrawaNoga22,4);
      ruchLiniowy(bazowanieLewaNoga23,bazowaniePrawaNoga23,2);
      ruchLiniowy(bazowanieLewaNoga31,bazowaniePrawaNoga31,2);
      ruchLiniowy(bazowanieLewaNoga32,bazowaniePrawaNoga32,4);
      ruchLiniowy(bazowanieLewaNoga33,bazowaniePrawaNoga33,2);
      ruchLiniowy(bazowanieLewaNoga41,bazowaniePrawaNoga41,2);
      ruchLiniowy(bazowanieLewaNoga42,bazowaniePrawaNoga42,4);
      ruchLiniowy(bazowanieLewaNoga43,bazowaniePrawaNoga43,2);
      ruchLiniowy(bazowanieLewaNoga51,bazowaniePrawaNoga51,2);
      ruchLiniowy(bazowanieLewaNoga52,bazowaniePrawaNoga52,4);
      ruchLiniowy(bazowanieLewaNoga53,bazowaniePrawaNoga53,2);
      ruchLiniowy(bazowanieLewaNoga61,bazowaniePrawaNoga61,2);
      ruchLiniowy(bazowanieLewaNoga62,bazowaniePrawaNoga62,4);
      ruchLiniowy(bazowanieLewaNoga63,bazowaniePrawaNoga63,2);
      ustawienie = 1;
    }
    ruchLiniowy(ruchLewaNoga11,ruchPrawaNoga11,2);
    ruchLiniowy(ruchLewaNoga12,ruchPrawaNoga12,4);
    ruchLiniowy(ruchLewaNoga13,ruchPrawaNoga13,2);
    ruchLiniowy(ruchLewaNoga21,ruchPrawaNoga21,2);
    ruchLiniowy(ruchLewaNoga22,ruchPrawaNoga22,4);
    ruchLiniowy(ruchLewaNoga23,ruchPrawaNoga23,2);
    ruchLiniowy(ruchLewaNoga31,ruchPrawaNoga31,2);
    ruchLiniowy(ruchLewaNoga32,ruchPrawaNoga32,4);
    ruchLiniowy(ruchLewaNoga33,ruchPrawaNoga33,2);
    ruchLiniowy(ruchLewaNoga41,ruchPrawaNoga41,2);
    ruchLiniowy(ruchLewaNoga42,ruchPrawaNoga42,4);
    ruchLiniowy(ruchLewaNoga43,ruchPrawaNoga43,2);
    ruchLiniowy(ruchLewaNoga51,ruchPrawaNoga51,2);
    ruchLiniowy(ruchLewaNoga52,ruchPrawaNoga52,4);
    ruchLiniowy(ruchLewaNoga53,ruchPrawaNoga53,2);
    ruchLiniowy(ruchLewaNoga61,ruchPrawaNoga61,2);
    ruchLiniowy(ruchLewaNoga62,ruchPrawaNoga62,4);
    ruchLiniowy(ruchLewaNoga63,ruchPrawaNoga63,2);
  }
  if ((ruchLoop == 2) && (katRuchuLoop >= 0)){
    if ((katRuchuLoop != staryKatRuchu)){
      wyznaczKatRuchu(katRuchuLoop);
      wstepnyCzteropodporowy();
      ruchCzteropodporowy();
      ustawienie = 0;
    }
    if (ustawienie == 0){
      ruchLiniowy(wstepnyCzteropodporowyLewa11,wstepnyCzteropodporowyPrawa11,2);
      ruchLiniowy(wstepnyCzteropodporowyLewa12,wstepnyCzteropodporowyPrawa12,4);
      ruchLiniowy(wstepnyCzteropodporowyLewa13,wstepnyCzteropodporowyPrawa13,2);
      ruchLiniowy(wstepnyCzteropodporowyLewa21,wstepnyCzteropodporowyPrawa21,2);
      ruchLiniowy(wstepnyCzteropodporowyLewa22,wstepnyCzteropodporowyPrawa22,4);
      ruchLiniowy(wstepnyCzteropodporowyLewa23,wstepnyCzteropodporowyPrawa23,2);
      ruchLiniowy(wstepnyCzteropodporowyLewa31,wstepnyCzteropodporowyPrawa31,2);
      ruchLiniowy(wstepnyCzteropodporowyLewa32,wstepnyCzteropodporowyPrawa32,4);
      ruchLiniowy(wstepnyCzteropodporowyLewa33,wstepnyCzteropodporowyPrawa33,2);
      ustawienie = 1;
    }
    ruchLiniowy(czteropodporowyLewa11,czteropodporowyPrawa11,2);
    ruchLiniowy(czteropodporowyLewa12,czteropodporowyPrawa12,4);
    ruchLiniowy(czteropodporowyLewa13,czteropodporowyPrawa13,2);
    ruchLiniowy(czteropodporowyLewa21,czteropodporowyPrawa21,2);
    ruchLiniowy(czteropodporowyLewa22,czteropodporowyPrawa22,4);
    ruchLiniowy(czteropodporowyLewa23,czteropodporowyPrawa23,2);
    ruchLiniowy(czteropodporowyLewa31,czteropodporowyPrawa31,2);
    ruchLiniowy(czteropodporowyLewa32,czteropodporowyPrawa32,4);
    ruchLiniowy(czteropodporowyLewa33,czteropodporowyPrawa33,2);
  }
  staryKatRuchu = katRuchuLoop;
  if((obrotLoop == 1) || (obrotLoop == -1)){
    if(obrotLoop == 1){
      obrotKat(-10);
    }
    if(obrotLoop == -1){
      obrotKat(10);
    }
    ruchLiniowy(obrotLewa11,obrotPrawa11,2);
    ruchLiniowy(obrotLewa12,obrotPrawa12,4);
    ruchLiniowy(obrotLewa13,obrotPrawa13,2);
    ruchLiniowy(obrotLewa21,obrotPrawa21,2);
    ruchLiniowy(obrotLewa22,obrotPrawa22,4);
    ruchLiniowy(obrotLewa23,obrotPrawa23,2);
  }
}
