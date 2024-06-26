#include <cstdint>
//Ultrasuoni Destra
#define triggerPin1 19
#define echoPin1 18

//Ultrasuoni Sinistra
#define triggerPin2 32
#define echoPin2 35

//Ultrasuoni Avanti
#define triggerPin3 23
#define echoPin3 22

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                //motori A (sinistra)
#define enA 12
#define in1 14
#define in2 27

//motori B (destra)
#define in3 26
#define in4 25
#define enB 33

//Righe e Colonne
#define RIGHE 30 //righe
#define COLONNE 30 //colonne

struct cella {

  uint16_t valore;
  //valenza cella

  uint8_t stato : 2;
  /*
      0 - inesplorato
      1 - esplorato
      2 - esplorato + di una volta
  */
}
struct coordinate {
  uint8_t x;
  uint8_t y;
}
cella matrix[RIGHE][COLONNE];
uint8_t direzione;
void stampaMatrice(cella matrix[RIGHE][COLONNE], const coordinate &posizione) {
  for (uint8_t i = 0; i < RIGHE; i++) {
    for (uint8_t j = 0; j < COLONNE; j++) {
      if (i == posizione.y && j == posizione.x) {
        Serial.print("X");
      } else {
        Serial.print(matrix[RIGHE][COLONNE].valore);
        Serial.print(" ");
      }
    }
    Serial.println("");
  }

}
void setup() {
  Serial.begin(115200);
  direzione = 1; //direzione default : Nord
  coordinate posizione = {0, 0}; //posizione iniziale
  /*         1
             |
           2- -3
             |
             4
  */
  for (uint8_t i = 0; i < RIGHE; i++) {
    for (uint8_t j = 0; i < COLONNE; j++) {
      matrix[i][j].valore = 901;
      matrix[i][j].stato = 0;
    }
  }
  //Ultrasuoni Destra
  pinMode(triggerPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  //Ultrasuoni Sinistra
  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  //Ultrasuoni Avanti
  pinMode(triggerPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

  //motori A (sinistra)
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  //motori B (destra)
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);



}

void loop() {
  float EchoDestra = misuraDistanza(triggerPin1, echoPin1);
  float EchoSinistra = misuraDistanza(triggerPin2, echoPin2);
  float EchoAvanti = misuraDistanza(triggerPin3, echoPin3);

  Serial.print("Distanza sensore 1: ");
  Serial.println(EchoDestra);

  Serial.print("Distanza sensore 2: ");
  Serial.println(EchoSinistra);

  Serial.print("Distanza sensore 3: ");
  Serial.println(EchoAvanti);

  avanti();
  //va avanti di 30 cm e si ferma
  switch (direzione) {
    case 1 : //Su
      posizione.y--;
      break;
    case 2 : //Sinistra
      posizione.x--;
      break;
    case 3 : //Destra
      posizione.x++;
      break;
    case 4 : //Giù
      posizione.y++;
      break;
    default :
      Serial.println("Direzione non valida");
      return 1;
  }
  Serial.println("Posizione attuale : ")
  Serial.println("x = ")
  Serial.print(posizione.x);
  Serial.println("y = ")
  Serial.print(posizione.y);

  if (EchoAvanti < 20 && EchoDestra < 20 && EchoSinistra > 20) {
    sinistra();
    delay(1000);
  }
  else if (EchoAvanti < 20 && EchoSinistra < 20 && EchoDestra > 20) {
    destra();
    delay(1000);
  }
  else if (EchoAvanti < 20 && EchoDestra < 20 && EchoSinistra < 20) {
    destra();
    delay(1000);
    spento();
    delay(500);
    destra();
    delay(1000);
  }
  else if (EchoAvanti > 20 && EchoDestra > 20 && EchoSinistra > 20) {
    avanti();
    delay(1000);
  }
  else if (EchoAvanti < 20 && EchoDestra > 20 && EchoSinistra > 20) {
    destra();
    delay(1000);
  }

}



float misuraDistanza(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  float duration = pulseIn(echoPin, HIGH);
  float distance = duration * 0.034 / 2;

  return distance;
}
void spento(){
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  }
void avanti() {
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, 1); //motore a avanti
  digitalWrite(in2, 0);
  digitalWrite(in3, 1); //motore b avanti
  digitalWrite(in4, 0);
}



void destra() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, 1); //motore a avanti
  digitalWrite(in2, 0);
  digitalWrite(in3, 0); //motore b avanti
  digitalWrite(in4, 1);
  switch (direzione) {
    case 1 : //nel caso fosse verso nord
      direzione = 3; //est-destra
      break;
    case 2 : //nel caso fosse verso ovest
      direzione = 1; //nord-su
      break;
    case 3 : //nel caso fosse verso est
      direzione = 4; //sud-giù
      break;
    case 4 : //nel caso fosse verso sud
      direzione = 2; //ovest-sinistra
      break;
  }
}




void sinistra() {
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, 0); //motore a avanti
  digitalWrite(in2, 1);
  digitalWrite(in3, 1); //motore b avanti
  digitalWrite(in4, 0);
  switch (direzione) {
    case 1 : //nel caso fosse verso nord
      direzione = 2; //ovest-sinistra
      break;
    case 2 : //nel caso fosse verso ovest
      direzione = 4; //sud-giù
      break;
    case 3 : //nel caso fosse verso est
      direzione = 1; //nord-su
      break;
    case 4 : //nel caso fosse verso sud
      direzione = 3; //est-destra
  }
}
