#include <ESP32Servo.h>
#include "Adafruit_TCS34725.h"
#include <MPU6050_light.h>
#include "Wire.h"
MPU6050 mpu(Wire);
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
volatile boolean state = false;
float sogliaAngolo = 0.2;
uint8_t va, vb;
int debug=0; 
int angolo = 0;
int ultimoValoreCheckpoint = 0;
bool blu = 0;

// pin servo medkit
#define pinKit 25
#define pinCam 19

Servo servoKit;
Servo servoCam;
// Righe e Colonne
#define RIGHE 15 // righe
#define COLONNE 15 // colonne
///NORD****************
#define POSIZIONESU_NORD           matrix[posizione.y-1][posizione.x]
#define POSIZIONESINISTRA_NORD     matrix[posizione.y][posizione.x-1]
#define POSIZIONEDESTRA_NORD       matrix[posizione.y][posizione.x+1]
#define POSIZIONEGIU_NORD          matrix[posizione.y+1][posizione.x]
///OVEST*****************
#define POSIZIONESU_OVEST           matrix[posizione.y][posizione.x-1]
#define POSIZIONESINISTRA_OVEST     matrix[posizione.y+1][posizione.x]
#define POSIZIONEDESTRA_OVEST       matrix[posizione.y-1][posizione.x]
#define POSIZIONEGIU_OVEST          matrix[posizione.y][posizione.x+1]
///EST*****************
#define POSIZIONESU_EST           matrix[posizione.y][posizione.x+1]
#define POSIZIONESINISTRA_EST     matrix[posizione.y-1][posizione.x]
#define POSIZIONEDESTRA_EST       matrix[posizione.y+1][posizione.x]
#define POSIZIONEGIU_EST          matrix[posizione.y][posizione.x-1]
///SUD*****************
#define POSIZIONESU_SUD           matrix[posizione.y+1][posizione.x]
#define POSIZIONESINISTRA_SUD     matrix[posizione.y][posizione.x+1]
#define POSIZIONEDESTRA_SUD       matrix[posizione.y][posizione.x-1]
#define POSIZIONEGIU_SUD          matrix[posizione.y-1][posizione.x]
///MURO*****************
#define POSIZIONESU_LIBERO        (muroNord=0&&matrix[posizione.y-1][posizione.x].stato==0)
#define POSIZIONESINISTRA_LIBERO        (muroNord=0&&matrix[posizione.y][posizione.x-1].stato==0)
#define POSIZIONEDESTRA_LIBERO        (muroNord=0&&matrix[posizione.y][posizione.x+1].stato==0)
#define POSIZIONEGIU_LIBERO        (muroNord=0&&matrix[posizione.y+1][posizione.x].stato==0)
float EchoDestra = 0.0;
float EchoSinistra = 0.0;
float EchoAvanti = 0.0; 
bool muroNord=0;
bool muroOvest=0;
bool muroEst=0;
int i = -1;
uint8_t contatore = 0;
uint8_t direzione = 1;
bool mod = 0;
uint8_t prevClk;
uint8_t currClk;
volatile int tic, cit=0;
int temp =0;
#define encoder 23
#define pulsante_checkpoint 35
#define triggerPin1 19  //destra
#define echoPin1 18

#define triggerPin2 33  //sinistra
#define echoPin2 32

#define triggerPin3 12  //avanti
#define echoPin3 14
//motori a
#define enA 5
#define in1 17
#define in2 16

//motori b
#define in3 4
#define in4 0
#define enB 2
byte status;
uint16_t r, g, b, c;
struct cella {
    uint8_t valore; // valenza cella
    uint8_t stato;
      /*
        0 - inesplorato
        1 - esplorato
        2 - esplorato + di una volta
        3 - muro
    */
  
};

struct coordinate {
    int x;
    int y; 
};

cella matrix[RIGHE][COLONNE];

  coordinate posizione = {7, 7}; // posizione iniziale
  coordinate precedente = {0, 0}; // posizione precedente

void isr() 
{
  state = true;
}
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}
void stampaMatrice(cella matrix[RIGHE][COLONNE], coordinate posizione, coordinate precedente) {
    for (uint8_t i = 0; i < RIGHE; i++) {
        for (uint8_t j = 0; j < COLONNE; j++) {
            if(matrix[i][j].stato==3){Serial.print("- ");
            }
            else if (i == posizione.y && j == posizione.x) {
                Serial.print("X ");
                if (matrix[i][j].stato == 0||matrix[i][j].stato == 4) {
                    if (matrix[i][j].stato == 0)matrix[i][j].stato = 1;
                    contatore = matrix[precedente.y][precedente.x].valore;
                    if(mod==0)contatore++; else contatore--;
                    matrix[i][j].valore = contatore;
                }else if(matrix[i][j].stato == 1){
                matrix[i][j].stato = 2;
                }
                else if (matrix[i][j].stato == 2) {
                Serial.println("Posizione gia' raggiunta");
                return ;
                }
            } else {
                if(matrix[i][j].valore>=10){Serial.print(int(matrix[i][j].valore) - matrix[i][j].valore/10*10);Serial.print(" ");
                }
                else {Serial.print(int(matrix[i][j].valore));Serial.print(" ");
                }
            }
        }
        Serial.println();
    }
}

void kit(uint8_t a){
 for(uint8_t i = 0;i<a;i++){
 servoKit.write(70);
 delay(2000);
 servoKit.write(110);
 delay(1000);
 servoKit.write(70);
 }
}

void setup() {
   Serial.begin(9600);
   Wire.begin(21,22); 
   attachInterrupt(digitalPinToInterrupt(encoder), isr, CHANGE);  
   servoKit.setPeriodHertz(50);
   servoKit.attach(pinKit, 500, 2500);
   servoCam.setPeriodHertz(50);
   servoCam.attach(pinCam, 500, 2500);                                    
   status = mpu.begin();
   Serial.print(F("MPU6050 status: "));
   Serial.println(status);
   while (status != 0) { }
   Serial.println(F("Calculating offsets, do not move MPU6050"));
   delay(100);
   mpu.calcOffsets(); // gyro and accelero
   Serial.println("Done!\n");
   Serial.begin(9600);
   if (tcs.begin()) {
     Serial.println("TCS34725 pronto");
   } else {
    Serial.println("Errore nel trovare il sensore TCS34725");
     while (1);
  }
         for (uint8_t i = 0; i < RIGHE; i++) {
        for (uint8_t j = 0; j < COLONNE; j++) {
            matrix[i][j].valore = 0;
            matrix[i][j].stato = 0;
        }
    }
      tcs.write8(TCS34725_PERS, TCS34725_PERS_NONE); 
  tcs.setInterrupt(true);
  
  Serial.flush();
 pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(2, OUTPUT);
  //Echo destra
  pinMode(triggerPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  
  //Echo sinistra
  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  
  //Echo avanti
  pinMode(triggerPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(encoder, INPUT);     //encoder 23
  pinMode(pulsante_checkpoint, INPUT_PULLUP);
  // Esegui prima lettura dei valori sui pin
  prevClk = digitalRead(encoder);
  tic = 0;
  analogWrite(enA, 200);
  analogWrite(enB, 200);
 // attachInterrupt(pulsante_checkpoint , checkpoint() , FALLING );
}

void loop() {
//fermo :

  spento();
  Serial.println("Stato posizione : ");
  Serial.print(matrix[posizione.y][posizione.x].stato);
  Serial.println("modalita' : ");
  Serial.print(mod);
 //while(debug!=1)debug=Serial.parseInt();
  debug=0;
     tcs.getRawData(&r, &g, &b, &c);
   Serial.print("Colore: R:"); Serial.print(r);
   Serial.print(" G:"); Serial.print(g);
   Serial.print(" B:"); Serial.print(b);
   Serial.print(" C:"); Serial.print(c);

   if(blu==1){
     delay(4000);
     blu=0;
     }
  EchoDestra = misuraDistanza(triggerPin1, echoPin1);
  delay(10);
  EchoSinistra = misuraDistanza(triggerPin2, echoPin2);
  delay(10);
  EchoAvanti = misuraDistanza(triggerPin3, echoPin3);
  delay(10);
  Serial.println(posizione.x);
  Serial.println(posizione.y);  
  if(EchoSinistra<20)muroOvest=1; else muroOvest=0;
  if(EchoDestra<20)muroEst=1; else muroEst=0;
  if(EchoAvanti<20)muroNord=1; else {muroNord=0;}
  stampaMatrice(matrix, posizione, precedente);
  i++;  
  Serial.println("");
  Serial.print("Muro Nord : ");
  Serial.print(muroNord);
  Serial.print(" Muro Ovest : ");
  Serial.print(muroOvest);
  Serial.print("; Muro Est : ");
  Serial.print(muroEst);
  precedente = posizione;
  if(muroNord){
if(EchoAvanti<8){
  digitalWrite(in1, 0); //motore a indietro
  digitalWrite(in2, 1); 
  digitalWrite(in3, 0); //motore b indietro
  digitalWrite(in4, 1);
  Serial.println("indietro");
  while(EchoAvanti<8){
    EchoAvanti = misuraDistanza(triggerPin3, echoPin3);
      Serial.println(EchoAvanti);
    }
    spento();
  }
  else if(EchoAvanti>8){
  digitalWrite(in1, 1); //motore a avanti
  digitalWrite(in2, 0); 
  digitalWrite(in3, 1); //motore b avanti
  digitalWrite(in4, 0);
    Serial.println("avanti");
  while(EchoAvanti>8){
          Serial.println(EchoAvanti);
    EchoAvanti = misuraDistanza(triggerPin3, echoPin3);
    }
    spento();
  }
       // gioele dell'utri
      

    }
    spento();
    for(uint8_t i = 1;i<=3;i++){
      if(i==1){
        if(muroOvest){
          servoCam.write(180);
          delay(1000);
          }
          continue;
        }

      if(i==2){
        if(muroNord){
          servoCam.write(90);
          delay(1000);
          }
          continue;
        
        }

        if(i==3){
        if(muroEst){
          servoCam.write(0);
          delay(1000);
          }
            continue;
        
        }
        
      }
   if(mod==0){

      if((muroEst==0&&muroOvest==0)||(muroNord==0 && muroEst==0&&muroOvest==1) || (muroNord==0 && muroEst!=0&&muroOvest==0)){
                   matrix[posizione.y][posizione.x].stato=4;digitalWrite(2, HIGH);
         }else{digitalWrite(2, LOW);}
             

      switch(direzione){
            case 1: // Su
                if((muroNord==1 || POSIZIONESU_NORD.stato>=3) && matrix[posizione.y][posizione.x].stato==4){mod=0;
                destra();//direzione=3;
                }
                else if(muroNord==0&&POSIZIONESU_NORD.stato==0){direzione=1;}    //cella.stato su == 0
                else if((POSIZIONESU_NORD.stato>=3||muroNord==1) && matrix[posizione.y][posizione.x].stato!=4){             //cella.stato su == 3
                if((POSIZIONEDESTRA_NORD.stato==0&&muroEst==0) && (POSIZIONESINISTRA_NORD.stato>=3||muroOvest==1)){
                  destra();//direzione=3;
                  }       //cella.stato destra == 0
                else if((POSIZIONESINISTRA_NORD.stato==0&&muroOvest==0) && (POSIZIONEDESTRA_NORD.stato>=3||muroEst==1)) //cella.stato sinistra == 0
                sinistra();//direzione=2;
                else if((POSIZIONESINISTRA_NORD.stato>=3||muroOvest==1) && (POSIZIONEDESTRA_NORD.stato>=3||muroEst==1)){
                mod=1;
                puntaverso(4);//direzione=4;
                }
                }
                break;

                
            case 2: // Sinistra
                if((POSIZIONESU_OVEST.stato>=3||muroNord==1) && matrix[posizione.y][posizione.x].stato==4){mod=0;
                destra();//direzione=1;
                }
                else if(POSIZIONESU_OVEST.stato==0&&muroNord==0)direzione=2;     //resta cosi' com'e'
                else if((POSIZIONESU_OVEST.stato>=3||muroNord==1) && matrix[posizione.y][posizione.x].stato!=4){             //cella.stato su == 3
                if(POSIZIONEDESTRA_OVEST.stato==0&&muroEst==0)//cella.stato destra == 0
                destra();  //  direzione=1;    
                else if(POSIZIONESINISTRA_OVEST.stato==0&&muroOvest==0)  //cella.stato sinistra == 0
                sinistra(); //direzione=4;
                else if((POSIZIONESU_OVEST.stato>=3 || muroNord==1) && (POSIZIONESINISTRA_OVEST.stato>=3||muroOvest==1) && (POSIZIONEDESTRA_EST.stato>=3||muroEst==1)){
                mod=1;
                puntaverso(3);//direzione=3;
                }
                }
                break;
 
            case 3: // Destra
                if((POSIZIONESU_EST.stato>=3||muroNord==1) && matrix[posizione.y][posizione.x].stato==4){mod=0;
                destra();//direzione=4;
                }
                else if(POSIZIONESU_EST.stato==0&&muroNord==0)direzione=3;     //resta cosi' com'e'
                else if((POSIZIONESU_EST.stato>=3||muroNord==1) && matrix[posizione.y][posizione.x].stato!=4){             //cella.stato su == 3
                if(POSIZIONEDESTRA_EST.stato==0&&muroEst==0){//cella.stato destra == 0
                destra();   direzione=4;} //    
                else if(POSIZIONESINISTRA_EST.stato==0&&muroOvest==0){  //cella.stato sinistra == 0
                sinistra();direzione=1;} //
                else if((POSIZIONESU_EST.stato>=3 || muroNord==1) && (POSIZIONESINISTRA_EST.stato>=3||muroOvest==1) && (POSIZIONEDESTRA_EST.stato>=3||muroEst==1)){
                mod=1;
                puntaverso(2);direzione=2;//
                }
                }
                break;
   
            case 4: // Giù
                if((POSIZIONESU_SUD.stato>=3||muroNord==1) && matrix[posizione.y][posizione.x].stato==4){mod=0;
                destra();direzione=2;//
                }
                else if(POSIZIONESU_SUD.stato==0&&muroNord==0){direzione=4;}     //resta cosi' com'e'
                else if((POSIZIONESU_SUD.stato>=3||muroNord==1) && matrix[posizione.y][posizione.x].stato!=4){             //cella.stato su == 3
                if(POSIZIONEDESTRA_SUD.stato==0&&muroEst==0){//cella.stato destra == 0
                destra();   direzione=2;} //    
                else if(POSIZIONESINISTRA_SUD.stato==0&&muroOvest==0){  //cella.stato sinistra == 0
                sinistra();direzione=3;} //
                else if((POSIZIONESU_SUD.stato>=3 || muroNord==1) && (POSIZIONESINISTRA_SUD.stato>=3||muroOvest==1) && (POSIZIONEDESTRA_SUD.stato>=3||muroEst==1)){
                mod=1;
                puntaverso(1);direzione=1;//
                }
                }
                break;


}

    }else if (mod==1){

      if(matrix[posizione.y][posizione.x].stato!=4){
      if(POSIZIONESU_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
      {
        puntaverso(1);
    }
      else if(POSIZIONEGIU_NORD.valore==matrix[posizione.y][posizione.x].valore-1)
        {
        puntaverso(4);
      }
      else if(POSIZIONESINISTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
   {
       puntaverso(2);
   }
    else if(POSIZIONEDESTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
        {
        puntaverso(3);
        }
else {
    Serial.println("FINITO!");
    return ;
}
    }else{
      switch(direzione){
        case 1:
         if(POSIZIONESU_NORD.stato==0 && muroNord==0){puntaverso(1);mod=0;}
         else if(POSIZIONESINISTRA_NORD.stato==0 && muroOvest==0){puntaverso(2);mod=0;}
         else if(POSIZIONEDESTRA_NORD.stato==0 && muroEst==0){puntaverso(3);mod=0;}
         else{
          if(POSIZIONESU_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(1);
          else if(POSIZIONEGIU_NORD.valore==matrix[posizione.y][posizione.x].valore-1)
             puntaverso(4);
          else if(POSIZIONESINISTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(2);
          else if(POSIZIONEDESTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(3);
         }    
         break;

         case 2:
         if(POSIZIONESU_OVEST.stato==0 && muroNord==0){puntaverso(2);mod=0;}
         else if(POSIZIONESINISTRA_OVEST.stato==0 && muroOvest==0){puntaverso(4);mod=0;}
         else if(POSIZIONEDESTRA_OVEST.stato==0 && muroEst==0){puntaverso(1);mod=0;}
         else{
          if(POSIZIONESU_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(1);
          else if(POSIZIONEGIU_NORD.valore==matrix[posizione.y][posizione.x].valore-1)
             puntaverso(4);
          else if(POSIZIONESINISTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(2);
          else if(POSIZIONEDESTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(3);
         }    
         break;

         case 3:
         if(POSIZIONESU_EST.stato==0 && muroNord==0){puntaverso(3);mod=0;}
         else if(POSIZIONESINISTRA_EST.stato==0 && muroOvest==0){puntaverso(1);mod=0;}
         else if(POSIZIONEDESTRA_EST.stato==0 && muroEst==0){puntaverso(4);mod=0;}
         else{
          if(POSIZIONESU_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(1);
          else if(POSIZIONEGIU_NORD.valore==matrix[posizione.y][posizione.x].valore-1)
             puntaverso(4);
          else if(POSIZIONESINISTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(2);
          else if(POSIZIONEDESTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(3);
         }    
         break;

         case 4:

         if(POSIZIONESINISTRA_SUD.stato==0 && muroOvest==0){sinistra();mod=0;}
         else if(POSIZIONEDESTRA_SUD.stato==0 && muroEst==0){puntaverso(2);mod=0;}
         else{
          if(POSIZIONESU_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(1);
          else if(POSIZIONEGIU_NORD.valore==matrix[posizione.y][posizione.x].valore-1)
             puntaverso(4);
          else if(POSIZIONESINISTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(2);
          else if(POSIZIONEDESTRA_NORD.valore == matrix[posizione.y][posizione.x].valore-1)
             puntaverso(3);
         }    
         break;
        }

    }
    }
//delay(1000);
    avanti();
spento();   

  Serial.println();
  Serial.println("");
  Serial.println("");
  Serial.print("Stato posizione : ");
  Serial.print(matrix[posizione.y][posizione.x].stato);
}





void spento() {
  digitalWrite(in1, 0);
  digitalWrite(in2, 0); 
  digitalWrite(in3, 0); 
  digitalWrite(in4, 0);
  }
//***********************************

void puntaverso(uint8_t x){
  if( (direzione==1 && x==4) || (direzione==4 & x==1) || (direzione==2 && x==3) || (direzione==3 && x==2) ){
    vicolo();
    }
  else if( (direzione==1 && x==3) || (direzione==3 & x==4) || (direzione==4 && x==2) || (direzione==2 && x==1) ){
    destra();
  }
  else if( (direzione==1 && x==2) || (direzione==2 & x==4) || (direzione==4 && x==3) || (direzione==3 && x==1) ){
    sinistra();
    }

  }
  
//***********************************

  void destra(){
  spento();
  status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println("destra");
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(200);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  switch(direzione){
    case 1 :
    direzione=3;
    break;
    case 2 :
    direzione=1;
    break;
    case 3 :
    direzione=4;
    break;
    case 4 :
    direzione=2;
    break;
    }
    temp = mpu.getAngleZ();
     analogWrite(enA, 250);
  analogWrite(enB, 250);
  
  digitalWrite(in1, 1); 
  digitalWrite(in2, 0);
   
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
do{
  mpu.update();
  delay(10);
  angolo = mpu.getAngleZ();
   Serial.println(mpu.getAngleZ());
  }while (angolo >temp-90);
  spento();
  status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  angolo=0;
  } 
  
//***********************************
void vicolo(){
  spento();
    Serial.println("vicolo");
  status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(200);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  switch(direzione){
    case 1 :
    direzione=4;
    break;
    case 2 :
    direzione=3;
    break;
    case 3 :
    direzione=2;
    break;
    case 4 :
    direzione=1;
    break;
    }

     temp = mpu.getAngleZ();
do{
  mpu.update();
  delay(10);
  angolo = mpu.getAngleZ();
   Serial.println(mpu.getAngleZ());
  analogWrite(enA, 250);
  analogWrite(enB, 250);
  
  digitalWrite(in1, 1); 
  digitalWrite(in2, 0);
   
  digitalWrite(in3, 0);
  digitalWrite(in4, 1);
  }while (angolo >temp-180);
  spento();
  status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  angolo=0;
  }
  //*********************************** 

void avanti(){
  analogWrite(enA, 170);
  analogWrite(enB, 170);
  
  digitalWrite(in1, 1); //motore a avanti
  digitalWrite(in2, 0); 
  digitalWrite(in3, 1); //motore b avanti
  digitalWrite(in4, 0);
tic=0;
       while(tic<43){
       // mpu.update();  
        //regolaVelocita(mpu.getAngleZ());

        
       currClk = digitalRead(encoder);
     if (currClk != prevClk) {
       tic ++;
  if (state) {
    uint16_t r, g, b, c, colorTemp, lux;
    getRawData_noDelay(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);
identifica_colore(r,g,b);
    Serial.flush();

    tcs.clearInterrupt();
    state = false;
  }

       cit=tic;
       Serial.println("TIC : ");Serial.print(tic);
      }
     }
cit=0;
  //spento();
              switch (direzione) {
            case 1: // Su
                posizione.y--;
                break;
            case 4: // Giu
             posizione.y++;
                
                break;
            case 2: // Sinistra
            posizione.x--;
        
                break;
            case 3: // Destra
                     posizione.x++;
                break;
            default:

                contatore--;
        }
}

//***********************************

  
    
//***********************************

  void sinistra(){
    spento();
      Serial.println("sinistra");
  status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
    switch(direzione){
    case 1 :
    direzione=2;
    break;
    case 2 :
    direzione=4;
    break;
    case 3 :
    direzione=1;
    break;
    case 4 :
    direzione=3;
    break;
    }
     temp = mpu.getAngleZ();
  do{
  mpu.update();
  delay(10);
  angolo = mpu.getAngleZ();
   Serial.println(mpu.getAngleZ());
  analogWrite(enA, 230);
  analogWrite(enB, 230);
  
  digitalWrite(in1, 0); 
  digitalWrite(in2, 1);
   
  digitalWrite(in3, 1);
  digitalWrite(in4, 0);
  }while (angolo <temp+90);
  spento();
  status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
  angolo=0;
  }
  
//***********************************
   void identifica_colore(uint16_t r, uint16_t g, uint16_t b) {
     if (r <= 20 && g <= 20 && b <= 20) {
       mod=1;
         matrix[posizione.y][posizione.x].stato=3;
        indietro();
                      switch (direzione) {
            case 1: // Su
                posizione.y++;
                break;
            case 4: // Giu
             posizione.y--;
                
                break;
            case 2: // Sinistra
            posizione.x++;
        
                break;
            case 3: // Destra
                     posizione.x--;
                break;
            default:
               contatore++;
     
        }
        tic=0;
    while(tic<(cit-4)){
     currClk = digitalRead(23);
   if (currClk != prevClk) {
     tic ++;
     Serial.println("TIC : ");Serial.print(tic);
     prevClk = currClk;
     }
   }
   spento();
         Serial.println("Nero");
     }else if (r < g && r < b && abs(g - b) <= 5) {
     blu = true;
     } else {
         Serial.println( "Altro colore");
     }
 }

  void indietro(){
  tic=0; 
  analogWrite(enA, 170);
  analogWrite(enB, 170);
  digitalWrite(in1, 0); //motore a indietro
  digitalWrite(in2, 1); 
  digitalWrite(in3, 0); //motore b indietro
  digitalWrite(in4, 1);
  tic=0;
              switch (direzione) {
            case 1: // Su
                posizione.y++;
                break;
            case 4: // Giu
             posizione.y--;
                
                break;
            case 2: // Sinistra
            posizione.x++;
        
                break;
            case 3: // Destra
                     posizione.x--;
                break;
            default:
               contatore++;
     
        }
}

//************************************

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

void checkpoint(){
  while(digitalRead(pulsante_checkpoint)==0);
   for (uint8_t i = 0; i < RIGHE; i++) {
        for (uint8_t j = 0; j < COLONNE; j++) {
          if(matrix[i][j].valore>ultimoValoreCheckpoint){
            matrix[i][j].valore=0;
            matrix[i][j].stato=0;
            }
        }
        }
  }
void regolaVelocita(float angleZ) {
  if (abs(angleZ) > sogliaAngolo) { 
    if(angleZ<(-1*sogliaAngolo)){//sta girando a dx
    va = 155 - abs(angleZ);
    vb = 200 + abs(angleZ);
    } else if(angleZ>sogliaAngolo){//sta girando a sx
    va = 200 + abs(angleZ);
    vb = 155 - abs(angleZ);
    }
  } else {
    // Mantieni la velocità dei motori costante
    va = 150;
    vb = 150;
  }

  // Assicurati che i valori di velocità siano compresi tra 0 e 255
  va = constrain(va, 150, 255);
  vb = constrain(vb, 150, 255);

  // Applica la velocità ai motori
  analogWrite(enA, va);
  analogWrite(enB, vb);
}
