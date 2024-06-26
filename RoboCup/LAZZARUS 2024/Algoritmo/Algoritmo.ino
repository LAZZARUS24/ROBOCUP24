

#define triggerPin1 19
#define echoPin1 18
#define triggerPin2 32
#define echoPin2 35
#define triggerPin3 23
#define echoPin3 22
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                //motori a
#define enA 12
#define in1 14
#define in2 27

//motori b
#define in3 26
#define in4 25
#define enB 33
void setup() {
 Serial.begin(115200);
 pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(triggerPin1, OUTPUT);
  pinMode(echoPin1, INPUT);

  pinMode(triggerPin2, OUTPUT);
  pinMode(echoPin2, INPUT);

  pinMode(triggerPin3, OUTPUT);
  pinMode(echoPin3, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
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

  if(EchoAvanti < 20 && EchoDestra<20 && EchoSinistra>20){delay(500);sinistra();delay(1000);}
  else if(EchoAvanti < 20 && EchoSinistra<20 && EchoDestra > 20){delay(500);destra();delay(1000);}
  else if(EchoAvanti < 20 && EchoDestra<20 && EchoSinistra<20){delay(500);destra();delay(1000);destra();delay(1000);}
  else if(EchoAvanti > 20 && EchoDestra>20 && EchoSinistra>20){delay(500);avanti();delay(1000);}
  else if(EchoAvanti < 20 && EchoDestra>20 && EchoSinistra>20){delay(500);destra();delay(1000);}
  // Aggiungi qui la logica di risposta in base alle distanze misurate
}\


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

void avanti(){
  analogWrite(enA, 200);
  analogWrite(enB, 200);
  digitalWrite(in1, 1); //motore a avanti
  digitalWrite(in2, 0); 
  digitalWrite(in3, 1); //motore b avanti
  digitalWrite(in4, 0);
}



void destra(){
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, 1); //motore a avanti
  digitalWrite(in2, 0); 
  digitalWrite(in3, 0); //motore b avanti
  digitalWrite(in4, 1);
  }



  
void sinistra(){
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, 0); //motore a avanti
  digitalWrite(in2, 1); 
  digitalWrite(in3, 1); //motore b avanti
  digitalWrite(in4, 0);
  }



  void indietro(){
  analogWrite(enA, 255);
  analogWrite(enB, 255);
  digitalWrite(in1, 0); //motore a avanti
  digitalWrite(in2, 1); 
  digitalWrite(in3, 0); //motore b avanti
  digitalWrite(in4, 1);
}
