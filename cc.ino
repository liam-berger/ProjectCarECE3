#include "ECE3.h"

uint16_t sensorValues[8]; // right -> left, 0 -> 7
uint16_t mins[8] = {669,734,618,757,614,711,734,805};
uint16_t maxs[8] = {1652,1635,1039,1565,853,1515,1754,1695};

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11;
const int right_dir_pin=30;
const int right_pwm_pin=39;

const int led=41;

float get_error(uint16_t sv[]) {

  for(int i = 0; i<8; i++) {
    Serial.print(sv[i]); Serial.print(" ");
    sv[i] = (sv[i] - mins[i]) * (1000.0 / maxs[i]);
  }
  Serial.println();

  uint16_t comb = (sv[4]-sv[3] + 2*(sv[5]-sv[2]) + 4*(sv[6]-sv[1]) + 8*(sv[7]-sv[0]))/4;
  Serial.println(comb);
  
  return (comb - 24.56) / 48.69;
}


void setup() {
  // put your setup code here, to run once:
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);

  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);

  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  

  ECE3_Init();

  digitalWrite(led,HIGH);

  Serial.begin(9600); 
  delay(2000); //Wait 2 seconds before starting 
}

void loop() {
  // put your main code here, to run repeatedly: 
  ECE3_read_IR(sensorValues);
  Serial.println(get_error(sensorValues));

  delay(500);

  //int leftSpd = 70;
  //analogWrite(left_pwm_pin,leftSpd);

}
