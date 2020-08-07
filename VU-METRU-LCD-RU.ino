#define GAIN 14 // усиление 0...50
#define STEP 2  // плавность полос 0...20
#define RL 0    // RL - горизонт, вертикаль 0...1
 
#include <LiquidCrystal.h>
#include <EEPROM.h>
  LiquidCrystal lcd(2, 3, 4, 5, 6, 7);// RS,E,D4,D5,D6,D7
  byte z,z0,z1,w,www=1;
  int ur,ul,urr,ull,urrr,ulll;
  int x,i,u_maxr,u_maxl;
  int u_l0[20],u_r0[20];
 
 
void setup() {
  pinMode(8,INPUT_PULLUP);
  byte znak_r[8]={0b00000,0b00000,0b11111,0b00101,0b00101,0b11010,0b00000,0b00000};
  byte znak_l[8]={0b00000,0b00000,0b11111,0b10000,0b10000,0b10000,0b00000,0b00000};
  lcd.createChar(4,znak_r); 
  lcd.createChar(5,znak_l);
  lcd.begin(16, 2);// LCD 16X2
 // analogReference(INTERNAL);  // если очень маленький уровень сигнала
  pinMode(A0,INPUT);// A0 - аналоговый вход R
  pinMode(A1,INPUT);// A1 - аналоговый вход L
  w=EEPROM.read(0);
}
 
void loop() {
  if(digitalRead(8)==LOW){w++;www=1;if(w>4){w=0;}delay(200);EEPROM.update(0,w);}
 
  if(w==0&&www==1){www=0;
  byte a1[8] = {0b10101,0b10101,0b10101,0b10101,0b10101,0b10101,0b10101,0b10101};
  byte a2[8] = {0b10100,0b10100,0b10100,0b10100,0b10100,0b10100,0b10100,0b10100};
  byte a3[8] = {0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000};
  byte a4[8] = {0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100};// 
  lcd.createChar(0,a1);lcd.createChar(1,a2);lcd.createChar(2,a3);lcd.createChar(6,a4);} 
  if(w==1&&www==1){www=0;
  byte a1[8] = {0b00000,0b10101,0b10101,0b10101,0b10101,0b10101,0b10101,0b00000};
  byte a2[8] = {0b00000,0b10100,0b10100,0b10100,0b10100,0b10100,0b10100,0b00000};
  byte a3[8] = {0b00000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b00000};
  byte a4[8] = {0b00000,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00000};//  
  lcd.createChar(0,a1);lcd.createChar(1,a2);lcd.createChar(2,a3);lcd.createChar(6,a4);} 
  if(w==2&&www==1){www=0;
  byte a1[8] = {0b10101,0b10101,0b10101,0b00000,0b00000,0b10101,0b10101,0b10101};
  byte a2[8] = {0b10100,0b10100,0b10100,0b00000,0b00000,0b10100,0b10100,0b10100};
  byte a3[8] = {0b10000,0b10000,0b10000,0b00000,0b00000,0b10000,0b10000,0b10000};
  byte a4[8] = {0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100}; 
  lcd.createChar(0,a1);lcd.createChar(1,a2);lcd.createChar(2,a3);lcd.createChar(6,a4);} 
  if(w==3&&www==1){www=0;
  byte a1[8] = {0b00000,0b10101,0b10101,0b00000,0b00000,0b10101,0b10101,0b00000};
  byte a2[8] = {0b00000,0b10100,0b10100,0b00000,0b00000,0b10100,0b10100,0b00000};
  byte a3[8] = {0b00000,0b10000,0b10000,0b00000,0b00000,0b10000,0b10000,0b00000};
  byte a4[8] = {0b00000,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100,0b00000};
  lcd.createChar(0,a1);lcd.createChar(1,a2);lcd.createChar(2,a3);lcd.createChar(6,a4);} 
  if(w==4&&www==1){www=0;
  byte a1[8] = {0b00000,0b00000,0b10101,0b10101,0b10101,0b10101,0b00000,0b00000};
  byte a2[8] = {0b00000,0b00000,0b10100,0b10100,0b10100,0b10100,0b00000,0b00000};
  byte a3[8] = {0b00000,0b00000,0b10000,0b10000,0b10000,0b10000,0b00000,0b00000};
  byte a4[8] = {0b00000,0b00000,0b00100,0b00100,0b00100,0b00100,0b00000,0b00000};
  lcd.createChar(0,a1);lcd.createChar(1,a2);lcd.createChar(2,a3);lcd.createChar(6,a4);} 
 
  urr = log(analogRead(0))*GAIN;if(urr>41){urr=41;}
  ull = log(analogRead(1))*GAIN;if(ull>41){ull=41;}
 
  if(RL==0){lcd.setCursor(0,0);lcd.write((uint8_t)4);lcd.setCursor(0,1);lcd.write((uint8_t)5);}
  if(RL==1){lcd.setCursor(0,1);lcd.print("L");lcd.setCursor(0,0);lcd.print("R");}
 
   if(urr<ur){ur=ur-1;delay(STEP);}else{ur = urr;}
  for(z=0,z0=0,z1=0;z<=ur;z++,z1++){if(z1>2){z1=0;z0++;}
   if(z1==1){lcd.setCursor(z0+1,1);lcd.write((uint8_t)0);lcd.setCursor(z0+2,1);if(ur<39){lcd.print(" ");}}}
   if(z1==3){lcd.setCursor(z0+1,1);lcd.write((uint8_t)1);}
   if(z1==2){lcd.setCursor(z0+1,1);lcd.write((uint8_t)2);}
 
   if(ull<ul){ul=ul-1;delay(STEP);}else{ul = ull;}
  for(z=0,z0=0,z1=0;z<=ul;z++,z1++){if(z1>2){z1=0;z0++;}
   if(z1==1){lcd.setCursor(z0+1,0);lcd.write((uint8_t)0);lcd.setCursor(z0+2,0);;if(ul<39){lcd.print(" ");}}}
   if(z1==3){lcd.setCursor(z0+1,0);lcd.write((uint8_t)1);}
   if(z1==2){lcd.setCursor(z0+1,0);lcd.write((uint8_t)2);}
 
//////////////////////////////////////////////////////////
 
i++;if(i<19){u_l0[i]=abs(ull);u_r0[i]=abs(urr);}else{i=1;}
  if(i==18){u_maxr=0;u_maxl=0;
    for(x=1;x<=15;x++){
      u_maxl=max(u_maxl,u_l0[x]);
      u_maxr=max(u_maxr,u_r0[x]);}}
 
  if(u_maxl<ulll){ulll=ulll-1;}else{ulll = u_maxl;}
  if(u_maxr<urrr){urrr=urrr-1;}else{urrr = u_maxr;}
 
  if(u_maxl<=ull){u_maxl=ull+1;} 
  if(u_maxr<=urr){u_maxr=urr+1;} 
 
  lcd.setCursor(ulll/3+1,0);if(ulll/3>2){lcd.write((uint8_t)6);}lcd.print("  ");
  lcd.setCursor(urrr/3+1,1);if(urrr/3>2){lcd.write((uint8_t)6);}lcd.print("  "); 
  delay(3);
}
