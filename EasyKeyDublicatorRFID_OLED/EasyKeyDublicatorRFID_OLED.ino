/*
  Скетч к проекту "Копировальщик ключей для домофона RFID с OLED дисплеем и хранением 8 ключей в память EEPROM"
  Аппаратная часть построена на Arduino Nano
  Исходники на GitHub: https://github.com/AlexMalov/EasyKeyDublicatorRFID_OLED/
  Автор: МЕХАТРОН DIY, AlexMalov, 2019
  v 3.2 fix cyfral bug
*/

// Настройки
#include <OneWire.h>
#include <OneWireSlave.h>
#include "pitches.h"
#include <EEPROM.h>
#include <OLED_I2C.h> 
OLED myOLED(SDA, SCL); //создаем экземпляр класса OLED с именем myOLED
extern uint8_t SmallFont[];
extern uint8_t BigNumbers[];
#include "GyverEncoder.h"
#include "TimerOne.h"

//settings
#define rfidUsePWD 0        // ключ использует пароль для изменения
#define rfidPWD 123456      // пароль для ключа
#define rfidBitRate 2       // Скорость обмена с rfid в kbps

//pins
#define iButtonPin A3      // Линия data ibutton
#define iBtnEmulPin A1     // Линия эмулятора ibutton
#define Luse_Led 13        // Светодиод лузы
#define R_Led 2            // RGB Led
#define G_Led 3
#define B_Led 4
#define ACpin 6            // Вход Ain0 аналогового компаратора 0.1В для EM-Marie 
#define speakerPin 12       // Спикер, он же buzzer, он же beeper
#define FreqGen 11         // генератор 125 кГц
#define CLK 8              // s1 энкодера
#define DT 9               // s2 энкодера
#define BtnPin 10          // Кнопка переключения режима чтение/запись

Encoder enc1(CLK, DT, BtnPin);
OneWire ibutton (iButtonPin);
OneWireSlave iBtnEmul(iBtnEmulPin);       //Эмулятор iButton для BlueMode
byte maxKeyCount;                         // максимальное кол-во ключей, которое влазит в EEPROM, но не > 20
byte EEPROM_key_count;                    // количество ключей 0..maxKeyCount, хранящихся в EEPROM
byte EEPROM_key_index = 0;                // 1..EEPROM_key_count номер последнего записанного в EEPROM ключа  
byte addr[8];                             // временный буфер
byte keyID[8];                            // ID ключа для записи
byte rfidData[5];                         // значащие данные frid em-marine
byte halfT;                               // полупериод для метаком
enum emRWType {rwUnknown, TM01, RW1990_1, RW1990_2, TM2004, T5557, EM4305};               // тип болванки
enum emkeyType {keyUnknown, keyDallas, keyTM2004, keyCyfral, keyMetacom, keyEM_Marine};    // тип оригинального ключа  
emkeyType keyType;
enum emMode {md_empty, md_read, md_write, md_blueMode};               // режим раоты копировальщика
emMode copierMode = md_empty;

void OLED_printKey(byte buf[8], byte msgType = 0){
  String st;
  switch (msgType){
    case 0: st = "The key " + String(EEPROM_key_index) + " of " + String(EEPROM_key_count) + " in ROM"; break;      
    case 1: st = "Hold the Btn to save";  break; 
    case 3: st = "The key " + String(indxKeyInROM(buf)) + " exists in ROM";  break;   
  }
  myOLED.clrScr();
  myOLED.print(st, 0, 0);  
  st = "";
  for (byte i = 0; i < 8; i++) st += String(buf[i], HEX) +":";
  myOLED.print(st, 0, 12);
  st = "Type ";
  switch (keyType){
    case keyDallas: st += "Dallas wire"; break;      
    case keyCyfral: st += "Cyfral wire";  break;  
    case keyMetacom: st += "Metakom wire"; break;             
    case keyEM_Marine: st += "EM_Marine rfid"; break;
    case keyUnknown: st += "Unknown"; break;
  }
  myOLED.print(st, 0, 24);
  myOLED.update();
}

void OLED_printError(String st, bool err = true){
  myOLED.clrScr();
  if (err) myOLED.print(F("Error!"), 0, 0);
    else myOLED.print(F("OK"), 0, 0);
  myOLED.print(st, 0, 12);  
  myOLED.update();
}

void setup() {
  pinMode(Luse_Led, OUTPUT); digitalWrite(Luse_Led, HIGH); //поменять на LOW
  myOLED.begin(SSD1306_128X32); //инициализируем дисплей
  pinMode(BtnPin, INPUT_PULLUP);                            // включаем чтение и подягиваем пин кнопки режима к +5В
  pinMode(speakerPin, OUTPUT);
  pinMode(ACpin, INPUT);                                    // Вход аналогового компаратора 3В для Cyfral 
  pinMode(R_Led, OUTPUT); pinMode(G_Led, OUTPUT); pinMode(B_Led, OUTPUT);  //RGB-led
  clearLed();
  pinMode(FreqGen, OUTPUT);                               
  Serial.begin(115200);
  myOLED.clrScr();                                          //Очищаем буфер дисплея.
  myOLED.setFont(SmallFont);                                //Перед выводом текста необходимо выбрать шрифт
  myOLED.print(F("Hello, read a key..."), LEFT, 0);
  char st[16] = {98, 121, 32, 77, 69, 88, 65, 84, 80, 79, 72, 32, 68, 73, 89, 0};
  myOLED.print(st, LEFT, 24);
  myOLED.update();
  Sd_StartOK();
  EEPROM_key_count = EEPROM[0];
  maxKeyCount = EEPROM.length() / 8 - 1; if (maxKeyCount > 20) maxKeyCount = 20;
  if (EEPROM_key_count > maxKeyCount) EEPROM_key_count = 0;
  if (EEPROM_key_count != 0 ) {
    EEPROM_key_index = EEPROM[1];
    Serial.print(F("Read key code from EEPROM: "));
    EEPROM_get_key(EEPROM_key_index, keyID);
    for (byte i = 0; i < 8; i++) {
      Serial.print(keyID[i], HEX); Serial.print(F(":"));  
    }
    Serial.println();
    delay(3000);
    OLED_printKey(keyID);
    copierMode = md_read;
    digitalWrite(G_Led, HIGH);
  } else {
    myOLED.print(F("ROM has no keys yet."), 0, 12);
    myOLED.update();  
  }
  enc1.setTickMode(AUTO);
  enc1.setType(TYPE2);
  enc1.setDirection(REVERSE);         // NORM / REVERSE
  Timer1.initialize(1000);            // установка таймера на каждые 1000 микросекунд (= 1 мс)
  Timer1.attachInterrupt(timerIsr);   // запуск таймера
  digitalWrite(Luse_Led, !digitalRead(Luse_Led)); 
}

void timerIsr() {   // прерывание таймера для энкодера
  enc1.tick();     
}

void clearLed(){
  digitalWrite(R_Led, LOW);
  digitalWrite(G_Led, LOW);
  digitalWrite(B_Led, LOW);  
}

byte indxKeyInROM(byte buf[]){ //возвращает индекс или ноль если нет в ROM
  byte buf1[8]; bool eq = true;
  for (byte j = 1; j<=EEPROM_key_count; j++){  // ищем ключ в eeprom. 
    EEPROM.get(j*sizeof(buf1), buf1);
    for (byte i = 0; i < 8; i++) 
      if (buf1[i] != buf[i]) { eq = false; break;}
    if (eq) return j;
    eq = true;
  }
  return 0;
}

bool EPPROM_AddKey(byte buf[]){
  byte buf1[8]; byte indx;
  indx = indxKeyInROM(buf);                 // ищем ключ в eeprom. Если находим, то не делаем запись, а индекс переводим в него
  if ( indx != 0) { 
    EEPROM_key_index = indx;
    EEPROM.update(1, EEPROM_key_index);
    return false; 
  }
  if (EEPROM_key_count <= maxKeyCount) EEPROM_key_count++;
  if (EEPROM_key_count < maxKeyCount) EEPROM_key_index = EEPROM_key_count;
    else EEPROM_key_index++;
  if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
  Serial.println(F("Adding to EEPROM"));
  for (byte i = 0; i < 8; i++) {
    buf1[i] = buf[i];
    Serial.print(buf[i], HEX); Serial.print(F(":"));  
  }
  Serial.println();
  EEPROM.put(EEPROM_key_index*sizeof(buf1), buf1);
  EEPROM.update(0, EEPROM_key_count);
  EEPROM.update(1, EEPROM_key_index);
  return true;
}

void EEPROM_get_key(byte EEPROM_key_index1, byte buf[8]){
  byte buf1[8];
  int address = EEPROM_key_index1*sizeof(buf1);
  if (address > EEPROM.length()) return;
  EEPROM.get(address, buf1);
  for (byte i = 0; i < 8; i++) buf[i] = buf1[i];
  keyType = getKeyType(buf1);
}

emkeyType getKeyType(byte* buf){
  if (buf[0] == 0x01) return keyDallas;                       // это ключ формата dallas
  if ((buf[0] >> 4) == 0b0001) return keyCyfral;
  if ((buf[0] >> 4) == 0b0010) return keyMetacom;
  if ((buf[0] == 0xFF) && vertEvenCheck(buf)) return keyEM_Marine;
  return keyUnknown;
}

//*************** dallas **************
emRWType getRWtype(){    
   byte answer;
  // TM01 это неизвестный тип болванки, делается попытка записи TM-01 без финализации для dallas или c финализацией под cyfral или metacom
  // RW1990_1 - dallas-совместимые RW-1990, RW-1990.1, ТМ-08, ТМ-08v2 
  // RW1990_2 - dallas-совместимая RW-1990.2
  // TM2004 - dallas-совместимая TM2004 в доп. памятью 1кб
  // пробуем определить RW-1990.1
  ibutton.reset(); ibutton.write(0xD1); // проуем снять флаг записи для RW-1990.1
  ibutton.write_bit(1);                 // записываем значение флага записи = 1 - отключаем запись
  delay(10); pinMode(iButtonPin, INPUT);
  ibutton.reset(); ibutton.write(0xB5); // send 0xB5 - запрос на чтение флага записи
  answer = ibutton.read();
  //Serial.print(F("\n Answer RW-1990.1: ")); Serial.println(answer, HEX);
  if (answer == 0xFE){
    Serial.println(F(" Type: dallas RW-1990.1 "));
    return RW1990_1;            // это RW-1990.1
  }
  // пробуем определить RW-1990.2
  ibutton.reset(); ibutton.write(0x1D);  // пробуем установить флаг записи для RW-1990.2 
  ibutton.write_bit(1);                  // записываем значение флага записи = 1 - включаем запись
  delay(10); pinMode(iButtonPin, INPUT);
  ibutton.reset(); ibutton.write(0x1E);  // send 0x1E - запрос на чтение флага записи
  answer = ibutton.read();
  if (answer == 0xFE){
    ibutton.reset(); ibutton.write(0x1D); // возвращаем оратно запрет записи для RW-1990.2
    ibutton.write_bit(0);                 // записываем значение флага записи = 0 - выключаем запись
    delay(10); pinMode(iButtonPin, INPUT);
    Serial.println(F(" Type: dallas RW-1990.2 "));
    return RW1990_2; // это RW-1990.2
  }
  // пробуем определить TM-2004
  ibutton.reset(); ibutton.write(0x33);                     // посылаем команду чтения ROM для перевода в расширенный 3-х байтовый режим
  for ( byte i=0; i<8; i++) ibutton.read();                 // читаем данные ключа
  ibutton.write(0xAA);                                      // пробуем прочитать регистр статуса для TM-2004    
  ibutton.write(0x00); ibutton.write(0x00);                 // передаем адрес для считывания
  answer = ibutton.read();                                  // читаем CRC комманды и адреса
  byte m1[3] = {0xAA, 0,0};                                 // вычисляем CRC комманды
  if (OneWire::crc8(m1, 3) == answer) {
    answer = ibutton.read();                                  // читаем регистр статуса
    //Serial.print(" status: "); Serial.println(answer, HEX);
    Serial.println(F(" Type: dallas TM2004"));
    ibutton.reset();
    return TM2004; // это Type: TM2004
  }
  ibutton.reset();
  Serial.println(F(" Type: dallas unknown, trying TM-01! "));
  return TM01;                              // это неизвестный тип DS1990, нужно перебирать алгоритмы записи (TM-01)
}

bool write2iBtnTM2004(){                // функция записи на TM2004
  byte answer; bool result = true;
  ibutton.reset();
  ibutton.write(0x3C);                                      // команда записи ROM для TM-2004    
  ibutton.write(0x00); ibutton.write(0x00);                 // передаем адрес с которого начинается запись
  for (byte i = 0; i<8; i++){
    digitalWrite(R_Led, !digitalRead(R_Led));
    ibutton.write(keyID[i]);
    answer = ibutton.read();
    //if (OneWire::crc8(m1, 3) != answer){result = false; break;}     // crc не верный
    delayMicroseconds(600); ibutton.write_bit(1); delay(50);         // испульс записи
    pinMode(iButtonPin, INPUT);
    Serial.print('*');
    Sd_WriteStep();
    if (keyID[i] != ibutton.read()) { result = false; break;}    //читаем записанный байт и сравниваем, с тем что должно записаться
  } 
  if (!result){
    ibutton.reset();
    Serial.println(F(" The key copy faild"));
    OLED_printError(F("The key copy faild"));
    Sd_ErrorBeep();
    digitalWrite(R_Led, HIGH);
    return false;    
  }
  ibutton.reset();
  Serial.println(F(" The key has copied successesfully"));
  OLED_printError(F("The key has copied"), false);
  Sd_ReadOK();
  delay(2000);
  digitalWrite(R_Led, HIGH);
  return true;
}

bool write2iBtnRW1990_1_2_TM01(emRWType rwType){              // функция записи на RW1990.1, RW1990.2, TM-01C(F)
  byte rwCmd, bitCnt = 64, rwFlag = 1;
  switch (rwType){
    case TM01: rwCmd = 0xC1; if ((keyType == keyMetacom)||(keyType == keyCyfral)) bitCnt = 36; break;                   //TM-01C(F)
    case RW1990_1: rwCmd = 0xD1; rwFlag = 0; break;  // RW1990.1  флаг записи инвертирован
    case RW1990_2: rwCmd = 0x1D; break;              // RW1990.2
  }
  ibutton.reset(); ibutton.write(rwCmd);       // send 0xD1 - флаг записи
  ibutton.write_bit(rwFlag);                   // записываем значение флага записи = 1 - разрешить запись
  delay(5); pinMode(iButtonPin, INPUT);
  ibutton.reset(); 
  if (rwType == TM01) ibutton.write(0xC5);
    else ibutton.write(0xD5);        // команда на запись
  if (bitCnt == 36) BurnByteMC(keyID);
  else for (byte i = 0; i< (bitCnt >> 3); i++){
    digitalWrite(R_Led, !digitalRead(R_Led));
    if (rwType == RW1990_1) BurnByte(~keyID[i]);      // запись происходит инверсно для RW1990.1
      else BurnByte(keyID[i]);
    Serial.print('*');
    Sd_WriteStep();
  }
  if (bitCnt == 64) {
      ibutton.write(rwCmd);                     // send 0xD1 - флаг записи
      ibutton.write_bit(!rwFlag);               // записываем значение флага записи = 1 - отключаем запись
      delay(5); pinMode(iButtonPin, INPUT);
    }
  digitalWrite(R_Led, LOW);       
  if (!dataIsBurningOK(bitCnt)){                  // проверяем корректность записи
    Serial.println(F(" The key copy faild"));
    OLED_printError(F("The key copy faild"));
    Sd_ErrorBeep();
    digitalWrite(R_Led, HIGH);
    return false;
  }
  Serial.println(F(" The key has copied successesfully"));
  if ((keyType == keyMetacom)||(keyType == keyCyfral)){      //переводим ключ из формата dallas
    ibutton.reset();
    if (keyType == keyCyfral) ibutton.write(0xCA);       // send 0xCA - флаг финализации Cyfral
      else ibutton.write(0xCB);                       // send 0xCB - флаг финализации metacom
    ibutton.write_bit(1);                             // записываем значение флага финализации = 1 - перевезти формат
    delay(10); pinMode(iButtonPin, INPUT);
  }
  OLED_printError(F("The key has copied"), false);
  Sd_ReadOK();
  delay(2000);
  digitalWrite(R_Led, HIGH);
  return true;
}

void BurnByte(byte data){
  for(byte n_bit = 0; n_bit < 8; n_bit++){ 
    ibutton.write_bit(data & 1);  
    delay(5);                        // даем время на прошивку каждого бита до 10 мс
    data = data >> 1;                // переходим к следующему bit
  }
  pinMode(iButtonPin, INPUT);
}

void BurnByteMC(byte buf[8]){
  byte j = 0;
  for(byte n_bit = 0; n_bit < 36; n_bit++){ 
    ibutton.write_bit(((~buf[n_bit>>3]) >> (7-j) ) & 1);  
    delay(5);                        // даем время на прошивку каждого бита 5 мс
    j++;
    if (j > 7) j = 0;
  }
  pinMode(iButtonPin, INPUT);
}

void convetr2MC(byte buff[8]){
  byte data;
  for (byte i = 0; i < 5; i++){
    data = ~buff[i];
    buff[i] = 0;
    for (byte j = 0; j < 8; j++) 
      if ( (data>>j)&1) bitSet(buff[i], 7-j);
  }
  buff[4] &= 0xf0;  buff[5] = 0; buff[6] = 0; buff[7] = 0;
}

bool dataIsBurningOK(byte bitCnt){
  byte buff[8];
  if (!ibutton.reset()) return false;
  ibutton.write(0x33);
  ibutton.read_bytes(buff, 8);
  if (bitCnt == 36) convetr2MC(buff);
  byte Check = 0;
  for (byte i = 0; i < 8; i++){ 
    if (keyID[i] == buff[i]) Check++;       // сравниваем код для записи с тем, что уже записано в ключе.
    Serial.print(buff[i], HEX); Serial.print(":");
  }
  if (Check != 8) return false;             // если коды совпадают, ключ успешно скопирован
  return true;
}

bool write2iBtn(){
  int Check = 0;
  if (!ibutton.search(addr)) { 
    ibutton.reset_search(); 
    return false;
  }
  Serial.print(F("The new key code is: "));
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); Serial.print(":");  
    if (keyID[i] == addr[i]) Check++;         // сравниваем код для записи с тем, что уже записано в ключе.
  }
  if (Check == 8) {                           // если коды совпадают, ничего писать не нужно
    digitalWrite(R_Led, LOW); 
    Serial.println(F(" it is the same key. Writing in not needed."));
    OLED_printError(F("It is the same key"));
    Sd_ErrorBeep();
    digitalWrite(R_Led, HIGH);
    delay(1000);
    return false;
  }
  emRWType rwType = getRWtype();                    // определяем тип RW-1990.1 или 1990.2 или TM-01
  Serial.print(F("\n Burning iButton ID: "));
  if (rwType == TM2004) return write2iBtnTM2004();  //шьем TM2004
    else return write2iBtnRW1990_1_2_TM01(rwType);  //пробуем прошить другие форматы
}

bool searchIbutton(){
  if (!ibutton.search(addr)) { 
    ibutton.reset_search(); 
    return false;
  }  
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); Serial.print(":");
    keyID[i] = addr[i];                               // копируем прочтенный код в ReadID
  }
  if (addr[0] == 0x01) {                              // это ключ формата dallas
    keyType = keyDallas;
    if (getRWtype() == TM2004) keyType = keyTM2004;
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println(F("CRC is not valid!"));
      OLED_printError(F("CRC is not valid!"));
      Sd_ErrorBeep();
      digitalWrite(B_Led, HIGH);
      return false;
    }
    return true;
  }
  switch (addr[0]>>4){
    case 1: Serial.println(F(" Type: May be cyfral in dallas key")); break;      
    case 2: Serial.println(F(" Type: May be metacom in dallas key"));  break;  
    case 3: Serial.println(F(" Type: unknown family dallas")); break;             
  }
  keyType = keyUnknown;
  return true;
}

//************ Cyfral ***********************
unsigned long pulseACompA(bool pulse, byte Average = 80, unsigned long timeOut = 1500){  // pulse HIGH or LOW
  bool AcompState;
  unsigned long tEnd = micros() + timeOut;
  do {
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    if (ADCH > 200) return 0;
    if (ADCH > Average) AcompState = HIGH;  // читаем флаг компаратора
      else AcompState = LOW;
    if (AcompState == pulse) {
      tEnd = micros() + timeOut;
      do {
          ADCSRA |= (1<<ADSC);
          while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
        if (ADCH > Average) AcompState = HIGH;  // читаем флаг компаратора
          else AcompState = LOW;
        if (AcompState != pulse) return (unsigned long)(micros() + timeOut - tEnd);  
      } while (micros() < tEnd);
      return 0;                                                 //таймаут, импульс не вернуся оратно
    }             // end if
  } while (micros() < tEnd);
  return 0;
}

void ADCsetOn(){
  ADMUX = (ADMUX&0b11110000) | 0b0011 | (1<<ADLAR);// (1 << REFS0);          // подключаем к AC Линию A3 ,  левое выравние, измерение до Vcc
  ADCSRB = (ADCSRB & 0b11111000) | (1<<ACME);                // источник перезапуска ADC FreeRun, включаем мультиплексор AC
  ADCSRA = (ADCSRA & 0b11111000) |0b011 | (1<<ADEN) | (1<<ADSC);// | (1<<ADATE);      // 0b011 делитель скорости ADC, // включаем ADC и запускаем ADC и autotriger ADC 
}

void ACsetOn(){
  ACSR |= 1<<ACBG;                            // Подключаем ко входу Ain0 1.1V для Cyfral/Metacom
  ADCSRA &= ~(1<<ADEN);                       // выключаем ADC
  ADMUX = (ADMUX&0b11110000) | 0b0011;        // подключаем к AC Линию A3
  ADCSRB |= 1<<ACME;                          // включаем мультиплексор AC
}

bool read_cyfral(byte* buf, byte CyfralPin){
  unsigned long ti; byte i=0, j = 0, k = 0;
  analogRead(iButtonPin);
  ADCsetOn(); 
  byte aver = calcAverage();
  unsigned long tEnd = millis() + 30;
  do{
    ti = pulseACompA(HIGH, aver);
    if ((ti == 0) || (ti > 260) || (ti < 10)) {i = 0; j=0; k = 0; continue;}
    if ((i < 3) && (ti > halfT)) {i = 0; j = 0; k = 0; continue;}      //контроль стартовой последовательности 0b0001
    if ((i == 3) && (ti < halfT)) continue;      
    if (ti > halfT) bitSet(buf[i >> 3], 7-j);
      else if (i > 3) k++; 
    if ((i > 3) && ((i-3)%4 == 0) ){        //начиная с 4-го бита проверяем количество нулей каждой строки из 4-и бит
      if (k != 1) {for (byte n = 0; n < (i >> 3)+2; n++) buf[n] = 0; i = 0; j = 0; k = 0; continue;}        //если нулей больше одной - начинаем сначала 
      k = 0; 
    }
    j++; if (j>7) j=0;
    i++;
  } while ((millis() < tEnd) && (i < 36));
  if (i < 36) return false;
  return true;
}

bool searchCyfral(){
  byte buf[8];
  for (byte i = 0; i < 8; i++) {addr[i] =0; buf[i] = 0;}
  if (!read_cyfral(addr, iButtonPin)) return false;
  if (!read_cyfral(buf, iButtonPin)) return false;
  for (byte i = 0; i < 8; i++) 
    if (addr[i] != buf[i]) return false;
  keyType = keyCyfral;
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); Serial.print(":");
    keyID[i] = addr[i];                                         // копируем прочтенный код в ReadID
  }
  Serial.println(F(" Type: Cyfral "));
  return true;  
}

byte calcAverage(){
  unsigned int sum = 127; byte preADCH = 0, j = 0; 
  for (byte i = 0; i<255; i++) {
    ADCSRA |= (1<<ADSC);
    delayMicroseconds(10);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    sum += ADCH;
  }
  sum = sum >> 8;
  unsigned long tSt = micros();
  for (byte i = 0; i<255; i++) {
    delayMicroseconds(4);
    ADCSRA |= (1<<ADSC);
    while(ADCSRA & (1 << ADSC)); // Wait until the ADSC bit has been cleared
    if (((ADCH > sum)&&(preADCH < sum)) | ((ADCH < sum)&&(preADCH > sum))) {
      j++;
      preADCH = ADCH;
    }   
  }
  halfT = (byte)((micros() - tSt) / j);
  return (byte)sum;
}

bool read_metacom(byte* buf, byte MetacomPin){
  unsigned long ti; byte i = 0, j = 0, k = 0;
  analogRead(iButtonPin);
  ADCsetOn();
  byte aver = calcAverage();
  unsigned long tEnd = millis() + 30;
  do{
    ti = pulseACompA(LOW, aver);
    if ((ti == 0) || (ti > 500)) {i = 0; j=0; k = 0; continue;}
    if ((i == 0) && (ti+30 < (halfT<<1))) continue;      //вычисляем период;
    if ((i == 2) && (ti > halfT)) {i = 0; j = 0;  continue;}      //вычисляем период;
    if (((i == 1) || (i == 3)) && (ti < halfT)) {i = 0; j = 0; continue;}      //вычисляем период;
    if (ti < halfT) {   
      bitSet(buf[i >> 3], 7-j);
      if (i > 3) k++;                             // считаем кол-во единиц
    }
    if ((i > 3) && ((i-3)%8 == 0) ){        //начиная с 4-го бита проверяем контроль четности каждой строки из 8-и бит
      if (k & 1) { for (byte n = 0; n < (i >> 3)+1; n++) buf[n] = 0; i = 0; j = 0;  k = 0; continue;}              //если нечетно - начинаем сначала
      k = 0;
    }   
    j++; if (j>7) j=0;
    i++;
  }  while ((millis() < tEnd) && (i < 36));
  if (i < 36) return false;
  return true;
}

bool searchMetacom(){
  byte buf[8];
  for (byte i = 0; i < 8; i++) {addr[i] =0; buf[i] = 0;}
  if (!read_metacom(addr, iButtonPin)) return false;
  if (!read_metacom(buf, iButtonPin)) return false;
  for (byte i = 0; i < 8; i++) 
    if (addr[i] != buf[i]) return false; 
  keyType = keyMetacom;
  for (byte i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX); Serial.print(":");
    keyID[i] = addr[i];                               // копируем прочтенный код в ReadID
  }
  Serial.println(F(" Type: Metacom "));
  return true;  
}

//**********EM-Marine***************************
bool vertEvenCheck(byte* buf){        // проверка четности столбцов с данными
  byte k;
  k = 1&buf[1]>>6 + 1&buf[1]>>1 + 1&buf[2]>>4 + 1&buf[3]>>7 + 1&buf[3]>>2 + 1&buf[4]>>5 + 1&buf[4] + 1&buf[5]>>3 + 1&buf[6]>>6 + 1&buf[6]>>1 + 1&buf[7]>>4;
  if (k&1) return false;
  k = 1&buf[1]>>5 + 1&buf[1] + 1&buf[2]>>3 + 1&buf[3]>>6 + 1&buf[3]>>1 + 1&buf[4]>>4 + 1&buf[5]>>7 + 1&buf[5]>>2 + 1&buf[6]>>5 + 1&buf[6] + 1&buf[7]>>3;
  if (k&1) return false;
  k = 1&buf[1]>>4 + 1&buf[2]>>7 + 1&buf[2]>>2 + 1&buf[3]>>5 + 1&buf[3] + 1&buf[4]>>3 + 1&buf[5]>>6 + 1&buf[5]>>1 + 1&buf[6]>>4 + 1&buf[7]>>7 + 1&buf[7]>>2;
  if (k&1) return false;
  k = 1&buf[1]>>3 + 1&buf[2]>>6 + 1&buf[2]>>1 + 1&buf[3]>>4 + 1&buf[4]>>7 + 1&buf[4]>>2 + 1&buf[5]>>5 + 1&buf[5] + 1&buf[6]>>3 + 1&buf[7]>>6 + 1&buf[7]>>1;
  if (k&1) return false;
  if (1&buf[7]) return false;
  //номер ключа, который написан на корпусе
  rfidData[0] = (0b01111000&buf[1])<<1 | (0b11&buf[1])<<2 | buf[2]>>6;
  rfidData[1] = (0b00011110&buf[2])<<3 | buf[3]>>4;
  rfidData[2] = buf[3]<<5 | (0b10000000&buf[4])>>3 | (0b00111100&buf[4])>>2;
  rfidData[3] = buf[4]<<7 | (0b11100000&buf[5])>>1 | 0b1111&buf[5];
  rfidData[4] = (0b01111000&buf[6])<<1 | (0b11&buf[6])<<2 | buf[7]>>6;
  return true;
}

byte ttAComp(unsigned long timeOut = 7000){  // pulse 0 or 1 or -1 if timeout
  byte AcompState, AcompInitState;
  unsigned long tEnd = micros() + timeOut;
  AcompInitState = (ACSR >> ACO)&1;               // читаем флаг компаратора
  do {
    AcompState = (ACSR >> ACO)&1;                 // читаем флаг компаратора
    if (AcompState != AcompInitState) {
      delayMicroseconds(1000/(rfidBitRate*4));    // 1/4 Period on 2 kBps = 125 mks 
      AcompState = (ACSR >> ACO)&1;               // читаем флаг компаратора      
      delayMicroseconds(1000/(rfidBitRate*2));    // 1/2 Period on 2 kBps = 250 mks 
      return AcompState;  
    }
  } while (micros() < tEnd);
  return 2;                                             //таймаут, компаратор не сменил состояние
}

bool readEM_Marie(byte* buf){
  unsigned long tEnd = millis() + 50;
  byte ti; byte j = 0, k=0;
  for (int i = 0; i<64; i++){    // читаем 64 bit
    ti = ttAComp();
    if (ti == 2)  break;         //timeout
    if ( ( ti == 0 ) && ( i < 9)) {  // если не находим 9 стартовых единиц - начинаем сначала
      if (millis() > tEnd) { ti=2; break;}  //timeout
      i = -1; j=0; continue;
    }
    if ((i > 8) && (i < 59)){     //начиная с 9-го бита проверяем контроль четности каждой строки
      if (ti) k++;                // считаем кол-во единиц
      if ( (i-9)%5 == 4 ){        // конец строки с данными из 5-и бит, 
        if (k & 1) {              //если нечетно - начинаем сначала
          i = -1; j = 0; k = 0; continue; 
        }
        k = 0;
      }
    }
    if (ti) bitSet(buf[i >> 3], 7-j);
      else bitClear(buf[i >> 3], 7-j);
    j++; if (j>7) j=0; 
  }
  if (ti == 2) return false;         //timeout
  return vertEvenCheck(buf);
}

void rfidACsetOn(){
  //включаем генератор 125кГц
  pinMode(FreqGen, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);  //Вкючаем режим Toggle on Compare Match на COM2A (pin 11) и счет таймера2 до OCR2A
  TCCR2B = _BV(WGM22) | _BV(CS20);                                // Задаем делитель для таймера2 = 1 (16 мГц)
  OCR2A = 63;                                                    // 63 тактов на период. Частота на COM2A (pin 11) 16000/64/2 = 125 кГц, Скважнось COM2A в этом режиме всегда 50% 
  OCR2B = 31;                                                     // Скважность COM2B 32/64 = 50%  Частота на COM2A (pin 3) 16000/64 = 250 кГц
  // включаем компаратор
  ADCSRB &= ~(1<<ACME);           // отключаем мультиплексор AC
  ACSR &= ~(1<<ACBG);             // отключаем от входа Ain0 1.1V
}

bool searchEM_Marine( bool copyKey = true){
  byte gr = digitalRead(G_Led);
  bool rez = false;
  rfidACsetOn();            // включаем генератор 125кГц и компаратор
  delay(6);                //13 мс длятся переходные прцессы детектора 
  if (!readEM_Marie(addr)) {
    if (!copyKey) TCCR2A &=0b00111111;              //Оключить ШИМ COM2A (pin 11)
    digitalWrite(G_Led, gr);
    return rez;
  }
  rez = true;
  keyType = keyEM_Marine;
  for (byte i = 0; i<8; i++){
    if (copyKey) keyID[i] = addr [i];
    Serial.print(addr[i], HEX); Serial.print(":");
  }
  Serial.print(F(" ( id "));
  Serial.print(rfidData[0]); Serial.print(" key ");
  unsigned long keyNum = (unsigned long)rfidData[1]<<24 | (unsigned long)rfidData[2]<<16 | (unsigned long)rfidData[3]<<8 | (unsigned long)rfidData[4];
  Serial.print(keyNum);
  Serial.println(F(") Type: EM-Marie "));
  if (!copyKey) TCCR2A &=0b00111111;              //Оключить ШИМ COM2A (pin 11)
  digitalWrite(G_Led, gr);
  return rez;
}

void TxBitRfid(byte data){
  if (data & 1) delayMicroseconds(54*8); 
    else delayMicroseconds(24*8);
  rfidGap(19*8);                       //write gap
}

void TxByteRfid(byte data){
  for(byte n_bit=0; n_bit<8; n_bit++){
    TxBitRfid(data & 1);
    data = data >> 1;                   // переходим к следующему bit
  }
}

void rfidGap(unsigned int tm){
  TCCR2A &=0b00111111;                //Оключить ШИМ COM2A 
  delayMicroseconds(tm);
  TCCR2A |= _BV(COM2A0);              // Включить ШИМ COM2A (pin 11)      
}

bool T5557_blockRead(byte* buf){
  byte ti; byte j = 0, k=0;
  for (int i = 0; i<33; i++){                           // читаем стартовый 0 и 32 значащих bit
    ti = ttAComp(2000);
    if (ti == 2)  break;                                //timeout
    if ( ( ti == 1 ) && ( i == 0)) { ti=2; break; }     // если не находим стартовый 0 - это ошибка
    if (i > 0){                                         //начиная с 1-го бита пишем в буфер
      if (ti) bitSet(buf[(i-1) >> 3], 7-j);
        else bitClear(buf[(i-1) >> 3], 7-j);
      j++; if (j>7) j=0;
    }
  }
  if (ti == 2) return false;                           //timeout
  return true;
}

bool sendOpT5557(byte opCode, unsigned long password = 0, byte lockBit = 0, unsigned long data = 0, byte blokAddr = 1){
  TxBitRfid(opCode >> 1); TxBitRfid(opCode & 1); // передаем код операции 10
  if (opCode == 0b00) return true;
  // password
  TxBitRfid(lockBit & 1);               // lockbit 0
  if (data != 0){
    for (byte i = 0; i<32; i++) {
      TxBitRfid((data>>(31-i)) & 1);
    }
  }
  TxBitRfid(blokAddr>>2); TxBitRfid(blokAddr>>1); TxBitRfid(blokAddr & 1);      // адрес блока для записи
  delay(4);                                                                     // ждем пока пишутся данные
  return true;
}

bool write2rfidT5557(byte* buf){
  bool result; unsigned long data32;
  delay(6);
  for (byte k = 0; k<2; k++){                                       // send key data
    data32 = (unsigned long)buf[0 + (k<<2)]<<24 | (unsigned long)buf[1 + (k<<2)]<<16 | (unsigned long)buf[2 + (k<<2)]<<8 | (unsigned long)buf[3 + (k<<2)];
    rfidGap(30 * 8);                                                 //start gap
    sendOpT5557(0b10, 0, 0, data32, k+1);                            //передаем 32 бита ключа в blok k
    Serial.print('*'); delay(6);
  }
  delay(6);
  rfidGap(30 * 8);                  //start gap
  sendOpT5557(0b00);
  delay(4);
  result = readEM_Marie(addr);
  TCCR2A &=0b00111111;              //Оключить ШИМ COM2A (pin 11)
  for (byte i = 0; i < 8; i++)
    if (addr[i] != keyID[i]) { result = false; break; }
  if (!result){
    Serial.println(F(" The key copy faild"));
    OLED_printError(F("The key copy faild"));
    Sd_ErrorBeep();
  } else {
    Serial.println(F(" The key has copied successesfully"));
    OLED_printError(F("The key has copied"), false);
    Sd_ReadOK();
    delay(2000);
  }
  digitalWrite(R_Led, HIGH);
  return result;  
}

emRWType getRfidRWtype(){
  unsigned long data32, data33; byte buf[4] = {0, 0, 0, 0}; 
  rfidACsetOn();                // включаем генератор 125кГц и компаратор
  delay(13);                    //13 мс длятся переходные процессы детектора
  rfidGap(30 * 8);              //start gap
  sendOpT5557(0b11, 0, 0, 0, 1); //переходим в режим чтения Vendor ID 
  if (!T5557_blockRead(buf)) return rwUnknown; 
  data32 = (unsigned long)buf[0]<<24 | (unsigned long)buf[1]<<16 | (unsigned long)buf[2]<<8 | (unsigned long)buf[3];
  delay(4);
  rfidGap(20 * 8);          //gap  
  data33 = 0b00000000000101001000000001000000 | (rfidUsePWD << 4);   //конфиг регистр 0b00000000000101001000000001000000
  sendOpT5557(0b10, 0, 0, data33, 0);   //передаем конфиг регистр
  delay(4);
  rfidGap(30 * 8);          //start gap
  sendOpT5557(0b11, 0, 0, 0, 1); //переходим в режим чтения Vendor ID 
  if (!T5557_blockRead(buf)) return rwUnknown; 
  data33 = (unsigned long)buf[0]<<24 | (unsigned long)buf[1]<<16 | (unsigned long)buf[2]<<8 | (unsigned long)buf[3];
  sendOpT5557(0b00, 0, 0, 0, 0);  // send Reset
  delay(6);
  if (data32 != data33) return rwUnknown;    
  Serial.print(F(" The rfid RW-key is T5557. Vendor ID is "));
  Serial.println(data32, HEX);
  return T5557;
}

bool write2rfid(){
  bool Check = true;
  if (searchEM_Marine(false)) {
    for (byte i = 0; i < 8; i++)
      if (addr[i] != keyID[i]) { Check = false; break; }  // сравниваем код для записи с тем, что уже записано в ключе.
    if (Check) {                                          // если коды совпадают, ничего писать не нужно
      digitalWrite(R_Led, LOW); 
      Serial.println(F(" it is the same key. Writing in not needed."));
      OLED_printError(F("It is the same key"));
      Sd_ErrorBeep();
      digitalWrite(R_Led, HIGH);
      delay(1000);
      return false;
    }
  }
  emRWType rwType = getRfidRWtype(); // определяем тип T5557 (T5577) или EM4305
  if (rwType != rwUnknown) Serial.print(F("\n Burning rfid ID: "));
  switch (rwType){
    case T5557: return write2rfidT5557(keyID); break;                    //пишем T5557
    //case EM4305: return write2rfidEM4305(keyID); break;                  //пишем EM4305
    case rwUnknown: break;
  }
  return false;
}


void SendEM_Marine(byte* buf){ 
  TCCR2A &=0b00111111; // отключаем шим 
  digitalWrite(FreqGen, LOW);
  //FF:A9:8A:A4:87:78:98:6A
  delay(20);
  for (byte k = 0; k<10; k++){
    for (byte i = 0; i<8; i++){
      for (byte j = 0; j<8; j++){
        if (1 & (buf[i]>>(7-j))) {
          pinMode(FreqGen, INPUT);
          delayMicroseconds(250);
          pinMode(FreqGen, OUTPUT); 
          delayMicroseconds(250);
        } else {
          pinMode(FreqGen, OUTPUT);
          delayMicroseconds(250);
          pinMode(FreqGen, INPUT);
          delayMicroseconds(250);
        }
      }
    }
   // delay(1);
  }  
}

void SendDallas(byte* buf){
/*  iBtnEmul.init(buf);
  //iBtnEmul.waitForRequest(false);
  unsigned long tStart = millis();
  do {
    if (!iBtnEmul.waitReset(10) ) continue;
    if (!iBtnEmul.presence() ) continue;
    if (iBtnEmul.recvAndProcessCmd() ) break;
  } while (millis() < 200 + tStart);  */
}

void BM_SendKey(byte* buf){
  switch (keyType){
    case keyEM_Marine: SendEM_Marine(buf); break;
    default: SendDallas(buf); break;
  }
}

unsigned long stTimer = millis();
void loop() {
  char echo = Serial.read();
  if (echo == 'e'){
    myOLED.print(F("EEPROM cleared success!"), 0, 0);
    Serial.println(F("EEPROM cleared"));
    EEPROM.update(0, 0); EEPROM.update(1, 0);
    EEPROM_key_count = 0; EEPROM_key_index = 0;
    Sd_ReadOK();
    myOLED.update();
  }
  if ((echo == 't') || enc1.isClick()) {  // переключаель режима чтение/запись
    switch (copierMode){
      case md_empty: Sd_ErrorBeep(); break;
      case md_read: copierMode = md_write; clearLed(); digitalWrite(R_Led, HIGH);  break;
      case md_write: copierMode = md_blueMode; clearLed(); digitalWrite(B_Led, HIGH); 
        digitalWrite(Luse_Led, !digitalRead(Luse_Led)); break;
      case md_blueMode: copierMode = md_read; clearLed(); digitalWrite(G_Led, HIGH); 
        digitalWrite(Luse_Led, !digitalRead(Luse_Led)); break;
    }
    OLED_printKey(keyID);
    Serial.print(F("Mode: ")); Serial.println(copierMode);
    Sd_WriteStep();
  }
  if (enc1.isLeft() && (EEPROM_key_count > 0)){       //при повороте энкодера листаем ключи из eeprom
    EEPROM_key_index--;
    if (EEPROM_key_index < 1) EEPROM_key_index = EEPROM_key_count;
    EEPROM_get_key(EEPROM_key_index, keyID);
    OLED_printKey(keyID);
    Sd_WriteStep();
  }
  if (enc1.isRight() && (EEPROM_key_count > 0)){
    EEPROM_key_index++;
    if (EEPROM_key_index > EEPROM_key_count) EEPROM_key_index = 1;
    EEPROM_get_key(EEPROM_key_index, keyID);
    OLED_printKey(keyID);
    Sd_WriteStep();            
  }
  if ((copierMode != md_empty) && enc1.isHolded()){     // Если зажать кнопкку - ключ сохранися в EEPROM
    if (EPPROM_AddKey(keyID)) {
      OLED_printError(F("The key saved"), false);
      Sd_ReadOK();
      delay(1000); 
    }
      else Sd_ErrorBeep();
    OLED_printKey(keyID);  
  }   
  if (millis() - stTimer < 100) return; //задержка в 100 мс
  stTimer = millis();
  switch (copierMode){
      case md_empty: case md_read: 
        if (searchCyfral() || searchMetacom() || searchEM_Marine() || searchIbutton() ){     // запускаем поиск cyfral, затем поиск EM_Marine, затем поиск dallas
          //keyID[0] = 0xFF; keyID[1] = 0xA9; keyID[2] =  0x8A; keyID[3] = 0xA4; keyID[4] = 0x87; keyID[5] = 0x78; keyID[6] = 0x98; keyID[7] = 0x6A;
          Sd_ReadOK();
          copierMode = md_read;
          digitalWrite(G_Led, HIGH);
          if (indxKeyInROM(keyID) == 0) OLED_printKey(keyID, 1);
            else OLED_printKey(keyID, 3);
          } 
        break;
      case md_write:
        if (keyType == keyEM_Marine) write2rfid();
          else write2iBtn(); 
        break;
      case md_blueMode: 
        BM_SendKey(keyID);
        break;
    } //end switch
}

//***************** звуки****************
void Sd_ReadOK() {  // звук ОК
  for (int i=400; i<6000; i=i*1.5) { tone(speakerPin, i); delay(20); }
  noTone(speakerPin);
}

void Sd_WriteStep(){  // звук "очередной шаг"
  for (int i=2500; i<6000; i=i*1.5) { tone(speakerPin, i); delay(10); }
  noTone(speakerPin);
}

void Sd_ErrorBeep() {  // звук "ERROR"
  for (int j=0; j <3; j++){
    for (int i=1000; i<2000; i=i*1.1) { tone(speakerPin, i); delay(10); }
    delay(50);
    for (int i=1000; i>500; i=i*1.9) { tone(speakerPin, i); delay(10); }
    delay(50);
  }
  noTone(speakerPin);
}

void Sd_StartOK(){   // звук "Успешное включение"
  tone(speakerPin, NOTE_A7); delay(100);
  tone(speakerPin, NOTE_G7); delay(100);
  tone(speakerPin, NOTE_E7); delay(100); 
  tone(speakerPin, NOTE_C7); delay(100);  
  tone(speakerPin, NOTE_D7); delay(100); 
  tone(speakerPin, NOTE_B7); delay(100); 
  tone(speakerPin, NOTE_F7); delay(100); 
  tone(speakerPin, NOTE_C7); delay(100);
  noTone(speakerPin); 
}
