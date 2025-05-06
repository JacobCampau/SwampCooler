//Nick Campbell, Jacob Campau, Nathaniel Dulcero

#include <LiquidCrystal.h>
#include <Stepper.h>
#include <dht.h>
#include <RTClib.h>

#define RDA 0x80
#define TBE 0x20

const int RS = 37, EN = 35, D4 = 33, D5 = 31, D6 = 29, D7 = 27;

int speedPin = 46;
int dir1 = 42;
int dir2 = 44;
int mSpeed = 180;

unsigned int waterLevel;

unsigned int LOW_WATER_LEVEL_THRESHOLD = 10;
float TEMP_THRESHOLD = 79;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

dht DHT;
#define DHT11_PIN 12

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Saturday", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday"};

const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 45, 47, 49, 51);

float humidity;
float temperature;

enum State{DISABLED, IDLE, ERROR, RUNNING};
State state = DISABLED;
volatile bool buttonPressed = false;

unsigned long previousMillis = 0;
const long interval = 60000;

void setup(){
  U0init(9600);

  DDRE |= (1 << PE3); //Green LED set to output
  DDRE |= (1 << PE4); //Red LED set to output
  DDRE |= (1 << PE5); //Blue LED set to output
  DDRH |= (1 << PH5); //Yellow LED set to output

  DDRD &= ~(1 << PD3); // Set Start button as input
  PORTD |= (1 << PD3); // Pullup resistor active

  DDRH &= ~(1 << PH4); //Stop button input

  DDRH &= ~(1 << PH3); //Reset button input

  DDRB &= ~(1 << PB3); //Button for increase stepper as input
  DDRB &= ~(1 << PB1); //Button for decrease stepper as input

  DDRL |= (1 << PL3); //speedPin output
  DDRL |= (1 << PL7); //dir1 output
  DDRL |= (1 << PL5); //dir2 output

  attachInterrupt(digitalPinToInterrupt(18), startButtonISR, CHANGE);

  adc_init();
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  rtc.begin();

  stateChange();
  getDate();
}

void loop(){
  unsigned long currentMillis = millis();
  switch(state){
    case DISABLED:
      //Yellow LED on, others off
      PORTH |= (1 << PH5);
      PORTE &= ~(1 << PE3);
      PORTE &= ~(1 << PE4);
      PORTE &= ~(1 << PE5);
      //Don't monitor temp or water
      //Start button monitored using ISR
      if(buttonPressed){
        buttonPressed = false;
        state = IDLE;
        stateChange();
        getDate();
        break;
      }
      break;
    case IDL E:
      //Green LED, others off
      PORTE |= (1 << PE3);
      PORTH &= ~(1 << PH5);
      PORTE &= ~(1 << PE4);
      PORTE &= ~(1 << PE5);
      //Only monitor water level
      waterLevel = adc_read(0);
      if(waterLevel < LOW_WATER_LEVEL_THRESHOLD){
        state = ERROR;  // Transition to ERROR state if water is too low
        stateChange();
        getDate();
      }
      //Temperature every minute (Record transistion times)
      if(currentMillis - previousMillis >= interval){
        previousMillis = currentMillis;
        getDHT();
        recordOnLCD();
      }
      if(PINH & (1 << PH4)){
        resetLCD();
        state = DISABLED;
        stateChange();
        getDate();
        break;
      }
      if(temperature > TEMP_THRESHOLD){
        state = RUNNING;
        stateChange();
        getDate();
        break;
      }
      //Go to ERROR if water = low
      waterLevel = adc_read(0);
      if(waterLevel < LOW_WATER_LEVEL_THRESHOLD){
        state = ERROR;
        stateChange();
        getDate();
        break;
      }
      break;
    case ERROR:
      //Red LED, others off
      PORTE |= (1 << PE4);
      PORTH &= ~(1 << PH5);
      PORTE &= ~(1 << PE3);
      PORTE &= ~(1 << PE5);
      //Read for reset button
      if(PINH & (1 << PH3)){
        state = IDLE;
        stateChange();
        getDate();
        break;
      }
      //Read for stop button
      if(PINH & (1 << PH4)){
        resetLCD();
        state = DISABLED;
        stateChange();
        getDate();
        break;
      }
      //Error to LCD
      if(currentMillis - previousMillis >= interval){
        previousMillis = currentMillis;
        getDHT();
        resetLCD();
        lcd.write("Water level is");
        lcd.setCursor(0, 1);
        lcd.write("too low");
      }
      break;
    case RUNNING:
      //Blue LED, others off
      PORTE |= (1 << PE5);
      PORTH &= ~(1 << PH5);
      PORTE &= ~(1 << PE3);
      PORTE &= ~(1 << PE4);
      //Motor should be on
      startMotor();
      //Go to IDLE if temp < threshold
      if(currentMillis - previousMillis >= interval){
        previousMillis = currentMillis;
        getDHT();
        recordOnLCD();
      }
      if(temperature < TEMP_THRESHOLD){
        state = IDLE;
        stateChange();
        getDate();
        stopMotor();
        break;
      }
      //Go to ERROR if water = low
      waterLevel = adc_read(0);
      if(waterLevel < LOW_WATER_LEVEL_THRESHOLD){
        state = ERROR;
        stateChange();
        getDate();
        stopMotor();
        break;
      }
      if(PINH & (1 << PH4)){
        state = DISABLED;
        resetLCD();
        stateChange();
        getDate();
        stateChange();
        stopMotor();
        break;
      }
      break;
    default:
      break;
  }
  //Stepper motor
  if(PINB & (1 << PB3)){
    myStepper.setSpeed(5);
    myStepper.step(20);
    getDate();
    stepperChange("Right");
  }
  if(PINB & (1 << PB1)){
    myStepper.setSpeed(5);
    myStepper.step(-20);
    getDate();
    stepperChange("Left");
  }
}

void startMotor(){
  analogWrite(dir1, 255);
  analogWrite(dir2, 0);
  analogWrite(speedPin, mSpeed);
}

void stopMotor(){
  analogWrite(dir1, 0);
  analogWrite(dir2, 0);
  analogWrite(speedPin, 0);
}

void resetLCD(){
  lcd.clear();
  lcd.setCursor(0, 0);
}

void recordOnLCD(){
  resetLCD();
  lcd.write("Temp: ");
  lcd.print(temperature);
  lcd.write(" F");
  lcd.setCursor(0, 1);
  lcd.write("Hum: ");
  lcd.print(humidity);
  lcd.write(" %");
}

void getDHT(){
  int chk = DHT.read11(DHT11_PIN);
  temperature = DHT.temperature * 9.0 / 5.0 + 32.0;
  humidity = DHT.humidity;
}

void getDate(){
  DateTime now = rtc.now();
  
  U0putchar((now.year() / 1000) + '0');
  U0putchar(((now.year() / 100) % 10) + '0');
  U0putchar(((now.year() / 10) % 10) + '0');
  U0putchar((now.year() % 10) + '0');

  U0putchar('/');
  U0putchar(now.month() / 10 + '0');
  U0putchar(now.month() % 10 + '0');

  U0putchar('/');
  U0putchar((now.day() - 1) / 10 + '0');
  U0putchar((now.day() - 1) % 10 + '0');

  U0putchar(' ');
  U0putchar('(');

  char *dayName = daysOfTheWeek[now.dayOfTheWeek()];
  for(int i = 0; dayName[i] != '\0'; i++){
    U0putchar(dayName[i]);
  }

  U0putchar(')');

  U0putchar(' ');

  U0putchar((now.hour() - 3) / 10 + '0');
  U0putchar((now.hour() - 3) % 10 + '0');

  U0putchar(':');
  U0putchar((now.minute() - 2) / 10 + '0');
  U0putchar((now.minute()  - 2) % 10 + '0');

  U0putchar(':');
  U0putchar(now.second() / 10 + '0');
  U0putchar(now.second() % 10 + '0');

  U0putchar('\n');
}

void stepperChange(char* dir){
  char* message = "Stepper changed to the ";
  for(int i = 0; message[i] != '\0'; i++){
    U0putchar(message[i]);
  }
  for(int i = 0; dir[i] != '\0'; i++){
    U0putchar(dir[i]);
  }
  U0putchar('\n');
}

void stateChange(){
  char* message = "State changed to ";
  for(int i = 0; message[i] != '\0'; i++){
    U0putchar(message[i]);
  }

  char* st;
  switch(state){
    case DISABLED:
      st = "DISABLED";
      break;
    case IDLE:
      st = "IDLE";
      break;
    case ERROR:
      st = "ERROR";
      break;
    case RUNNING:
      st = "RUNNING";
      break;
    default:
      break;
  }
  for(int i = 0; st[i] != '\0'; i++){
    U0putchar(st[i]);
  }
  U0putchar('\n');
}

void startButtonISR(){
  buttonPressed = true;
}

void adc_init() //write your code after each commented line and follow the instruction 
{
  // setup the A register
 // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= 0x80;
 // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0xBF;
 // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0xDF;
 // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= 0xF8;
  // setup the B register
// clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0xF7;
 // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= 0xF8;
  // setup the MUX Register
 // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= 0x7F;
// set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0x40;
  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0xDF;
 // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= 0xE0;
}
unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0xE0;

  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= 0xF7;
 
  // set the channel selection bits for channel 0
  *my_ADMUX |= adc_channel_num;

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;

  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)

  *my_ADCSRA |= 0x10;
  unsigned int val = (*my_ADC_DATA & 0x03FF);
  return val;
}

void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 UCSR0A = 0x20;
 UCSR0B = 0x18;
 UCSR0C = 0x06;
 UBRR0  = tbaud;
}

unsigned char U0kbhit()
{
  return UCSR0A & RDA;
}

unsigned char U0getchar()
{
  return UDR0;
}

void U0putchar(unsigned char U0pdata)
{
  while((UCSR0A & TBE)==0);
  UDR0 = U0pdata;
}

