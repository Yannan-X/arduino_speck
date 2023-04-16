/**
  *************************************************************************************************
  * @file           step
  * @author         张文超
  * @version        V2.0
  * @note    ´      步进电机基础例程
  * @note    ´      深圳市沁和智能科技有限公司，版权所有，严禁私自传播，否则必究法律责任
  *************************************************************************************************
  */

//ms1   ms2    ms3
//L       L        L       整步(没有细分)
//H      L         L      1/2(2细分)
//L      H        L       1/4(4细分)
//H      H        L       1/8(8细分)
//H      H        H      1/16(16细分)
  
#include <avr/interrupt.h>
#include "PinChangeInt.h"
#include "OLED12864.h"

#define MICRO 16
#define REDUCE 1
#define STEP_ANGLE 200

#define GET_TARGET   TARGET_STEP = MICRO*REDUCE*STEP_ANGLE*ANGLE/360;


// OLED12864 oled12864;

int ADD_PIN = A0;
int SUB_PIN = A1;
int SET_PIN = A2;

int EN_PIN = 8;    //使能引脚
int DIR_PIN = 5;   //方向引脚
int STEP_PIN = 2;  //脉冲引脚

int flag_dir = 0; //正转
int flag =0;
int flag_chang = 0;
double ANGLE = 0;

unsigned long STEP = 0;
unsigned long TARGET_STEP = 0;

unsigned char tcnt2 = 255;

void set_key_deal(){
  digitalWrite(DIR_PIN, LOW);
  TIMSK2 |= (1<<TOIE2);
  flag=1;
}

void add_key_deal(){
    flag_chang = 1;
    if(ANGLE < 360){ ANGLE = ANGLE + 10; }
    
    Serial.println("ADD KEY");
}

void sub_key_deal(){
  digitalWrite(DIR_PIN, HIGH);
  TIMSK2 |= (1<<TOIE2);
  flag=1;
}

ISR(TIMER2_OVF_vect) {
  TCNT2 = tcnt2;
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(31);
  digitalWrite(STEP_PIN, LOW);
  // delayMicroseconds(10);

  if(++STEP>= 1700&&flag==1){
  
      TIMSK2 &= ~(1<<TOIE2);
      STEP=0;
  }

  if(++STEP>= 1700&&flag==0){
  
      TIMSK2 &= ~(1<<TOIE2);
      STEP=0;
  }  

}

void setup() {
  Serial.begin(115200);
  
  // oled12864.init();  // initialize with the I2C addr 0x3D (for the 128x64)
  // oled12864.clear();

  pinMode( STEP_PIN,  OUTPUT ); 
  pinMode( DIR_PIN,  OUTPUT );
  pinMode( EN_PIN,  OUTPUT );
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(EN_PIN, LOW);

  attachPinChangeInterrupt( ADD_PIN , add_key_deal, RISING );
  attachPinChangeInterrupt( SUB_PIN , sub_key_deal, RISING );
  attachPinChangeInterrupt( SET_PIN , set_key_deal, RISING );

  TIMSK2 &= ~(1<<TOIE2);
  TCCR2A &= ~((1<<WGM21) | (1<<WGM20));   //模式选择，正常模式
  TCCR2B &= ~(1<<WGM22);
  ASSR &= ~(1<<AS2);     //禁止异步中断触发
  TIMSK2 &= ~(1<<OCIE2A);     //中断允许标志位

  TCCR2B |= ( (1<<CS22) );                //分频控制--64分频
  TCCR2B &= ~( (1<<CS21)| (1<<CS20) );

  //tcnt2 = 256 - (int)((float)F_CPU * 0.001 / 64);   //F_CPU = 16000000

  TCNT2 = tcnt2;     //初值
  //TIMSK2 |= (1<<TOIE2);  //溢出中断允许标志位
  TIMSK2 &= ~(1<<TOIE2);   //关闭定时器
  Serial.println("Starting......");

  // oled12864.show(0,0,"SPEED:");
  // oled12864.show(0,6,tcnt2);
  // oled12864.show(2,0,"MICRO:");
  // oled12864.show(2,6,MICRO);
  // oled12864.show(3,0,"ANGLE:");
  // oled12864.show(3,6,(int)ANGLE);
  // oled12864.display(); 
}

void loop() {
  // Serial.println(flag);

}
