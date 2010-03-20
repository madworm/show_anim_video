#define __spi_clock 13   // SCK - hardware SPI
#define __spi_latch 10
#define __spi_data 11    // MOSI - hardware SPI
#define __spi_data_in 12 // MISO - hardware SPI (unused)
#define __display_enable 9
#define __rows 8
#define __max_row __rows-1
#define __leds_per_row 8
#define __max_led __leds_per_row-1
#define __brightness_levels 28 // 0...15 above 32 is bad for ISR
#define __max_brightness __brightness_levels-1
#define __fade_delay 4
#define __fade_delay_micros 400  // in microseconds
#define __fade_out_display 2000 // in milliseconds

#define __LED_rx_buffer_length 43
#define __LED_rx_data_length 40
#define __LED_start_byte 0x0A
#define __LED_stop_byte 0x0D

#define __led_pin 4
#define __blink_delay 5

#define __image_delay 60

#define __TIMER1_MAX 0xFFFF // 16 bit CTR
#define __TIMER1_CNT 0x0130 // 32 levels --> 0x0130; 38 --> 0x0157 (flicker)
#include <avr/interrupt.h>   
#include <avr/io.h>
#include <avr/pgmspace.h>

byte brightness_red[__leds_per_row][__rows]; 
byte brightness_green[__leds_per_row][__rows];
byte brightness_blue[__leds_per_row][__rows]; 


/*
images in PROGMEM
*/

const char frame1[193] PROGMEM = {31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,0,0,0,0,0,0,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,0,0,0,0,0,0,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31};

const char frame2[193] PROGMEM = {24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,31,0,0,31,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,31,0,0,31,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24};

const char frame3[193] PROGMEM = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,31,31,31,31,31,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,31,31,31,31,31,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

const char frame4[193] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,16,0,0,16,0,0,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,16,0,0,16,0,0,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

const char frame5[193] PROGMEM = {31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,16,0,0,16,0,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,0,16,0,0,16,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,0,16,0,0,16,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,16,0,0,16,0,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0};

const char frame6[193] PROGMEM = {31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,0,0,0,0,0,0,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,0,0,0,0,0,0,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,31,31,31,31,31,31,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31};

const char frame7[193] PROGMEM = {24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,31,0,0,31,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,31,0,0,31,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,24,24,24,24,24,24,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24};

const char frame8[193] PROGMEM = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,31,31,31,31,31,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,31,31,31,31,31,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,8,8,8,8,8,8,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};

const char frame9[193] PROGMEM = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,16,0,0,16,0,0,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,16,0,0,16,0,0,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,0,0,0,0,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

const char frame10[193] PROGMEM = {31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,16,0,0,16,0,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,0,16,0,0,16,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,0,16,0,0,16,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,16,0,0,16,0,0,16,0,0,16,0,0,31,31,31,31,0,0,31,0,0,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0,31,0,0};

const char * frames[] = {frame1,frame2,frame3,frame4,frame5,frame6,frame7,frame8,frame9,frame10};

void setup(void) {
  Serial.begin(57600);
  pinMode(__spi_clock,OUTPUT);
  pinMode(__spi_latch,OUTPUT);
  pinMode(__spi_data,OUTPUT);
  pinMode(__spi_data_in,INPUT);
  pinMode(__display_enable,OUTPUT);
  digitalWrite(__spi_latch,HIGH);
  digitalWrite(__spi_data,HIGH);
  digitalWrite(__spi_clock,HIGH);
  setup_hardware_spi();
  setup_timer1_ovf();
  set_matrix_rgb(255,255,255);
  pinMode(__led_pin,OUTPUT);
}


void loop(void) {
  /* this would show the images stored in the above PROGMEM arrays
  byte counter;
  for (counter=0; counter<10; counter++) {
    show_image(counter,100);
  }
  */

  /* receive image data over serial */
  serial_handler();	
}


void show_image(byte image, int img_delay) {
  byte row;
  byte led;
  for ( row = 0; row < 8; row++) {
    for ( led = 0; led < 8; led++ ) {
      set_led_red(row,led,pgm_read_byte(    &frames[image][row*3*8+led*3+0] ));
      set_led_green(row,led,pgm_read_byte(  &frames[image][row*3*8+led*3+1] ));
      set_led_blue(row,led,pgm_read_byte(   &frames[image][row*3*8+led*3+2] ));
    }
  }
  delay(img_delay);
}


void serial_handler(void) {
  static unsigned long timer = 0;
  static byte LED_rx_buffer[__LED_rx_buffer_length];
  static byte LED_rx_ctr = 0;
  static byte LED_data = 0;
  
  if ( Serial.available() ) {
    LED_data = Serial.read();
    if ( LED_data == __LED_start_byte ) {
      //blink_led();
      while (LED_rx_ctr <= __LED_rx_data_length) {
        if ( Serial.available() ) {
          LED_rx_buffer[LED_rx_ctr] = Serial.read();
          LED_rx_ctr++;
        }
      }
      LED_rx_ctr = 0;
      if ( LED_rx_buffer[__LED_rx_data_length] == __LED_stop_byte) {
        //blink_led();
	byte counter;
        for ( counter = 0; counter < 8; counter++ ) {
          set_led_red(LED_rx_buffer[counter*5+0],LED_rx_buffer[counter*5+1],LED_rx_buffer[counter*5+2]);
	  set_led_green(LED_rx_buffer[counter*5+0],LED_rx_buffer[counter*5+1],LED_rx_buffer[counter*5+3]);
	  set_led_blue(LED_rx_buffer[counter*5+0],LED_rx_buffer[counter*5+1],LED_rx_buffer[counter*5+4]);
        }
	clear_buffer(LED_rx_buffer,__LED_rx_buffer_length);
        timer = millis(); 
      }
    }
  }
  
  if ( (millis() - timer) > __fade_out_display ) {
    fade_out_display();
  }
}


void clear_buffer(byte * buffer, byte length) {
  byte ctr;
  for(ctr = 0; ctr <= (length-1); ctr++) {
    buffer[ctr] = 0;
  }
}


void blink_led() {
  delay(__blink_delay);
  digitalWrite(__led_pin,HIGH);
  delay(__blink_delay);
  digitalWrite(__led_pin,LOW); 
}


void fade_out_display(void) {
  byte ctr1;
  byte ctr2;
  
  for(ctr2 = 0; ctr2 <= __max_row; ctr2++) {
    for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      if(brightness_red[ctr1][ctr2] > 0) {brightness_red[ctr1][ctr2]--;}
      if(brightness_green[ctr1][ctr2] > 0) {brightness_green[ctr1][ctr2]--;}
      if(brightness_blue[ctr1][ctr2] > 0)  {brightness_blue[ctr1][ctr2]--;}
      delayMicroseconds(__fade_delay_micros);
    }
  }
}


byte spi_transfer(byte data)
{
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1<<SPIF)))     // Wait the end of the transmission
  {
  };
  return SPDR;                    // return the received byte, we don't need that
}


void set_led_red(byte row, byte led, byte red) {
  if( (row > __max_row) || (led > __max_led) ) { return; }
  brightness_red[row][led] = red;
}


void set_led_green(byte row, byte led, byte green) {
  if( (row > __max_row) || (led > __max_led) ) { return; }
  brightness_green[row][led] = green;
}


void set_led_blue(byte row, byte led, byte blue) {
  if( (row > __max_row) || (led > __max_led) ) { return; }
  brightness_blue[row][led] = blue;
}


void set_led_rgb(byte row, byte led, byte red, byte green, byte blue) {
  set_led_red(row,led,red);
  set_led_green(row,led,green);
  set_led_blue(row,led,blue);
}


void set_matrix_rgb(byte red, byte green, byte blue) {
  byte ctr1;
  byte ctr2;
  for(ctr2 = 0; ctr2 <= __max_row; ctr2++) {
    for(ctr1 = 0; ctr1 <= __max_led; ctr1++) {
      set_led_rgb(ctr2,ctr1,red,green,blue);
    }
  }
}


void setup_hardware_spi(void) {
  byte clr;
  // spi prescaler: 
  // SPI2X SPR1 SPR0
  //   0     0     0    fosc/4
  //   0     0     1    fosc/16
  //   0     1     0    fosc/64
  //   0     1     1    fosc/128
  //   1     0     0    fosc/2
  //   1     0     1    fosc/8
  //   1     1     0    fosc/32
  //   1     1     1    fosc/64
  SPCR |= ( (1<<SPE) | (1<<MSTR) ); // enable SPI as master
  //SPCR |= ( (1<<SPR1) ); // set prescaler bits
  SPCR &= ~ ( (1<<SPR1) | (1<<SPR0) ); // clear prescaler bits
  clr=SPSR; // clear SPI status reg
  clr=SPDR; // clear SPI data reg
  //SPSR |= (1<<SPI2X); // set prescaler bits
  SPSR &= ~(1<<SPI2X); // clear prescaler bits
}

void setup_timer1_ovf(void) {
  // Arduino runs at 16 Mhz...
  // Timer1 (16bit) Settings:
  // prescaler (frequency divider) values:   CS12    CS11   CS10
  //                                           0       0      0    stopped
  //                                           0       0      1      /1  
  //                                           0       1      0      /8  
  //                                           0       1      1      /64
  //                                           1       0      0      /256 
  //                                           1       0      1      /1024
  //                                           1       1      0      external clock on T1 pin, falling edge
  //                                           1       1      1      external clock on T1 pin, rising edge
  //
  TCCR1B &= ~ ( (1<<CS11) );
  TCCR1B |= ( (1<<CS12) | (1<<CS10) );      
  //normal mode
  TCCR1B &= ~ ( (1<<WGM13) | (1<<WGM12) );
  TCCR1A &= ~ ( (1<<WGM11) | (1<<WGM10) );
  //Timer1 Overflow Interrupt Enable  
  TIMSK1 |= (1<<TOIE1);
  TCNT1 = __TIMER1_MAX - __TIMER1_CNT;
}


ISR(TIMER1_OVF_vect) {
  sei();
  TCNT1 = __TIMER1_MAX - __TIMER1_CNT;
  byte cycle;
  
  digitalWrite(__display_enable,LOW); // enable display inside ISR
  
  for(cycle = 0; cycle < __max_brightness; cycle++) {
    byte led;
    byte row = B00000000;
    byte red;   
    byte green; 
    byte blue;  

    for(row = 0; row <= __max_row; row++) {
      
      red = 0xFF; 
      green = 0xFF;
      blue = 0xFF;
      
      for(led = 0; led <= __max_led; led++) {
        if(cycle < brightness_red[row][led]) {
          red &= ~(1<<led);
        }
        if(cycle < brightness_green[row][led]) {
          green &= ~(1<<led);
        }
        if(cycle < brightness_blue[row][led]) {
          blue &= ~(1<<led);
        }
      }

      digitalWrite(__spi_latch,LOW);
      spi_transfer(B00000001<<row);
      spi_transfer(blue);
      spi_transfer(green);
      spi_transfer(red);
      digitalWrite(__spi_latch,HIGH);
    }
  }
  digitalWrite(__display_enable,HIGH);    // disable display outside ISR
}


