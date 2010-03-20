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

#define __led_pin 4

#define __twi_slave_address 0x10 // we listen to this address
#define __RED 0x01
#define __GREEN 0x02
#define __BLUE 0x03

#define __TIMER1_MAX 0xFFFF // 16 bit CTR
#define __TIMER1_CNT 0x0130 // 32 levels --> 0x0130; 38 --> 0x0157 (flicker)
#include <avr/interrupt.h>   
#include <avr/io.h>
#include <Wire.h>

// NEEDS FASTER I2C !!!!!


byte brightness_red[__leds_per_row][__rows]; 
byte brightness_green[__leds_per_row][__rows];
byte brightness_blue[__leds_per_row][__rows]; 


void setup(void) {
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
  Wire.begin(__twi_slave_address); // join the bus with this address
  Wire.onReceive(receiveEvent); // register event
}


void loop(void) {
  // nothing
}


void receiveEvent(int dummy) {
  byte rx_buffer[4];
  byte counter = 0;
  if( Wire.available() ) {
    rx_buffer[counter] = Wire.receive();
    counter++;
    if( counter > 3) {
      counter = 0;
      switch( rx_buffer[0] ) {
        case 0x01:
          set_led_red(rx_buffer[1],rx_buffer[2],rx_buffer[3]);  
        break;
        case 0x02:
          set_led_green(rx_buffer[1],rx_buffer[2],rx_buffer[3]);
        break;
        case 0x03:
          set_led_blue(rx_buffer[1],rx_buffer[2],rx_buffer[3]);
        break;
        default:
        break;
      }
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
  if( led > __max_led ) { return; } // led out of bounds
  if( row > __max_row ) { return; } // row out of bounds
  brightness_red[row][led] = red; // row still on local board
}


void set_led_green(byte row, byte led, byte green) {
  if( led > __max_led ) { return; }
  if( row > __max_row ) { return; }
  brightness_green[row][led] = green;
}


void set_led_blue(byte row, byte led, byte blue) {
  if( led > __max_led ) { return; }
  if( row > __max_row ) { return; }
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


