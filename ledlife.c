#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/twi.h>
#include <stdio.h>

#include "ledlife.h"
#include "life-unroll.h"

/* serial output */
/* boardunio: digital 11 */
#define MOSI_PINB PB3

/* serial clock */
/* boarduino: digital 13 */
#define SCK_PINB PB5

/* slave select, needs to be output but is otherwise just ued for
 * debugging */
/* boarduino: digital 10 */
#define SS_PINB PB2

/* pull this up for the left half of the display, which will be the
 * TWI master, and down for the right half of the display, which will
 * be the TWI slave. */
/* boarduino: digital 6 */
#define SCREEN_HALF_SELECT_PIND PD6

/* pin on PORTB that's hooked up to the latch output from the buffer board */
/* boarduino: digital 9 */
#define LATCH_IN_PINB PB1

/* our output latch port to the daughterboard */
/* boarduino: digital 8 */
#define LATCH_OUT_PINB PB0

/* pin on portd that's hooked up to one of the power pins so we can
 * synchronize which power pin we're using */
/* boarduino: digital 7 */
#define POWER_PIND PD7

/* output pin that's 1 when we are on our first power output */
/* boarduino: digital 6 */
#define DEBUG_FIRST_POWER_PIND PD6

/* pin pair that's connected to xbee for communication (in parallel
 * with the other controller) */
/* boarduino: TXD */
#define XBEE_TX_PIND PD1
/* boarduino: RXD */
#define XBEE_RX_PIND PD0

/* TWI pins that are connected to the same pins on the other
 * controller */
/* boarduino: analog 5 */
#define TWI_SCL_PINC PC5
/* boarduino: analog 4 */
#define TWI_SDA_PINC PC4

// address we use to talk on the TWI bus
#define TWI_SLAVE_ADDR 0xF0

// debugging options
#undef DEBUG_TWI_DATA

/* for swapping between MSB and LSB: */
uint8_t backward_bytes[256] __attribute__((section(".backward_bytes")))  = {
#include "backward_bytes.h"
};

unsigned char data_buf[(192/8)* 16 * 2];
unsigned char *show_data_buf = &data_buf[(192/8)*16*0];
unsigned char *compute_data_buf = &data_buf[(192/8)*16*1];
unsigned char **serial_write_buf = &compute_data_buf;

enum run_state {
  LIFE_RANDOM,
  LIFE_EXPLICIT,
  TEST_PATTERN,
  HOLD_SINGLE,
  HOLD_DOUBLE
};

volatile enum run_state current_run_state = LIFE_RANDOM;

enum attention_cmd {
  ADDRESS_1 = 0,
  ADDRESS_2 = 1,
  ADDRESS_BOTH = 2,
  QUOTE_NEXT = 3
};

struct serial_state {
  // if 0, ignore all commands and data:
  unsigned addressing_us:1;

  // if 1, quoting next character so don't let it be an attention:
  unsigned quoting:1;

  // if 1, we're processing a command.  otherwise we're uploading data.
  unsigned processing_command:1;

  // if 1, we're processing an argument to a command.  otherwise we're uploading data.
  unsigned processing_command_arg:1;
};

volatile struct serial_state current_serial_state;

enum command {
  // change our mode
  SET_RANDOM_LIFE_STATE = 64,
  SET_EXPLICIT_LIFE_STATE = 65,
  SET_TEST_PATTERN_STATE = 66,
  SET_HOLD_SINGLE_MODE = 67,
  SET_HOLD_DOUBLE_MODE = 68,
  
  // wipe the screen
  CLEAR = 70,
  
  // set cursor X position (128=0 to 135=7)
  SET_CURSOR_X = 71,
  // set cursor y position (128=0 to 175=47)
  SET_CURSOR_Y = 72,

  // swap the output and compute buffer
  SWAP_BUFFER = 73,

  // perform one round of lie
  LIFE_ONE_ROUND = 74
};

extern inline unsigned char twi_master(void)
{
  return PIND & _BV(SCREEN_HALF_SELECT_PIND);
}

static int uart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL,
                                         _FDEV_SETUP_WRITE);

static int
uart_putchar(char c, FILE *stream)
{
  if (UCSR0B & _BV(TXEN0)) {
    while (!(UCSR0A & _BV(UDRE0)))
      ;
    UDR0 = c;
  }
  return 0;
}

volatile enum command current_command;


/* the bits are arranged in a 64 x 48 grid.  The bytes are as following:
 * (<- = MSB first, -> = LSB first)
 *
 * top third (starting at row 0)
 *  0<-  1<-  4<-  5<-  8<-  9<- 12<- 13<-
 * 24<- 25<- 28<- 29<- 32<- 33<- 36<- 37<-
 * ...
 *
 * middle third (starting at row 16)
 *  2->  3->  6->  7-> 10-> 11-> 14-> 15->
 * 26-> 27-> 30-> 31-> 34-> 35-> 38-> 39->
 * ...
 *
 * bottom third (starting at row 32)
 * 16<- 17<- 19-> 18-> 20<- 21<- 23-> 22->
 * 40<- 41<- 43-> 42-> 44<- 45<- 47-> 47->
 * ...
 */

uint16_t EEMEM randSeed1;
uint16_t EEMEM randSeed2;

extern inline void set_compute_bit(unsigned char x, unsigned char y)
{
  compute_data_buf[(x >> 3) + ((unsigned short)y)*8] |= _BV(x & 7);
}

extern inline void clear_compute_bit(unsigned char x, unsigned char y)
{
  compute_data_buf[(x >> 3) + ((unsigned short)y)*8] &= ~_BV(x & 7);
}

extern inline void toggle_bit(unsigned char x, unsigned char y)
{
  compute_data_buf[(x >> 3) + ((unsigned short)y)*8] ^= _BV(x & 7);
}

extern inline void toggle_show_bit(unsigned char x, unsigned char y)
{
  show_data_buf[(x >> 3) + ((unsigned short)y)*8] ^= _BV(x & 7);
}

extern inline void clear_buf(unsigned char *buf)
{
  memset(buf, 0, (192/8) * 16);
}

extern inline unsigned char get_compute_bit(unsigned char x, unsigned char y)
{
  return compute_data_buf[(((unsigned short)y) << 3) | (x >> 3) ] & _BV(x & 7);
}

extern inline unsigned char get_show_bit(unsigned char x, unsigned char y)
{
  return show_data_buf[(((unsigned short)y) << 3) | (x >> 3) ] & _BV(x & 7);
}

static void swapbuf(void)
{
  unsigned char *tmp;
  cli();
  tmp = compute_data_buf;
  compute_data_buf = show_data_buf;
  show_data_buf = tmp;
  sei();
}

void twi_master_fail(unsigned char where) {
  printf_P(PSTR("MF:%d\n"), where);

  TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
  return;
}

void twi_slave_fail(unsigned char where) {
  printf_P(PSTR("SF:%d\n"), where);
  
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO) | _BV(TWEA);
  return;
}

extern inline void twi_wait(void)
{
  // wait for signal from twi
  while (!(TWCR & _BV(TWINT)))
    ;
}

extern inline char twi_wait_timeout(int ms)
{
  // wait for signal from twi
  while (!(TWCR & _BV(TWINT))) {
    if (ms-- == 0) {
      return 0;
    }
    _delay_ms(1);
  }
  return 1;
}

extern inline unsigned char twi_status(void)
{
  unsigned char st = TWSR;
  printf_P(PSTR(":%x "), st);
  return (st & 0xf8);
}

struct twi_buf {
  unsigned char pass_left[48 / 8];
  unsigned char pass_right[48 / 8];
} twi_in, twi_out;

static void twi_exchange(void)
{
  unsigned char pos;
  unsigned char len = sizeof(struct twi_buf);
  unsigned char *out_buf = (unsigned char*) &twi_out;
  unsigned char *in_buf = (unsigned char*) &twi_in;

  if (twi_master()) {
    putchar('a');
    // send "start" signal
    TWCR = _BV(TWINT)|_BV(TWSTA)|_BV(TWEN);
    twi_wait();
    putchar('b');
    if (twi_status()!=TW_START)
      return twi_master_fail(1);
    
    // Send slave address
    TWDR = TWI_SLAVE_ADDR | TW_WRITE;
    TWCR = _BV(TWINT)|_BV(TWEN);
    twi_wait();
    putchar('c');
    // Check slave address complete
    if (twi_status()!=TW_MT_SLA_ACK)
      return twi_master_fail(2);

    for (pos = 0; pos < len; pos++) {
      // transmit a byte of data
      TWDR = out_buf[pos];
      TWCR = _BV(TWINT) | _BV(TWEN);
      twi_wait();
      if (twi_status() != TW_MT_DATA_ACK)
	return twi_master_fail(3);
    }

    // restart, in read mode
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    twi_wait();
    putchar('d');
    
    if (twi_status()!=TW_REP_START)
      return twi_master_fail(4);
    
    // Send slave address, with read bit set
    TWDR = TWI_SLAVE_ADDR | TW_READ;
    TWCR = _BV(TWINT)|_BV(TWEN);
    
    // Wait for SLA_R, TWINT=1
    twi_wait();
    putchar('e');

    // Check slave address complete
    if (twi_status()!=TW_MR_SLA_ACK)
      return twi_master_fail(5);

    for (pos = 0; pos < len; pos++) {
      // receive a byte
      TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
      twi_wait();
      putchar('x');
      switch (twi_status()) {
      case TW_MR_DATA_ACK:
      case TW_MR_DATA_NACK:
	break;
      default:
	return twi_master_fail(6);
      }
      
      in_buf[pos] = TWDR;
    }

    // return to idle state
    TWCR = _BV(TWSTO)|_BV(TWEN)|_BV(TWINT);
    putchar('!');
    putchar('\n');
    return;
  } else {
    putchar('a');
    // Wait for us to be addressed
    if (!twi_wait_timeout(1000)) {
      printf_P(PSTR("timeout\n"));
      return;
    }

    putchar('b');
    // Check to make sure we got someone talking to us
    if (twi_status()!=TW_SR_SLA_ACK)
      return twi_slave_fail(1);

    // acknowledge that we're being addressed
    TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWINT);

    for (pos = 0; pos < len; pos++) {
      // wait for a byte
      twi_wait();
      putchar('c');
      in_buf[pos] = TWDR;
      switch (twi_status()) {
      case TW_SR_DATA_ACK:
      case TW_SR_DATA_NACK:
	break;
      default:
	return twi_slave_fail(2);
      }
      // ack it, unless it's the last byte, then nack it:
      if (pos == (len - 1))
	TWCR = _BV(TWEN) | _BV(TWINT);
      else
	TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWINT);
    }

    twi_wait();
    putchar('d');
    // the other end should have stopped, and then transmitted a restart
    if (twi_status() != TW_SR_STOP)
      return twi_slave_fail(3);

    // acknowledge that we're stopped, and start listening for our address again:
    TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWINT);
    twi_wait();
    putchar('e');

    // the other end should have addressed us.
    if (twi_status() != TW_ST_SLA_ACK)
      return twi_slave_fail(4);

    putchar('i');
    for (pos = 0; pos < len; pos++) {
      // transmit bte
      TWDR = out_buf[pos];
      if (pos == (len - 1)) {
	// last byte; expect nack
	TWCR = _BV(TWEN) | _BV(TWINT);
	twi_wait();
	putchar('l');
	if (twi_status() != TW_ST_LAST_DATA)
	  return twi_slave_fail(5);
      } else {
	TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWINT);
	twi_wait();
	if (twi_status() != TW_ST_DATA_ACK)
	  return twi_slave_fail(6);
      }
    }
    putchar('h');
    putchar('\n');

    // return to regular listening mode
    TWCR = _BV(TWEA)|_BV(TWEN)|_BV(TWINT);
    return;
  }
}

unsigned short serial_cursor; // asm("r10");

void process_attention(unsigned char b)
{
  switch(b) {
  case ADDRESS_1:
  case ADDRESS_2:
    if (twi_master() ? (b != ADDRESS_1) : (b != ADDRESS_2)) {
      current_serial_state.addressing_us = 0;
      current_serial_state.processing_command = 0;
      break;
    }
  case ADDRESS_BOTH:
    // TODO: figure out how to detect which one we're on
    current_serial_state.addressing_us = 1;
    current_serial_state.processing_command = 1;
    break;
  case QUOTE_NEXT:
    current_serial_state.quoting = 1;
    break;
  }
}

void init_life(void)
{
  unsigned char i;
  for (i = 0; i < (192/8)*16/4; i++) {
    ((unsigned long *)compute_data_buf)[i] = random();
  }
}

void process_command(unsigned char b)
{
  current_serial_state.processing_command = 0;
  if (current_serial_state.processing_command_arg == 0) {
    switch(b) {
    case SET_RANDOM_LIFE_STATE:
      current_run_state = LIFE_RANDOM;
      init_life();
      break;
    case SET_EXPLICIT_LIFE_STATE:
      current_run_state = LIFE_EXPLICIT;
      break;
    case SET_TEST_PATTERN_STATE:
      current_run_state = TEST_PATTERN;
      break;
    case SET_HOLD_SINGLE_MODE:
      current_run_state = HOLD_SINGLE;
      serial_write_buf = (&show_data_buf);
      break;
    case SET_HOLD_DOUBLE_MODE:
      current_run_state = HOLD_DOUBLE;
      serial_write_buf = (&compute_data_buf);
      break;
    case CLEAR:
      clear_buf(*serial_write_buf);
      serial_cursor = 0;
      break;
    case SET_CURSOR_X:
    case SET_CURSOR_Y:
      current_serial_state.processing_command = 1;
      current_serial_state.processing_command_arg = 1;
      current_command = b;
      break;
    case SWAP_BUFFER:
      swapbuf();
      break;
    case LIFE_ONE_ROUND:
      clear_buf(compute_data_buf);
      life_round();
      swapbuf();
      break;
    default: // do nothing
      break;
    }    
  } else {
    current_serial_state.processing_command_arg = 0;
    switch(current_command) {
    case SET_CURSOR_Y:
      serial_cursor = (((unsigned short)(b - 128)) << 3) | (serial_cursor & 7);
      break;
    case SET_CURSOR_X:
      serial_cursor = (b - 128) + (serial_cursor & ~7);
      break;
    default: // do nothing
      break;
    }
  }
}

void process_serial(void)
{
  unsigned char b;

  b = UDR0;
  if (current_serial_state.quoting == 0) {
    if (b < 4) {
      // attention command
      process_attention(b);
      return;
    }
  } else
    current_serial_state.quoting = 0;
  if (current_serial_state.addressing_us == 0) return;
  if (current_serial_state.processing_command)
    process_command(b);
  else {
    if (serial_cursor >= 384)
      serial_cursor = 0;
    (*serial_write_buf)[serial_cursor++] = b;
  }
}

extern inline void check_serial(void)
{
  if (UCSR0A & _BV(RXC0)) {
    process_serial();
  }
}

ISR(USART_RX_vect)
{
  process_serial();
}

unsigned char get_life_bit(signed char x, signed char y)
{
  if (y > 47)
    y = 0;
  else if (y < 0)
    y = 47;
  
  if (x > 63)
    return twi_in.pass_left[y >> 3] & _BV(y & 7);
  if (x < 0)
    return twi_in.pass_right[y >> 3] & _BV(y & 7);

  return get_show_bit(x,y);
}

extern inline unsigned char get_bit(unsigned char *data, unsigned short bit)
{
  return data[bit >> 3] & _BV(bit & 7);
}

extern inline void set_bit(unsigned char *data, unsigned short bit)
{
  data[bit >> 3] |= _BV(bit & 7);
}

void life_round(void)
{
  unsigned char y;
  unsigned char *prevline;
  unsigned char *thisline;
  unsigned char *nextline;
  unsigned char *thisline_out;
  unsigned char count_pass_left, count_pass_right;

  memset(&twi_out, 0, sizeof(twi_out));

  for (y = 0; y < 48; y++)
    if (get_show_bit(0, y))
      set_bit(twi_out.pass_left, y);

  for (y = 0; y < 48; y++)
    if (get_show_bit(63, y))
      set_bit(twi_out.pass_right, y);

  // memcpy(&twi_in.pass_left, &twi_out.pass_left, sizeof(twi_in.pass_left));
  // memcpy(&twi_in.pass_right, &twi_out.pass_right, sizeof(twi_in.pass_right));

  twi_exchange();

#ifdef DEBUG_TWI_DATA
  {
    unsigned char b;
    printf_P(PSTR("pass left out: "));
    for (b = 0; b < (48/8); b++) {
      printf_P(PSTR("%02x"), twi_out.pass_left[b]);
    }
    putchar('\n');
    printf_P(PSTR("pass right out:"));
    for (b = 0; b < (48/8); b++) {
      printf_P(PSTR("%02x"), twi_out.pass_right[b]);
    }
    putchar('\n');

    printf_P(PSTR("pass left in : "));
    for (b = 0; b < (48/8); b++) {
      printf_P(PSTR("%02x"), twi_in.pass_left[b]);
    }
    putchar('\n');

    printf_P(PSTR("pass right  in:"));
    for (b = 0; b < (48/8); b++) {
      printf_P(PSTR("%02x"), twi_in.pass_right[b]);
    }
    putchar('\n');
  }
#endif
  
  for (y = 0; y < 48; y++) {
    count_pass_left = 0; count_pass_right = 0;
    if (y == 0) {
      if (get_bit(twi_in.pass_left, 47))
	count_pass_left++;
      if (get_bit(twi_in.pass_right, 47))
	count_pass_right++;
      prevline = &show_data_buf[47 * 8];
    } else {
      if (get_bit(twi_in.pass_left, y-1))
	count_pass_left++;
      if (get_bit(twi_in.pass_right, y-1))
	count_pass_right++;
      prevline = &show_data_buf[(y - 1) * 8];
    }

    thisline = &show_data_buf[y * 8];
    thisline_out = &compute_data_buf[y * 8];

    if (get_bit(twi_in.pass_left, y))
      count_pass_left++;
    if (get_bit(twi_in.pass_right, y))
      count_pass_right++;

    if (y == 47) {
      if (get_bit(twi_in.pass_left, 0))
	count_pass_left++;
      if (get_bit(twi_in.pass_right, 0))
	count_pass_right++;
      nextline = &show_data_buf[0 * 8];
    } else {
      if (get_bit(twi_in.pass_left, y+1))
	count_pass_left++;
      if (get_bit(twi_in.pass_right, y+1))
	count_pass_right++;
      nextline = &show_data_buf[(y+1) * 8];
    }

    life_round_line(prevline, thisline, nextline,
		    count_pass_right, count_pass_left,
		    thisline_out);
  }
}

void ioinit(void)
{
  cli();

  // set output pins for output
  DDRB = _BV(MOSI_PINB) | _BV(SCK_PINB) | _BV(SS_PINB) | _BV(LATCH_OUT_PINB);

  PRR = 0;

  // enable internal pull-up on power pin so it doesn't cause issues if unconnected
  // PORTD = _BV(POWER_PIND);

  // debugging output
  DDRD = _BV(DEBUG_FIRST_POWER_PIND);

  // enable PCI0 interrupt interrupt when latch pin changes
  PCMSK0 = _BV(PCINT1);
  // enable PCI2 interrupt when power pin changes (PD7)
  PCMSK2 = _BV(PCINT23);
  // enable PCI0 and PCI2 interrupts
  PCICR = _BV(PCIE0) | _BV(PCIE2);

  // put MOSI up when we're not clocking out
  PORTB |= _BV(MOSI_PINB);
  
  // set up UART for 19200 baud at 16Mhz
  UCSR0B = 0;
#define BAUD 19200
#include <util/setbaud.h>
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A |= (1 << U2X0);
#else
  UCSR0A &= ~(1 << U2X0);
#endif
  UCSR0B = _BV(RXEN0) | _BV(RXCIE0);
  stdout = &mystdout;
  if (twi_master()) {
    // enable debugging for only one at a time
    DDRD |= _BV(XBEE_TX_PIND);
    UCSR0B |= _BV(TXEN0);
  }
  UCSR0C = 3 << UCSZ00; // 8 bits per byte

  TWBR = 15;
  if (twi_master()) {
    TWCR = _BV(TWEN);
  } else {
    // enable our TWI in slave mode - address 0xF
    TWAR = TWI_SLAVE_ADDR;
    TWAMR = 0;
    TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWINT);
  }

  sei ();
}

static void spistart(void)
{
  // start SPI in master mode, at SPI clock
  // running at CPU frequency / 8
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0) | _BV(DORD); // | _BV(CPOL) | _BV(CPHA)
  SPSR = _BV(SPI2X);

  PORTB &= ~_BV(SS_PINB); // lower slave select (for debugging)
}

static void spistop(void)
{
  SPCR = 0;
  PORTB |= _BV(SS_PINB); // raise slave select (for debugging)
}

unsigned volatile char can_write_out;
unsigned volatile short frame_counter;

unsigned char pow_select; // 0..15 depending on sequence of latching
unsigned char pow_inverse;
unsigned char display_count;
unsigned char inverse;
unsigned char hilite_byte;

#define wait_and_clock_lsb_first(region,pos) do { \
    /* SPIF drops low when the SPI output is finished, so we want to start \
     * the next byte as soon as possible. \
     * \
     * This requires any work that needs to be done to gather the value to \
     * write (such as loading it in a register) to be done before we wait \
     * for SPIF to drop \
     * \
     * with msb first, we have to look up the bytes in our backwards table */ \
    asm volatile( \
	    "ldd r16, Y+(" #pos ")"            "\n\t" \
	    ".spi_not_ready_%=:"                      "\n\t" \
	    "in __tmp_reg__,%[spsr]"               "\n\t" \
	    "sbrs __tmp_reg__, %[spif]"            "\n\t" \
	    "rjmp .spi_not_ready_%="                  "\n\t" \
	    "out %[spdr], r16"              "\n\t" \
	    : \
            : \
	    [spsr] "I" (_SFR_IO_ADDR(SPSR)), \
	    [spif] "I" (SPIF), \
	    [spdr] "I" (_SFR_IO_ADDR(SPDR)), \
            [start] "y" (region) \
            : \
            "r16" \
	    ); \
} while(0);

#define wait_and_clock_msb_first(region,pos) do { \
    /* SPIF drops low when the SPI output is finished, so we want to start \
     * the next byte as soon as possible. \
     * \
     * This requires any work that needs to be done to gather the value to \
     * write (such as loading it in a register) to be done before we wait \
     * for SPIF to drop \
     * \
     * with msb first, we have to look up the bytes in our backwards table */ \
    asm volatile( \
	    "ldd %A[backwards], Y+(" #pos ")"            "\n\t" \
            "lpm r16, %a[backwards]"                     "\n\t" \
	    ".spi_not_ready_%=:"                      "\n\t" \
	    "in __tmp_reg__,%[spsr]"               "\n\t" \
	    "sbrs __tmp_reg__, %[spif]"            "\n\t" \
	    "rjmp .spi_not_ready_%="                  "\n\t" \
	    "out %[spdr], r16"              "\n\t" \
	    : \
            [backwards] "+z" (backwards) \
            : \
	    [spsr] "I" (_SFR_IO_ADDR(SPSR)), \
	    [spif] "I" (SPIF), \
	    [spdr] "I" (_SFR_IO_ADDR(SPDR)), \
            [start] "y" (region) \
            : \
            "r16" \
	    ); \
} while(0);

// clock out serial data
static void writeout(void)
{
  unsigned char this_pow_select;
  unsigned char *region_1, *region_2, *region_3;
  unsigned char *backwards = backward_bytes;
  
  this_pow_select = pow_select;
  pow_select++;
  if (pow_select > 15)
    pow_select = 0;

  sei();

  if (this_pow_select == 0)
    PORTD |= _BV(DEBUG_FIRST_POWER_PIND);

  // turn off the latch to the daughterboards
  PORTB &= ~_BV(LATCH_OUT_PINB);

  // find our sections
  region_1 = show_data_buf + (8 * this_pow_select) + (8*16*0);
  region_2 = show_data_buf + (8 * this_pow_select) + (8*16*1);
  region_3 = show_data_buf + (8 * this_pow_select) + (8*16*2);

  check_serial();

  // and actually clock out
  spistart();

  // SPDR = 0b01010101;
  __asm__("/* starting clocking out */");
  SPDR =                   region_1 [0];
  wait_and_clock_lsb_first(region_1, 1);
  check_serial();
  wait_and_clock_msb_first(region_2, 1);
  wait_and_clock_msb_first(region_2, 0);
  check_serial();

  wait_and_clock_lsb_first(region_1, 2);
  wait_and_clock_lsb_first(region_1, 3);
  check_serial();
  wait_and_clock_msb_first(region_2, 3);
  wait_and_clock_msb_first(region_2, 2);
  check_serial();
  
  wait_and_clock_lsb_first(region_1, 4);
  wait_and_clock_lsb_first(region_1, 5);
  check_serial();
  wait_and_clock_msb_first(region_2, 5);
  wait_and_clock_msb_first(region_2, 4);
  check_serial();
  
  wait_and_clock_lsb_first(region_1, 6);
  wait_and_clock_lsb_first(region_1, 7);
  check_serial();
  wait_and_clock_msb_first(region_2, 7);
  wait_and_clock_msb_first(region_2, 6);
  check_serial();
  
  wait_and_clock_lsb_first(region_3, 0);
  wait_and_clock_lsb_first(region_3, 1);
  check_serial();
  wait_and_clock_msb_first(region_3, 3);
  wait_and_clock_msb_first(region_3, 2);
  check_serial();
  
  wait_and_clock_lsb_first(region_3, 4);
  wait_and_clock_lsb_first(region_3, 5);
  check_serial();
  wait_and_clock_msb_first(region_3, 7);
  wait_and_clock_msb_first(region_3, 6);
  __asm__("/* done clocking out */");
  check_serial();

  // wait for SPIF to be set indicating the end of the last byte finished
  while (!(SPSR & _BV(SPIF)));
  
  spistop();
  PORTD &= ~_BV(DEBUG_FIRST_POWER_PIND);

  frame_counter++;
}

ISR(PCINT0_vect) /* "PCI0" interrupts */
{
  // latch pin changed.  write out serial if we're not in the middle of it already
  if (!can_write_out) return;

  // output latch pin to activate the new values we shifted in last time:
  PORTB |= _BV(LATCH_OUT_PINB);
  // (it's turned off within "writeout")

  // mark writing out as busy, and then enable interrupts so we can go on with life
  can_write_out = 0;  

  // now, emit our data
  writeout();

  // we're done! re-enable writing out
  can_write_out = 1;
}

// PCINT23 toggled
ISR(PCINT2_vect)
{
  if (PIND & _BV(POWER_PIND)) {
    // low->high transition at the beginning of the power signal.
    // we get a latch spike at approximately this time, so we have to figure out whether
    // we got that or not in order to avoid the race condition:
    if (can_write_out == 0) {
      // we're busy writing out, hopefully this is frame 15.  reset to frame 0.
      pow_select = 0;
    } else {
      // haven't started to write out frame 0 yet; next frame is 0
      pow_select = 15;
    }
  }
}

void next_frame(void)
{
  unsigned char frame_counter2;
  // wait for 1/3rd of a second, assuming a 60 hz refresh rate:
  for (frame_counter2 = 0; frame_counter2 < 1 ; frame_counter2++) {
    while (frame_counter < 16*10) {
    }
    frame_counter = 0;
  }
}

void get_random_seed(void)
{
  srandom((((unsigned long)eeprom_read_word(&randSeed1)) << 16) + (eeprom_read_word(&randSeed2)));
}

void save_random_seed(void)
{
  unsigned long seed = random();
  eeprom_write_word(&randSeed1, (seed >> 16));
  eeprom_write_word(&randSeed2, (seed & 0xFFFF));
}

int main(void) 
{
  unsigned char rand_pos = 0;
  unsigned char diag = 0;
  can_write_out = 1;
  ioinit();

  get_random_seed();
  init_life();
  save_random_seed();

  for (;;) {
    switch (current_run_state) {
    case LIFE_RANDOM:
      swapbuf();
      rand_pos += random();
      clear_buf(compute_data_buf);
      if (rand_pos == 0) init_life(); // reinit every 256 rounds
      life_round();
      break;
    case LIFE_EXPLICIT:
      clear_buf(compute_data_buf);
      life_round();
      swapbuf();
      break;
    case TEST_PATTERN: {
      unsigned char x, y;
      unsigned char incr;

      // start out with a clear canvas
      clear_buf(compute_data_buf);
      swapbuf();

      //    for (incr = 32; incr >= 1; incr >>= 1) {
      for (incr = 1; incr <= 32; incr <<= 1) {
	if (current_run_state != TEST_PATTERN) break;
	for (y = 0; y < 48; y += incr) {
	  if (current_run_state != TEST_PATTERN) break;
	  for (x = 0; x < 64; x++) {
	    toggle_show_bit(x,y);
	    next_frame();
	  }
	}
	diag++;
	for (x = 0; x < 48; x++) {
	  switch(diag&3) {
	  case 0:
	    toggle_show_bit(x, x);
	    break;
	  case 1:
	    toggle_show_bit(x + (64 - 48), x);
	    break;
	  case 2:
	    toggle_show_bit(x, 48 - x);
	    break;
	  case 3:
	  default:
	    toggle_show_bit(x + (64 - 48), 48 - x);
	    break;
	  }
	  next_frame();
	}
      }
    }
      break;
    case HOLD_SINGLE:
      ;
      break;
    case HOLD_DOUBLE:
      ;
      break;
    default: ;
    }
  }
}
