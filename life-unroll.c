#include <avr/io.h>
#include "life-unroll.h"

static inline unsigned char life_round_byte(unsigned char prevbyte, unsigned char thisbyte, unsigned char nextbyte,
					    unsigned char prev_byte_count, unsigned char next_byte_count)
{
  unsigned char bi;
  unsigned char prev_col_count=0, this_col_count=0, next_col_count=0;
  unsigned char outbyte = 0;
  unsigned char count;

  for (bi = 0; bi < 8; bi++) {
    if (bi == 0) {
      prev_col_count = prev_byte_count;
      this_col_count = 0;
      if (prevbyte & _BV(bi))
	this_col_count++;
      if (thisbyte & _BV(bi))
	this_col_count++;
      if (nextbyte & _BV(bi))
	this_col_count++;
    } else {
      prev_col_count = this_col_count;
      this_col_count = next_col_count;
    }
    if (bi == 7) {
      next_col_count = next_byte_count;
    } else {
      next_col_count = 0;
      if (prevbyte & _BV(bi+1))
	next_col_count++;
      if (thisbyte & _BV(bi+1))
	next_col_count++;
      if (nextbyte & _BV(bi+1))
	next_col_count++;
    }
    count = prev_col_count + this_col_count + next_col_count;
    if (thisbyte & _BV(bi)) {
      // previously alive.  count includes the current pixel.
      if (count == (2+1) || count == (3+1))
	outbyte |= _BV(bi);
    } else {
      // previously dead
      if (count == 3)
	outbyte |= _BV(bi);
    }
  }
  return outbyte;
}

void life_round_line(unsigned char *prevline, unsigned char *thisline, unsigned char *nextline,
		     unsigned char count_on_left, unsigned char count_on_right,
		     unsigned char *thisline_out)
{
  unsigned char prev_byte_count;
  unsigned char next_byte_count;
  unsigned char by;

  for (by = 0; by < 8; by++) {
    if (by == 0) {
      prev_byte_count = count_on_left;
    } else {
      prev_byte_count = 0;
      if (prevline[by-1] & _BV(7))
	prev_byte_count++;
      if (thisline[by-1] & _BV(7))
	prev_byte_count++;
      if (nextline[by-1] & _BV(7))
	prev_byte_count++;
    }

    if (by == 7) {
      next_byte_count = count_on_right;
    } else {
      next_byte_count = 0;
      if (prevline[by+1] & _BV(0))
	next_byte_count++;
      if (thisline[by+1] & _BV(0))
	next_byte_count++;
      if (nextline[by+1] & _BV(0))
	next_byte_count++;
    }
    thisline_out[by] = life_round_byte(prevline[by], thisline[by], nextline[by], prev_byte_count, next_byte_count);
  }
}
