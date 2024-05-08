#include <Arduino.h>

/* > Defines */
#define BITCHAR_ALPHABET_SIZE 7
#define BITCHAR_ALPHABET_ANIMATION_SIZE 1
#define BITCHAR_ALPHABET_ANIMATION_FRAMES 16

/* > Variables definitions */
static uint8_t bitchar_alphabet
[BITCHAR_ALPHABET_SIZE][8] = {
    { /* SYM0 - SEPARATOR WITH UPPER ARROW */
      0b01010,
      0b10101,
      0b01010,
      0b10101,
      0b01010,
      0b10101,
      0b01010,
      0b10101
    },
    { /* SYM1 - SEPARATOR WITH LOWER ARROW */
      0b10101,
      0b01010,
      0b10101,
      0b01010,
      0b10101,
      0b01010,
      0b10101,
      0b01010
    },
    { /* SYM2 - LETTER D */
      0b11110,
      0b10001,
      0b10001,
      0b10001,
      0b10001,
      0b10001,
      0b11110,
      0b00000
    },
    { /* SYM3 - LEFT ARROW */
      0b00010,
      0b00110,
      0b01110,
      0b11111,
      0b11111,
      0b01110,
      0b00110,
      0b00010
    },
    { /* SYM4 - HORIZONTAL LINE IN THE MIDDLE */
      0b00000,
      0b00000,
      0b00000,
      0b11111,
      0b11111,
      0b00000,
      0b00000,
      0b00000
    },
    { /* SYM5 - RIGHT ARROW */
      0b01000,
      0b01100,
      0b01110,
      0b11111,
      0b11111,
      0b01110,
      0b01100,
      0b01000
    },
    { /* SYM6 - BLANK */
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    }
};

static uint8_t bitchar_alphabet_animation
[BITCHAR_ALPHABET_ANIMATION_SIZE][BITCHAR_ALPHABET_ANIMATION_FRAMES][8] =
{
  {
    {

      0b00100,
      0b01110,
      0b11111,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100
    },
    {
      0b01110,
      0b11111,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00000
    },
    {
      0b11111,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b01110
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b01110,
      0b11111
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b01110,
      0b11111,
      0b00100
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b01110,
      0b11111,
      0b00100,
      0b00100
    },
    {
      0b00000,
      0b00000,
      0b00100,
      0b01110,
      0b11111,
      0b00100,
      0b00100,
      0b00100
    },
    {
      0b00000,
      0b00100,
      0b01110,
      0b11111,
      0b00100,
      0b00100,
      0b00100,
      0b00100
    }
  },
  {
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b01110,
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b11111,
      0b01110,
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b11111,
      0b01110,
      0b00100,
      0b00000,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b11111,
      0b01110,
      0b00100,
      0b00000,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b00100,
      0b11111,
      0b01110,
      0b00100,
      0b00000,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b11111,
      0b01110,
      0b00100,
      0b00000
    },
    {
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b11111,
      0b01110,
      0b00100
    },
    {
      0b00000,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b11111,
      0b01110
    },
    {
      0b00000,
      0b00000,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b11111
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b00100,
      0b00100,
      0b00100,
      0b00100
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b00100,
      0b00100,
      0b00100
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b00100,
      0b00100
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100,
      0b00100
    },
    {
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00000,
      0b00100
    }
  }
};