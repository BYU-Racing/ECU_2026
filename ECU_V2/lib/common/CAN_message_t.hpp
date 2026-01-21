#pragma once

/* FlexCAN_T4.h includes a ton of things we don't have available in a testing environment,
 * so we can't use FlexCAN_T4.h when testing. Unfortunately, `CAN_message_t` comes from
 * FlexCAN_T4.h, so we have to make a "fake" `CAN_message_t` so that we can run the test.
 * This is literally just copied from FlexCAN_T4.h. */
#ifdef BUILD_MODE_TEST
typedef struct CAN_message_t {
  uint32_t id = 0;          // can identifier
  uint16_t timestamp = 0;   // FlexCAN time when message arrived
  uint8_t idhit = 0; // filter that id came from
  struct {
    bool extended = 0; // identifier is extended (29-bit)
    bool remote = 0;  // remote transmission request packet type
    bool overrun = 0; // message overrun
    bool reserved = 0;
  } flags;
  uint8_t len = 8;      // length of data
  uint8_t buf[8] = { 0 };       // data
  int8_t mb = 0;       // used to identify mailbox reception
  uint8_t bus = 0;      // used to identify where the message came from when events() is used.
  bool seq = 0;         // sequential frames
} CAN_message_t;
#else
#include "FlexCAN_T4.h"
#endif