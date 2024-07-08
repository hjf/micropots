#include <Arduino.h>
#include <SoftModem.h>
#include "NewTone.h"

#define HOOK_PIN 2
#define CALLER_HOOK_PIN 10
#define DEST_HOOK_PIN 8
#define CONNECT_RELAY 4
#define RINGER_RELAY 5
#define TONE_OUTPUT 9

#define TONE_FREQUENCY (440)

void caller_hook_isr();
SoftModem modem = SoftModem();

unsigned long caller_hook_last_transition = 0;
unsigned long dest_hook_last_transition = 0;
auto caller_off_hook = false;
auto dest_off_hook = false;
typedef enum
{
  UNINITIALIZED,
  IDLE,
  BUSY,
  WAIT_FOR_DIAL,
  CALLING,
  CONNECTED
} State;

State state = UNINITIALIZED;
String state_to_string(State s);
void transmit_caller_id();

void setup()
{
  Serial.begin(9600);
  modem.begin();
  pinMode(HOOK_PIN, INPUT_PULLUP);
  pinMode(CALLER_HOOK_PIN, INPUT_PULLUP);
  pinMode(DEST_HOOK_PIN, INPUT_PULLUP);
  pinMode(CONNECT_RELAY, OUTPUT);
  pinMode(RINGER_RELAY, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(HOOK_PIN), caller_hook_isr, CHANGE);
}

volatile boolean caller_hook_read = false;
volatile boolean dest_hook_read = false;

void loop()
{
  static State last_state = UNINITIALIZED;
  static unsigned long last_state_change = 0;
  static unsigned long last_tone_change = 0;
  static unsigned long last_ringer_change = 0;
  static long rings = 0;

  if (millis() - caller_hook_last_transition > 300)
  {
    caller_off_hook = caller_hook_read;
  }

  if (millis() - dest_hook_last_transition > 300)
  {
    dest_off_hook = dest_hook_read;
  }

  switch (state)
  {
  case UNINITIALIZED:
    state = IDLE;
  case IDLE:
    if (caller_off_hook && dest_off_hook)
    {
      state = BUSY;
    }
    else if (caller_off_hook && !dest_off_hook)
    {
      state = WAIT_FOR_DIAL;
    }
    break;

  case BUSY:
    if (!caller_off_hook && !dest_off_hook)
    {
      state = IDLE;
    }
    break;

    // "autodial" after 2s
  case WAIT_FOR_DIAL:
    rings = 0;
    if (!caller_off_hook)
    {
      state = IDLE;
    }
    if (millis() - last_state_change > 2000)
    {
      state = CALLING;
    }
    break;

  case CALLING:
    if (!caller_off_hook)
    {
      state = IDLE;
    }
    if (caller_off_hook && dest_off_hook)
    {
      state = CONNECTED;
    }
    break;

  case CONNECTED:
    if (!caller_off_hook)
    {
      state = BUSY;
    }
  }

  if ((last_state == BUSY && state != BUSY) ||
      (last_state == CALLING && state != CALLING) ||
      (last_state == WAIT_FOR_DIAL && state != WAIT_FOR_DIAL))
  {
    noNewTone(TONE_OUTPUT);
    digitalWrite(RINGER_RELAY, LOW);
    last_tone_change = 0;
  }

  if (state == WAIT_FOR_DIAL && last_state != WAIT_FOR_DIAL)
  {
    NewTone(TONE_OUTPUT, TONE_FREQUENCY, 5000);
  }

  if (state == BUSY)
  {
    if (millis() - last_tone_change > 500)
    {
      noNewTone(TONE_OUTPUT);
      NewTone(TONE_OUTPUT, TONE_FREQUENCY, 250);
      last_tone_change = millis();
    }
  }

  if (state == CALLING)
  {
    if (millis() - last_tone_change > 4000)
    {
      noNewTone(TONE_OUTPUT);
      NewTone(TONE_OUTPUT, TONE_FREQUENCY, 1500);
      last_tone_change = millis();
    }
    if (millis() - last_ringer_change > 2000)
    {
      if (rings == 1)
      {
        transmit_caller_id();
      }
      digitalWrite(RINGER_RELAY, !digitalRead(RINGER_RELAY));
      last_ringer_change = millis();
      rings++;
    }
  }

  if (state == CONNECTED)
  {
    digitalWrite(RINGER_RELAY, LOW);
    digitalWrite(CONNECT_RELAY, HIGH);
  }
  else
  {
    digitalWrite(CONNECT_RELAY, LOW);
  }

  if (last_state != state)
  {
    last_state = state;
    last_state_change = millis();
    Serial.print("State: ");
    String n = state_to_string(state);
    Serial.println(n);
  }
}

void caller_hook_isr()
{
  auto current_caller_hook_read = !digitalRead(CALLER_HOOK_PIN);
  auto current_dest_hook_read = !digitalRead(DEST_HOOK_PIN);
  if (current_caller_hook_read != caller_hook_read)
  {
    caller_hook_read = current_caller_hook_read;
    caller_hook_last_transition = millis();
  }
  if (current_dest_hook_read != dest_hook_read)
  {
    dest_hook_read = current_dest_hook_read;
    dest_hook_last_transition = millis();
  }
}

String state_to_string(State s)
{
  switch (s)
  {
  case UNINITIALIZED:
    return "UNINITIALIZED";
  case IDLE:
    return "IDLE";
  case BUSY:
    return "BUSY";
  case WAIT_FOR_DIAL:
    return "WAIT_FOR_DIAL";
  case CALLING:
    return "CALLING";
  case CONNECTED:
    return "CONNECTED";
  }
}

#define TX_BUF_LEN (64)
void transmit_caller_id()
{
  detachInterrupt(digitalPinToInterrupt(HOOK_PIN));
  delay(100);

  for (int i = 0; i < 30; i++)
  {
    modem.write(0x55);
  }

  // transmit 1200hz for 140ms
  char txbuf[TX_BUF_LEN];
  memset(txbuf, 0, TX_BUF_LEN);

  char message[] = {0x30, 0x39, 0x33, 0x30, 0x31, 0x32, 0x32, 0x34, 0x36, 0x30, 0x39, 0x35, 0x35, 0x35, 0x31, 0x32, 0x31, 0x32, 0x00};
  char msglen = strlen(message);

  txbuf[0] = 0x04; // message type
  txbuf[1] = msglen;

  memcpy(txbuf + 2, message, msglen);
  int checksum_position = msglen + 2;

  // The Checksum Word contains the twos complement of the modulo 256 sum of the message:
  unsigned char checksum = 0;
  for (unsigned int i = 0; i < checksum_position; i++)
  {
    checksum += txbuf[i];
  }
  checksum = ~checksum + 1;

  txbuf[checksum_position] = checksum;

  modem.write((uint8_t *)txbuf, strlen(txbuf));
  delay(100);
  attachInterrupt(digitalPinToInterrupt(HOOK_PIN), caller_hook_isr, CHANGE);
}