#include <Arduino.h>

#define TONE_OUTPUT 1
#define CALLER_HOOK 2
#define DEST_HOOK 3
#define CONNECT_RELAY 4
#define RINGER_RELAY 5
#define TONE_FREQUENCY (440)

void caller_hook_isr();
void dest_hook_isr();
unsigned long caller_hook_last_transition = 0;
unsigned long dest_hook_last_transition = 0;
auto caller_off_hook = false;
auto dest_off_hook = false;
typedef enum
{
  IDLE,
  BUSY,
  WAIT_FOR_DIAL,
  CALLING,
  CONNECTED
} State;

State state = IDLE;

void setup()
{
  pinMode(CALLER_HOOK, INPUT_PULLUP);
  pinMode(DEST_HOOK, INPUT_PULLUP);
  pinMode(CONNECT_RELAY, OUTPUT);
  pinMode(RINGER_RELAY, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(CALLER_HOOK), caller_hook_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DEST_HOOK), dest_hook_isr, CHANGE);
}

void loop()
{
  static State last_state = IDLE;
  static unsigned long last_state_change = 0;
  static unsigned long last_tone_change = 0;
  static unsigned long last_ringer_change = 0;

  if (millis() - caller_hook_last_transition > 100)
  {
    caller_off_hook = digitalRead(CALLER_HOOK);
  }

  if (millis() - dest_hook_last_transition > 100)
  {
    dest_off_hook = digitalRead(DEST_HOOK);
  }

  switch (state)
  {
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
    if (!caller_off_hook || !dest_off_hook)
    {
      state = BUSY;
    }
  }

  if (last_state == BUSY || last_state == CALLING)
  {
    noTone(TONE_OUTPUT);
    digitalWrite(RINGER_RELAY, LOW);
    last_tone_change = 0;
  }

  if (state == BUSY)
  {
    if (millis() - last_tone_change > 1000)
    {
      tone(TONE_OUTPUT, TONE_FREQUENCY, 500);
      last_tone_change = millis();
    }
  }

  if (state == CALLING)
  {
    if (millis() - last_tone_change > 2000)
    {
      tone(TONE_OUTPUT, TONE_FREQUENCY, 1000);
      last_tone_change = millis();
    }
    if (millis() - last_ringer_change > 1000)
    {
      digitalWrite(RINGER_RELAY, digitalRead(RINGER_RELAY));
      last_ringer_change = millis();
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
  }
}

void caller_hook_isr()
{
  caller_hook_last_transition = millis();
}

void dest_hook_isr()
{
  dest_hook_last_transition = millis();
}
