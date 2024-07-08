#include <Arduino.h>

#define CALLER_HOOK 2
#define DEST_HOOK 3
#define CONNECT_RELAY 4
#define RINGER_RELAY 5
#define TONE_OUTPUT 6

#define TONE_FREQUENCY (440)

void caller_hook_isr();
void dest_hook_isr();

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

void setup()
{
  Serial.begin(9600);
  pinMode(CALLER_HOOK, INPUT_PULLUP);
  pinMode(DEST_HOOK, INPUT_PULLUP);
  pinMode(CONNECT_RELAY, OUTPUT);
  pinMode(RINGER_RELAY, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(CALLER_HOOK), caller_hook_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DEST_HOOK), dest_hook_isr, CHANGE);
}

void loop()
{
  static State last_state = UNINITIALIZED;
  static unsigned long last_state_change = 0;
  static unsigned long last_tone_change = 0;
  static unsigned long last_ringer_change = 0;

  if (millis() - caller_hook_last_transition > 300)
  {
    caller_off_hook = !digitalRead(CALLER_HOOK);
  }

  if (millis() - dest_hook_last_transition > 300)
  {
    dest_off_hook = !digitalRead(DEST_HOOK);
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
    if (!caller_off_hook || !dest_off_hook)
    {
      state = BUSY;
    }
  }

  if ((last_state == BUSY && state != BUSY) ||
      (last_state == CALLING && state != CALLING) ||
      (last_state == WAIT_FOR_DIAL && state != WAIT_FOR_DIAL))
  {
    noTone(TONE_OUTPUT);
    digitalWrite(RINGER_RELAY, LOW);
    last_tone_change = 0;
  }

  if (state == WAIT_FOR_DIAL && last_state != WAIT_FOR_DIAL)
  {
    tone(TONE_OUTPUT, TONE_FREQUENCY, 5000);
  }

  if (state == BUSY)
  {
    if (millis() - last_tone_change > 500)
    {
      noTone(TONE_OUTPUT);
      tone(TONE_OUTPUT, TONE_FREQUENCY, 250);
      last_tone_change = millis();
    }
  }

  if (state == CALLING)
  {
    if (millis() - last_tone_change > 4000)
    {
      noTone(TONE_OUTPUT);
      tone(TONE_OUTPUT, TONE_FREQUENCY, 1500);
      last_tone_change = millis();
    }
    if (millis() - last_ringer_change > 2000)
    {
      digitalWrite(RINGER_RELAY, !digitalRead(RINGER_RELAY));
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
    Serial.print("State: ");
    String n = state_to_string(state);
    Serial.println(n);
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