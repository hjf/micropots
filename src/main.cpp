#include <Arduino.h>
#define SOFT_MODEM_BAUD_RATE (1200)
#define SOFT_MODEM_LOW_FREQ (1200)
#define SOFT_MODEM_HIGH_FREQ (2200)
#define SOFT_MODEM_RX_BUF_SIZE (32)
#define SOFT_MODEM_DEBUG_ENABLE (1)
#include <SoftModem.h>
#include "NewTone.h"

#define HOOK_PIN 2
#define CALLER_HOOK_PIN 10
#define DEST_HOOK_PIN 8
#define CONNECT_RELAY 4
#define RINGER_RELAY 5
#define TONE_OUTPUT 9

#define TONE_FREQUENCY (440)

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
char dtmf_majority(char n);
void caller_hook_isr();
char get_dtmf();
char findMajority(char arr[], char n);

void setup()
{
  Serial.begin(9600);
  modem.begin();
  pinMode(HOOK_PIN, INPUT_PULLUP);
  pinMode(CALLER_HOOK_PIN, INPUT_PULLUP);
  pinMode(DEST_HOOK_PIN, INPUT_PULLUP);
  pinMode(CONNECT_RELAY, OUTPUT);
  pinMode(RINGER_RELAY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(HOOK_PIN), caller_hook_isr, CHANGE);
}

volatile boolean caller_hook_read = false;
volatile boolean dest_hook_read = false;

#define NUMBER_LENGTH (32)
void loop()
{
  char prev_dtmf = 0;
  int spaces = 0;
  int ncount = 0;
  char number[NUMBER_LENGTH];
  memset(number, 0, NUMBER_LENGTH);
  while (spaces < 20 && ncount < NUMBER_LENGTH - 1)
  {
    char dtmf = dtmf_majority(3);
    if (dtmf != 0)
    {
      // Serial.println(dtmf);
      if (dtmf == '_')
        spaces++;
      else if (spaces > 1)
      {
        number[ncount++] = dtmf;
        spaces = 0;
      }
    }
    else
    {
      // Serial.println("No DTMF");
    }

    // else
    // {
    //   spaces++;
    // }
    // prev_dtmf = dtmf;
  }
  if (number[0] != 0)
    Serial.println(number);
  return;

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

  detachInterrupt(digitalPinToInterrupt(HOOK_PIN));
  delay(10);
  for (int i = 0; i < 30; i++)
  {
    modem.write(0x55);
  }

  modem.sendTone();
  modem.write((uint8_t *)txbuf, strlen(txbuf));
  delay(10);
  attachInterrupt(digitalPinToInterrupt(HOOK_PIN), caller_hook_isr, CHANGE);
}
#define ADC_MIDPOINT (512)
#define SAMPLE_RATE (8900)
#define SAMPLE_SIZE (50)
#define MAG_THRESHOLD (3500)

#include <Goertzel.h>
Goertzel X0(1209.0, SAMPLE_RATE);
Goertzel X1(1336.0, SAMPLE_RATE);
Goertzel X2(1477.0, SAMPLE_RATE);
Goertzel Y0(697.0, SAMPLE_RATE);
Goertzel Y1(770.0, SAMPLE_RATE);
Goertzel Y2(852.0, SAMPLE_RATE);
Goertzel Y3(941.0, SAMPLE_RATE);

char dtmf_lut[] = {
    '1',
    '2',
    '3',
    '4',
    '5',
    '6',
    '7',
    '8',
    '9',
    '*',
    '0',
    '#',
    'p',
    'q',
    'r',
    's',
};

char get_dtmf()
{
  int samples[SAMPLE_SIZE];
  float max_x = -1;
  float max_y = -1;
  int n = -1;
  noInterrupts();
  for (int i = 0; i < SAMPLE_SIZE; i++)
  {
    samples[i] = analogRead(A0);
  }
  interrupts();

  // calculate standard deviation from samples:
  float sum = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++)
  {
    sum += samples[i];
  }
  float mean = sum / SAMPLE_SIZE;

  float variance = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++)
  {
    variance += pow(samples[i] - mean, 2);
  }
  float standardDeviation = sqrt(variance / SAMPLE_SIZE);

  if (standardDeviation < 10)
  {
    return '_';
  }

  n = 0;
  float current_x = X0.Mag(samples, SAMPLE_SIZE);
  max_x = current_x;

  current_x = X1.Mag(samples, SAMPLE_SIZE);
  if (current_x > max_x)
  {
    n = 1;
    max_x = current_x;
  }
  current_x = X2.Mag(samples, SAMPLE_SIZE);
  if (current_x > max_x)
  {
    n = 2;
    max_x = current_x;
  }

  if (max_x < MAG_THRESHOLD)
    return 0;

  float current_y = Y0.Mag(samples, SAMPLE_SIZE);
  max_y = current_y;
  int ysum = 0;
  current_y = Y1.Mag(samples, SAMPLE_SIZE);

  if (current_y > max_y)
  {
    ysum = 3;
    max_y = current_y;
  }

  current_y = Y2.Mag(samples, SAMPLE_SIZE);
  if (current_y > max_y)
  {
    ysum = 6;
    max_y = current_y;
  }

  current_y = Y3.Mag(samples, SAMPLE_SIZE);
  if (current_y > max_y)
  {
    ysum = 9;
    max_y = current_y;
  }

  if (max_y < MAG_THRESHOLD)
    return 0;

  n += ysum;

  return dtmf_lut[n & 0x0f];
}

char dtmf_majority(char n)
{
  // call get_dtmf 3 times and choose the majority result:
  char dtmf[n];
  for (int i = 0; i < n; i++)
  {
    dtmf[i] = get_dtmf();
  }
  return findMajority(dtmf, n);
}

char findMajority(char arr[], char n)
{
  int maxCount = 0;
  int index = -1; // sentinels
  for (int i = 0; i < n; i++)
  {
    int count = 0;
    for (int j = 0; j < n; j++)
    {
      if (arr[i] == arr[j])
        count++;
    }

    // update maxCount if count of
    // current element is greater
    if (count > maxCount)
    {
      maxCount = count;
      index = i;
    }
  }

  // if maxCount is greater than n/2
  // return the corresponding element
  if (maxCount > n / 2)
    return arr[index];
}
