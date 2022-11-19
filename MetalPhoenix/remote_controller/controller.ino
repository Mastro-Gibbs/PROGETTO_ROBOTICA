#include <ESP8266WiFi.h>
#include <inttypes.h>

#include "uMQTTBroker.h"

#include "secrets.h"
#include "tc_headers.h"


struct
{
  struct{
    uint8_t cmd;
    uint8_t pressed;
    uint8_t old;
  } keys[5];

  struct
  {
    int pot_val;
    int old_pot_val;
  } Potentiometer;

  const int UPPER = D1;
  const int LOWER = D3;
  const int LEFT  = D4;
  const int RIGHT = D2;
  const int MODE  = D5;
  const int POT   = A0;

} Key;

struct
{
  const int LED_SELF_IP   = D7;
  const int LED_FIRST_IP  = D6;
  const int LED_SECOND_IP = D8;
  const int LED_THIRD_IP  = D0;

  byte self_p;
  byte first_p;
  byte second_p;
  byte third_p;

} LedGroup;


class myMQTTBroker: public uMQTTBroker
{
public:
    virtual bool onConnect(IPAddress addr, uint16_t client_count) {
      //Serial.println(addr.toString()+" connected");
      toogle_led(addr.toString());
      return true;
    }

    virtual void onDisconnect(IPAddress addr, String client_id) {
      //Serial.println(addr.toString()+" ("+client_id+") disconnected");
      toogle_led(addr.toString());
    }

    virtual bool onAuth(String username, String password, String client_id) {
      //Serial.println("Username/Password/ClientId: "+username+"/"+password+"/"+client_id);
      return true;
    }
};

myMQTTBroker myBroker;

void init_Keys()
{
  Key.keys[0].cmd = 'w';
  Key.keys[1].cmd = 'a';
  Key.keys[2].cmd = 's';
  Key.keys[3].cmd = 'd';
  Key.keys[4].cmd = 'm';

  Key.Potentiometer.pot_val = 0;
  Key.Potentiometer.old_pot_val = 0;
}

void init_LedGroup()
{
  LedGroup.self_p   = 0;
  LedGroup.first_p  = 0;
  LedGroup.second_p = 0;
  LedGroup.third_p  = 0;

  digitalWrite(LedGroup.LED_SELF_IP,   LOW);
  digitalWrite(LedGroup.LED_FIRST_IP,  LOW);
  digitalWrite(LedGroup.LED_SECOND_IP, LOW);
  digitalWrite(LedGroup.LED_THIRD_IP,  LOW);
}


void setup()
{
  Serial.begin(9600);
  //Serial.println();

  pinMode(Key.UPPER, INPUT_PULLUP);
  pinMode(Key.LOWER, INPUT_PULLUP);
  pinMode(Key.RIGHT, INPUT_PULLUP);
  pinMode(Key.LEFT,  INPUT_PULLUP);
  pinMode(Key.MODE,  INPUT_PULLUP);

  pinMode(LedGroup.LED_SELF_IP,   OUTPUT);
  pinMode(LedGroup.LED_FIRST_IP,  OUTPUT);
  pinMode(LedGroup.LED_SECOND_IP, OUTPUT);
  pinMode(LedGroup.LED_THIRD_IP,  OUTPUT);

  delay(1000);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pass);
  //Serial.println("AP started");
  //Serial.println("IP address: " + WiFi.softAPIP().toString());

  delay(1000);

  //Serial.println("Starting MQTT broker");
  myBroker.init();

  init_Keys();
  init_LedGroup();

  toogle_led(WiFi.softAPIP().toString());
}


void loop()
{
  Key.keys[0].pressed = digitalRead(Key.UPPER) ? 0:1;
  Key.keys[1].pressed = digitalRead(Key.LEFT)  ? 0:1;
  Key.keys[2].pressed = digitalRead(Key.LOWER) ? 0:1;
  Key.keys[3].pressed = digitalRead(Key.RIGHT) ? 0:1;
  Key.keys[4].pressed = digitalRead(Key.MODE)  ? 0:1;

  Key.Potentiometer.pot_val = (((analogRead(Key.POT) * 3495) / 1024) / 50) * 50;

  detectCommand();
}


void detectCommand()
{
  uint8_t* currState;
  uint8_t* oldState;
  uint8_t* cmd;

  for (uint8_t i = 0; i < 5; i++)
  {
    currState = &Key.keys[i].pressed;
    oldState  = &Key.keys[i].old;
    cmd       = &Key.keys[i].cmd;

    if (*currState == 1 && *currState != *oldState)
    {
      switch (*cmd)
      {
        case 'w':
          myBroker.publish("CMD", "FORWARD");
          Serial.println("FORWARD");
          break;
        case 's':
          myBroker.publish("CMD", "BACKWARD");
          Serial.println("BACKWARD");
          break;
        case 'a':
          myBroker.publish("CMD", "LEFT");
          Serial.println("LEFT");
          break;
        case 'd':
          myBroker.publish("CMD", "RIGHT");
          Serial.println("RIGHT");
          break;
        case 'm':
          myBroker.publish("CMD", "DONE");
          Serial.println("DONE");
          break;
      }

      Key.keys[i].old = *currState;
    }
    else if (*currState == 0 && *currState != *oldState)
    {
      myBroker.publish("CMD", "STOP");
      Serial.println("STOP");

      Key.keys[i].old = *currState;
    }
  }

  if (Key.Potentiometer.pot_val != Key.Potentiometer.old_pot_val)
  {
    myBroker.publish("CMD", (String)(Key.Potentiometer.pot_val + 600));
    Key.Potentiometer.old_pot_val = Key.Potentiometer.pot_val;

    Serial.println((String)(Key.Potentiometer.pot_val + 600));
  }

  delay(20);

}


void toogle_led(const String ip)
{
   if (ip.equalsIgnoreCase("192.168.4.1"))
   {
      LedGroup.self_p == 0 ? digitalWrite(LedGroup.LED_SELF_IP, HIGH) : digitalWrite(LedGroup.LED_SELF_IP, LOW);
      LedGroup.self_p = !LedGroup.self_p;
   }
   else if (ip.equalsIgnoreCase("192.168.4.2"))
   {
      LedGroup.first_p == 0 ? digitalWrite(LedGroup.LED_FIRST_IP, HIGH) : digitalWrite(LedGroup.LED_FIRST_IP, LOW);
      LedGroup.first_p = !LedGroup.first_p;
   }
   else if (ip.equalsIgnoreCase("192.168.4.3"))
   {
      LedGroup.second_p == 0 ? digitalWrite(LedGroup.LED_SECOND_IP, HIGH) : digitalWrite(LedGroup.LED_SECOND_IP, LOW);
      LedGroup.second_p = !LedGroup.second_p;
   }
   else if (ip.equalsIgnoreCase("192.168.4.4"))
   {
      LedGroup.third_p == 0 ? digitalWrite(LedGroup.LED_THIRD_IP, HIGH) : digitalWrite(LedGroup.LED_THIRD_IP, LOW);
      LedGroup.third_p = !LedGroup.third_p;
   }
}