#include <WiFi.h>
#include <PubSubClient.h>
//#include <esp_now.h>
#include <WiFi.h>
#include <SPI.h>
#include <RH_NRF24.h>

RH_NRF24 nrf24;
//#include "BluetoothSerial.h"
/*
uint8_t broadcastAddress[] = {0xC0,0x49,0xEF,0xCB,0xA6,0x64};


esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\t\t");
  //Serial.print("Last Packet Send Status: ");
  //Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  //Serial.print("\t\t");
}
*/


String dir = "STOP";
uint8_t data[] = "FORWARD";

# define led 2
# define led2 3

# define x1 A0
# define y1 A1
//# define sw1 9

# define x2 A2
# define y2 A3
//# define sw2 10

//# define pot 27

//# define dir 4
//# define m1 3
int x;
int y;
int s;
int pwm;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
/*
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    //Serial.println("Error initializing ESP-NOW");
    return;
  }
 
  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
 */

  pinMode(led,OUTPUT);
  digitalWrite(led,LOW);
  pinMode(led2,OUTPUT);
  digitalWrite(led2,LOW);
  pinMode(x1,INPUT);
  pinMode(y1,INPUT);
//  pinMode(sw1,OUTPUT);
  pinMode(x2,INPUT);
  pinMode(y2,INPUT);
 // pinMode(sw2,OUTPUT);
 // digitalWrite(sw1,HIGH);
 // digitalWrite(sw2,HIGH);
 // pinMode(m1,OUTPUT);
  //pinMode(pot,INPUT);
//  pinMode(dir,OUTPUT);


  while (!Serial) 
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed"); 
  
  

}

void loop() {

  // put your main code here, to run repeatedly:
  x = analogRead(x1);
  y = analogRead(y2);
//  s = digitalRead(sw1);
//  pwm = analogRead(pot);
  //Serial.println(s);
  //Serial.println(y);
  if(x == 0 )
  {
    dir = "FORWARD";
    //client.publish("robot","FORWARD");
    //Serial.print("forward");
    
    //Serial.print("\t\t");
    
    digitalWrite(led,HIGH);
    uint8_t data[] = "FORWARD";
    nrf24.send(data, sizeof(data));
    
  }
  else if(x!=0 && x!=1023)
  {
    //client.publish("robot","STOP");
    digitalWrite(led,LOW);
    dir = "STOP";
    uint8_t data[] = "STOP";
    nrf24.send(data, sizeof(data));
  }
 
  if(x==1023)
  {
    //client.publish("robot","REVERSE");
    //Serial.print("reverse");
    dir = "REVERSE";
    //Serial.print("\t\t");
    
    digitalWrite(led2,HIGH);
    uint8_t data[] = "REVERSE";
    nrf24.send(data, sizeof(data));
    
    
  }
  else if(x!=0 && x!=1023)
  {
    digitalWrite(led2,LOW);
    dir = "STOP";
    uint8_t data[] = "STOP";
    nrf24.send(data, sizeof(data));
  }

  if(y == 0)
  {
    //Serial.print("right");
    dir = "RIGHT";
    //Serial.print("\t\t");
    //client.publish("bot1","RIGHT");
  }

  if(y == 1023)
  {
    //Serial.print("left");
    dir = "LEFT";
    //Serial.print("\t\t");
    //client.publish("bot1","LEFT");
  }

  Serial.print("x: ");
  Serial.print(x);
  Serial.print("\t\t");

  Serial.print("y: ");
  Serial.print(y);
  Serial.print("\t\t");

  Serial.print("dir: ");
  Serial.print(dir);
  Serial.print("\t\t");

  

  //Serial.println("Sending to nrf24_server");
  // Send a message to nrf24_server
  
  
  
  nrf24.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  Serial.print("data: ");
  Serial.print(*data);
  Serial.print("\t\t");
  
  Serial.print("pwm: ");
  Serial.println(pwm);
  //Serial.print("\t\t");

  
 /* 
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dir, sizeof(dir));
   
  if (result == ESP_OK) {
    //Serial.print("Sending confirmed");
  }
  else {
    //Serial.print("Sending error");
  }
  //Serial.print("\t\t");

  //Serial.println(dir);
  */
  
  
}
