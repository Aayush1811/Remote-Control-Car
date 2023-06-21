#include <SPI.h>
#include <RH_NRF24.h>


# define en1 5
# define in1 3
# define in2 4
# define en2 6
# define in3 2
# define in4 7

// Singleton instance of the radio driver
RH_NRF24 nrf24;
// RH_NRF24 nrf24(8, 7); // use this to be electrically compatible with Mirf
// RH_NRF24 nrf24(8, 10);// For Leonardo, need explicit SS pin
// RH_NRF24 nrf24(8, 7); // For RFM73 on Anarduino Mini

void forward(int p)
{
  analogWrite(en1,p);
  analogWrite(en2,p);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  
}

void reverse(int p)
{
  analogWrite(en1,p);
  analogWrite(en2,p);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  
}

void stopp()
{
  analogWrite(en1,0);
  analogWrite(en2,0);
}

void setup() 
{
  pinMode(en1,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  Serial.begin(115200);
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

void loop()
{
  if (nrf24.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (nrf24.recv(buf, &len))
    {
//      NRF24::printBuffer("request: ", buf, len);
      //Serial.print("got request: ");
      //Serial.println((char*)buf);

       if(buf[0] == 'F')
       {
            Serial.println("FORWARD");
            forward(200);
       }
       else if(buf[0] == 'R')
       {
            Serial.println("REVERSE");
            reverse(200);
       }
       else
       {
            Serial.println("STOP");
            stopp();
       }
      
      // Send a reply
      uint8_t data[] = "And hello back to you";
      //nrf24.send(data, sizeof(data));
      //nrf24.waitPacketSent();
      //Serial.println("Sent a reply");
    }
    else
    {
      Serial.println("recv failed");
    }
  }

 
}
