/*
  MIT License

  Copyright (c) 2020 adbenjumea

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#define RX_PIN       2
#define TX_PIN       3

#define TX_RAW_PIN   4
#define TX_CLK_PIN   5

#define RX_DATA_LEN  16
#define TX_DATA_LEN  8

#define BAUD_RATE    1200

#define B0 6
#define B1 7
#define B2 8
#define B3 9
#define B4 10
#define B5 11
#define B6 12
#define B7 13

#define ADDR 29 //direccion 5 en realidad
int GROUP[16] = {1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}; //0 a 15, grupos 1,5 y 16 en realidad
uint8_t Scene;

const int ledPIN = 11;
uint16_t ledPOWER = 0;
uint8_t ledPOWER_fade = 0;
int fadeRate = 1600;//ms

unsigned int Te;

uint16_t rx_buffer = 0;
uint16_t rx_data = 0;
//uint16_t rx_data = 0b100001000111010; //Lampara 34, luminosidad 23%
//uint16_t rx_data = 0b1000100000000010; //Grupo 5, luminosidad 1%
//uint16_t rx_data = 0b1000101000000010; //Grupo 6, luminosidad 1%
//uint16_t rx_data = 0b100011111111; //Lampara 5, luminosidad 100%
//uint16_t rx_data = 0b101011111111; //Lampara 6, luminosidad 100%
//uint16_t rx_data = 0b1111111100100000; //Broadcast, comando 32



void manchester_encode(uint16_t data);
void manchester_decode(void);
void wait_nTe(int n = 1);
void rx_routine(void);

void setup() {
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_RAW_PIN, OUTPUT);
  pinMode(TX_CLK_PIN, OUTPUT);

  pinMode(B0, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(B3, OUTPUT);
  pinMode(B4, OUTPUT);
  pinMode(B5, OUTPUT);
  pinMode(B6, OUTPUT);
  pinMode(B7, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(RX_PIN), manchester_decode, CHANGE);

  Te = 500000 / BAUD_RATE;

  // Initialize LED
  pinMode(ledPIN , OUTPUT);

  // Initialize Serial Port
  Serial.begin(115200);
  while (!Serial) {}
}

void loop() {
  /*Serial.print("Buenas, soy la lámpara ");Serial.print(ADDR+1);Serial.print(" y pertenezco a los grupos ");Serial.print(GROUP[0]+1);Serial.print(", ");
    Serial.print(GROUP[1]+1);Serial.print(" y ");Serial.print(GROUP[2]+1);Serial.println(".");
    Serial.println("");Serial.println("Enviando a lámpara 34, luminosidad 23%");
    rx_data = 0b100001000111010; //Lampara 34, luminosidad 23%
    delay(1000);
    rx_routine();
    Serial.println("");Serial.println("Enviando a grupo 5, luminosidad 1%");
    rx_data = 0b1000100000000010; //Grupo 5, luminosidad 1%
    delay(1000);
    rx_routine();
    Serial.println("");Serial.println("Enviando a grupo 6, luminosidad 1%");
    rx_data = 0b1000101000000010; //Grupo 6, luminosidad 1%
    delay(1000);
    rx_routine();
    Serial.println("");Serial.println("Enviando broadcast, comando 32%");
    rx_data = 0b1111111100100000; //Broadcast, comando 32
    delay(1000);
    rx_routine();
    Serial.println("");Serial.println("Enviando a lámpara 5, luminosidad 100%");
    rx_data = 0b100011111111; //Lampara 5, luminosidad 100%
    delay(1000);
    rx_routine();
    Serial.println("");Serial.println("Enviando a lámpara 6, luminosidad 100%");
    rx_data = 0b101011111111; //Lampara 6, luminosidad 100%
    delay(1000);
    rx_routine();*/

  /*Serial.println("");Serial.println("Enviando a lámpara 5, luminosidad 100%");
    rx_data = 0b100011111111; //Lampara 5, luminosidad 100%
    delay(1000);
    rx_routine();

    Serial.println("");Serial.println("Enviando a lámpara 5, luminosidad 50%");
    rx_data = 0b100010000000; //Lampara 5, luminosidad 50%
    delay(1000);
    rx_routine();

    Serial.println("");Serial.println("Enviando broadcast, comando 0, apagar");
    rx_data = 0b1111111100000000; //Broadcast, comando 0
    delay(1000);
    rx_routine();

    Serial.println("");Serial.println("Enviando a grupo 5, go to scene 10");
    rx_data = 0b1000100100011010; //Grupo 5, gte 10
    delay(1000);
    rx_routine();

    Serial.println("");Serial.println("Enviando a lámpara 5, añadir al grupo 4");
    rx_data = 0b100101100011; //Lampara 5, atg 4
    delay(1000);
    rx_routine();*/

}



void rx_routine(void) {
  int i, j;
  int myADDR = 0; //1 = Trama me incluye, 0 = Trama no me incluye
  uint8_t byte_dir, byte_com;
  //Serial.print((char)rx_data);
  Serial.println(rx_data, BIN);

  byte_dir = rx_data >> 8;
  byte_com = (rx_data << 8) >> 8;

  //COMPROBAR SI SOY DESTINATARIO
  //¿Es un broadcast?
  if ((byte_dir >> 1) == 0b1111111)
  {
    myADDR = 1;
    Serial.println("Soy destinatario. Por Broadcast.");
    //¿Es un grupo?
  }
  else if ((byte_dir >> 5) >= 0b100) {
    if (GROUP[((byte_dir % 0b100000) >> 1)]) //¿Pertenezco al grupo?
    {
      myADDR = 1;
      Serial.println("Soy destinatario. Por pertenecer al grupo.");
    }
  }
  //¿Es mi dirección particular?
  else {
    if (ADDR == (byte_dir >> 1))
    {
      myADDR = 1;
      Serial.println("Soy destinatario. Por ser mi dirección particular.");
    }
  }

  //ACTUAR EN CASO DE SER DESTINATARIO
  if (myADDR) {
    //¿Es un valor directo de luminosidad?
    if (byte_dir % 0b10 == 0)
    {
      Serial.println("Es un valor directo de la luminosidad");
      analogWrite(ledPIN , byte_com);
      ledPOWER = byte_com;
    }
    //¿Es un comando DALI
    else if (byte_dir % 0b10 == 1)
    {
      Serial.println("Es un comando DALI");
      if (byte_com == 0b00000000) //¿Comando OFF?
      {
        Serial.println("Apagar luz");
        analogWrite(ledPIN , 0);
        ledPOWER = 0;
      }
      else if ((byte_com >> 4) == 0b0001) //¿Comando Go To Scene?
      {
        Scene = byte_com % 0b10000;
        Serial.print("Go to scene "); Serial.println(Scene);
        switch (Scene) {
          case 6:
            if (ADDR == 33)
            {
              analogWrite(ledPIN , 20 * 255 / 100);
              ledPOWER = 20 * 255 / 100;
            }
            else if (ADDR == 54)
            {
              analogWrite(ledPIN , 80 * 255 / 100);
              ledPOWER = 20 * 255 / 100;
            }
            break;
          case 8:
            if (ADDR == 54)
            {
              analogWrite(ledPIN , 20 * 255 / 100);
              ledPOWER = 20 * 255 / 100;
            }
            else if (ADDR == 33)
            {
              analogWrite(ledPIN , 80 * 255 / 100);
              ledPOWER = 20 * 255 / 100;
            }
            break;
          default:
            // statements
            break;
        }
      }
      else if ((byte_com >> 4) == 0b0110) //¿Comando Add To Group?
      {
        GROUP[(byte_com % 0b10000)] = 1;
        Serial.print("Add To Group "); Serial.println(byte_com % 0b10000 + 1);
      }
      else if ((byte_com >> 4) == 0b0111) //¿Comando Remove From Group?
      {
        GROUP[(byte_com % 0b10000)] = 0;
        Serial.print("Remove From Group "); Serial.println(byte_com % 0b10000 + 1);
      }
      else if (byte_com == 0b10100000) //¿Comando Query Level?
      {
        Serial.print("Enviando nivel de intensidad "); Serial.println(ledPOWER);
        wait_nTe(8);
        manchester_encode(ledPOWER);
      }
      else if ((byte_com >> 4) == 0b1101) //¿Comando Go To Level?
      {
        Serial.print("Go to level "); Serial.println((byte_com % 0b10000));
        ledPOWER_fade = (byte_com % 0b10000) * 255 / 15;
        if (ledPOWER < ledPOWER_fade)
        {
          while (ledPOWER < ledPOWER_fade)
          {
            for (j = 0; j < 10; j++)
            {
              delayMicroseconds(fadeRate);
            }
            analogWrite(ledPIN , ledPOWER + 1);
            ledPOWER = ledPOWER + 1;
          }
        }
        else if (ledPOWER > ledPOWER_fade)
        {
          while (ledPOWER > ledPOWER_fade)
          {
            for (j = 0; j < 10; j++)
            {
              delayMicroseconds(fadeRate);
            }
            analogWrite(ledPIN , ledPOWER - 1);
            ledPOWER = ledPOWER - 1;
          }
        }
      }

    }
    myADDR = 0;
  }
  else
  {
    Serial.println("No soy destinatario");
  }
  /*
    digitalWrite(B0, bitRead(rx_data, 0));
    digitalWrite(B1, bitRead(rx_data, 1));
    digitalWrite(B2, bitRead(rx_data, 2));
    digitalWrite(B3, bitRead(rx_data, 3));
    digitalWrite(B4, bitRead(rx_data, 4));
    digitalWrite(B5, bitRead(rx_data, 5));
    digitalWrite(B6, bitRead(rx_data, 6));
    digitalWrite(B7, bitRead(rx_data, 7));
  */
}



// ====================== //
//    COMMON FUNCTIONS    //
// ====================== //
void manchester_encode(uint16_t data) {
  detachInterrupt(digitalPinToInterrupt(RX_PIN));


  // ***** START BIT ***** //
  digitalWrite(TX_PIN, 1);
  wait_nTe();

  digitalWrite(TX_PIN, 0);
  wait_nTe();
  // ******************** //


  for (int i = (TX_DATA_LEN - 1); i >= 0; i--) {
    byte current_bit;
    current_bit = bitRead(data, i);

    // ---- For debuging only ---- //
    //digitalWrite(TX_RAW_PIN, current_bit);
    //digitalWrite(TX_CLK_PIN, 0);
    // --------------------------- //


    digitalWrite(TX_PIN, current_bit);
    wait_nTe();


    // ---- For debuging only ---- //
    //digitalWrite(TX_CLK_PIN, 1);
    // --------------------------- //


    digitalWrite(TX_PIN, !current_bit);
    wait_nTe();
  }

  // ***** STOP BIT ***** //
  digitalWrite(TX_CLK_PIN, 0);
  digitalWrite(TX_PIN, 0);
  wait_nTe(4);
  // ******************* //
  attachInterrupt(digitalPinToInterrupt(RX_PIN), manchester_decode, CHANGE);
}


void manchester_decode(void) {
  byte current_bit;
  static int rx_count = 0;
  static bool receiving = false;

  delayMicroseconds(Te >> 1);

  if (!receiving) {
    if (digitalRead(RX_PIN)) receiving = true;
  }
  else {
    detachInterrupt(digitalPinToInterrupt(RX_PIN));
    delayMicroseconds(Te + (Te >> 1));

    current_bit = digitalRead(RX_PIN);
    rx_buffer <<= 1;
    bitWrite(rx_buffer, 0, current_bit);
    rx_count++;

    if (rx_count == RX_DATA_LEN) {
      wait_nTe(4);  // Stop bits
      rx_data = rx_buffer;
      rx_buffer = 0;
      rx_count = 0;
      receiving = false;
      rx_routine();
    }

    attachInterrupt(digitalPinToInterrupt(RX_PIN), manchester_decode, CHANGE);

  }
}



void wait_nTe(int n) {
  for (int i = 0; i < n; i++) {
    delayMicroseconds(Te);
  }
}
