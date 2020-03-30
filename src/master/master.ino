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

#define RX_DATA_LEN  8
#define TX_DATA_LEN  16

#define BAUD_RATE    1200

#define B0 6
#define B1 7
#define B2 8
#define B3 9
#define B4 10
#define B5 11
#define B6 12
#define B7 13


unsigned int Te;
bool receiving = false;

uint16_t rx_buffer = 0;
uint16_t rx_data = 0;
int rx_count = 0;

void manchester_encode(uint16_t data);
void manchester_decode(void);
void wait_nTe(int n = 1);
void rx_routine(void);

void setup() {
  pinMode(TX_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  //pinMode(TX_RAW_PIN, OUTPUT);
  //pinMode(TX_CLK_PIN, OUTPUT);

  /*
  pinMode(B0, OUTPUT);
  pinMode(B1, OUTPUT);
  pinMode(B2, OUTPUT);
  pinMode(B3, OUTPUT);
  pinMode(B4, OUTPUT);
  pinMode(B5, OUTPUT);
  pinMode(B6, OUTPUT);
  pinMode(B7, OUTPUT);
  */
  
  attachInterrupt(digitalPinToInterrupt(RX_PIN), manchester_decode, CHANGE);
  
  Te = 500000 / BAUD_RATE;

  // Initialize Serial Port
  Serial.begin(115200);
  while (!Serial) {}
  
  // Waits for slaves to connect
  delay(100);
}


void loop() {
  uint16_t trama;
  int trama_parcial;
  String aux;

  aux=="0";

    while(aux!="1")
    {
      Serial.print("¿Es una sola lámpara? (1 sí, 0 no)");
      while(!((Serial.available() > 0)));
      while (Serial.available() > 0) {
          aux = Serial.readStringUntil('\n');
      }
      if(aux=="1") //Es una sola lámara
      {
        trama = 0b0000000000000000;
        Serial.print(". Introduce dirección de la lámpara (1-64): ");
        while(!((Serial.available() > 0)));
        while (Serial.available() > 0) {
          aux = Serial.readStringUntil('\n');
          Serial.print(aux);
          if(1<=aux.toInt()&&aux.toInt()<=64) //Si es una dirección válida
          {
            trama_parcial = (aux.toInt()-1)<<9;
            trama = trama + trama_parcial;
            aux = "1";
          }
          else
          {
            aux = "0";
            Serial.println("");
          }
        }
      }
      else
      {
        Serial.println("");Serial.print("¿Es una grupo? (1 sí, 0 no)");
        while(!((Serial.available() > 0)));
        while (Serial.available() > 0) {
          aux = Serial.readStringUntil('\n');
        }
        if(aux=="1") //Es un grupo
        {
          trama = 0b1000000000000000;
          Serial.print(". Introduce dirección del grupo (1-16): ");
          while(!((Serial.available() > 0)));
          while (Serial.available() > 0) {
            aux = Serial.readStringUntil('\n');
            Serial.print(aux);
            
            if(1<=aux.toInt()&&aux.toInt()<=16) //Si es una dirección válida
            {
              trama_parcial = (aux.toInt()-1)<<9;
              trama = trama + trama_parcial;
              aux = "1";
            }
            else
            {
              aux = "0";
              Serial.println("");
            }
          }
        }
        else{
          Serial.println("");Serial.print("¿Es un broadcast? (1 sí, 0 no)");
          while(!((Serial.available() > 0)));
          while (Serial.available() > 0) {
            aux = Serial.readStringUntil('\n');
          }
          if(aux=="1") //Es un broadcast
          {
            trama = 0b1111111000000000;
            Serial.print(" ");
          }
          
        }
      }
      if(aux=="0") Serial.println("");
    }
    Serial.println(""); Serial.println("DIRECCIÓN LEÍDA");

    aux="0";
    while(aux!="1")
    {
      Serial.print("¿Es un valor directo a la luminosidad? (1 sí, 0 no)");
      while(!((Serial.available() > 0)));
      while (Serial.available() > 0) {
          aux = Serial.readStringUntil('\n');
      }
      if(aux=="1") //Es un valor directo a la luminosidad
      {
        Serial.print(". Introduce el porcentaje del valor de luminosidad (0-100): ");
        while(!((Serial.available() > 0)));
        while (Serial.available() > 0) {
          aux = Serial.readStringUntil('\n');
          Serial.print(aux);
          trama_parcial = int(aux.toInt()*255/100);
          trama = trama + trama_parcial;
          aux="1";
        }
      }
      else
      {
        Serial.println("");Serial.print("¿Es un comando DALI? (1 sí, 0 no)");
        while(!((Serial.available() > 0)));
        while (Serial.available() > 0) {
          aux = Serial.readStringUntil('\n');
        }
        if(aux=="1") //Es un comando DALI
        {
          trama = trama + 0b100000000;
          Serial.print(". Introduce el comando: ");
          while(!((Serial.available() > 0)));
          while (Serial.available() > 0) {
            aux = Serial.readStringUntil('\n');
            Serial.print(aux);
            if(aux=="OFF")
            {
              trama_parcial = 0b00000000;
              trama = trama + trama_parcial;
              aux="1";
            }
            else if(aux=="GO_TO_SCENE")
            {
              Serial.print(". Introduce el número de escena: ");
              while(!((Serial.available() > 0)));
              while (Serial.available() > 0) {
                aux = Serial.readStringUntil('\n');
                Serial.print(aux);
                trama_parcial = 0b00010000+aux.toInt();
                trama = trama + trama_parcial;
                aux="1";
              }
            }
            else if(aux=="ADD_TO_GROUP")
            {
              Serial.print(". Introduce el nuevo grupo: ");
              while(!((Serial.available() > 0)));
              while (Serial.available() > 0) {
                aux = Serial.readStringUntil('\n');
                Serial.print(aux);
                trama_parcial = 0b01100000+aux.toInt()-1;
                trama = trama + trama_parcial;
                aux="1";
              }
            }
            else if(aux=="REMOVE_FROM_GROUP")
            {
              Serial.print(". Introduce el grupo a eliminar: ");
              while(!((Serial.available() > 0)));
              while (Serial.available() > 0) {
                aux = Serial.readStringUntil('\n');
                Serial.print(aux);
                trama_parcial = 0b01110000+aux.toInt()-1;
                trama = trama + trama_parcial;
                aux="1";
              }
            }
            else if(aux=="QUERY_LEVEL")
            {
              Serial.print(". Petición enviada.");
              trama_parcial = 0b10100000;
              trama = trama + trama_parcial;
              aux="1";
            }
            else if(aux=="GO_TO_LEVEL")
            {
              Serial.print(". Introduce el porcentaje del valor de luminosidad (0-100): ");
              while(!((Serial.available() > 0)));
              while (Serial.available() > 0) {
                aux = Serial.readStringUntil('\n');
                Serial.print(aux);
                trama_parcial = 0b11010000+aux.toInt()*15/100;
                trama = trama + trama_parcial;
                aux="1";
              }
            }
            else
            {
              Serial.print(". Comando no registrado.");
              aux="0";
            }
          }
        }
      }
      if(aux=="0") Serial.println("");
    }

    Serial.println("");
    Serial.print("Trama completa: ");Serial.print(trama,BIN);
    //manchester_encode(0b1010000110011011);
    //manchester_encode(0b11010010);
    manchester_encode(trama);
    Serial.println(""); Serial.println("COMANDO ENVIADO");
    wait_nTe(23);
}

void rx_routine(void) {

  Serial.print("El valor de intensidad recibido (%) es: ");Serial.println(rx_data * 100 / 255);
  
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

  // delayMicroseconds(Te>>1);
  
  if (!receiving) {
    if (digitalRead(RX_PIN)) receiving = true;
  }
  else {
    detachInterrupt(digitalPinToInterrupt(RX_PIN));
    delayMicroseconds(Te + (Te>>1));
    
    current_bit = digitalRead(RX_PIN);
    rx_buffer <<= 1;
    bitWrite(rx_buffer, 0, current_bit);
    rx_count++;
  
    if (rx_count == RX_DATA_LEN){
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
