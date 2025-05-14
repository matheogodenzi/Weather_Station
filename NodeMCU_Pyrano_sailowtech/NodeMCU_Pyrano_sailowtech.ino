//Récupération de la valeur W/m2 d'un pyranomètre RS485 en MODBUS


byte message[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A}; // Trame MODBUS pour interroger mon senseur




float light;


void setup() {
  Serial1.begin(4800);
  Serial.begin(9600);
  Serial.println("Test_Pyranomètre_OK 21.11.2024");
}

void myTimerEvent(){
Serial1.write(message, sizeof(message)); //interrogation
    delay(10);
        if(Serial1.available()){ 
           (Serial1.read(), HEX); //lecture des 3 premiers octets de la réponse, quick and dirty, sans intérêts
           (Serial1.read(), HEX);
           (Serial1.read(), HEX);
            light = (256 * (Serial1.read())) + (Serial1.read()); //voilà les deux octets qui m'intéressent
            light = light * 0.207; // conversion µmol/sm2 -> w/m2
              Serial.println(light);
           (Serial1.read(), HEX); //2 octets de checksum, ranapéter
           (Serial1.read(), HEX);
        }

  // You can send any value at any time.
  // Please don't send more that 10 values per second.

    
}

void loop() { 
  
  myTimerEvent();
  delay(1000);
  
}
