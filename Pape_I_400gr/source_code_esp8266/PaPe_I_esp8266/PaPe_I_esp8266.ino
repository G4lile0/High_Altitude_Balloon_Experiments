/*******************************************************************************
   GPS TTNMapper by Akirasan 2018
    Nodo LoRaWAN con GPS para realizar mapeo sobre TTNMapper

   twitter: @akirasan

   Parte del código es grácias al ejemplo del envío de un paquete "Hello, world!"
  por LoRaWAN utilizando autenticación ABP, realizado por:
     Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
*******************************************************************************/

#define GPS_ON        // GPS conectado. Si no está definido se envían datos nulos. Para uso con TTNMAPPER App móvil
#define DEBUG_ON      // Info por Serial
//#define MODO_TEST     // Usa utiliza el canal 0 para transmitir ---- SOLO TEST
#define BAUD_SERIAL 115200


#include "ESP8266WiFi.h"


//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include <EEPROM.h>

// Data wire is plugged into port 2 on the Arduino
//#define ONE_WIRE_BUS A2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
//DallasTemperature sensors(&oneWire);



#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>


#ifdef GPS_ON
#include <SoftwareSerial.h>
#include <MicroNMEA.h>
#define RX_GPS D3
#define TX_GPS D4
#define BAUD_GPS 9600
byte gps_set_sucess = 0;

// Refer to serial devices by use
HardwareSerial& console = Serial;
SoftwareSerial gps(RX_GPS, TX_GPS);

char nmeaBuffer[500];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

#endif GPS_ON


//*************************************
//***** CONFIGURACION LORAWAN *********

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { };


// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = {  };




// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x ; // <-- Change this address for every node!
//**************************************



// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }


static uint8_t payload[12];
static osjob_t sendjob;

// Planificacion del envio de paquetes LoRa. El intervalo es en segundos
byte unsigned TX_INTERVAL = 20;
byte n_packets = 0;
int dr = DR_SF7;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = D0,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {D1, D2, LMIC_UNUSED_PIN},
};


//************ Gestion de enventos del módulo LoRa RFM95W
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("BEACON"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("BEA_MIS"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("BEATRAC"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE inc Rx"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Rec. ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), enviar_datos_GPS);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}



// ----------------------------
// Captura y envío de datos GPS  -----------------------------
// ----------------------------
void enviar_datos_GPS(osjob_t* j) {
  boolean datos_gps = false;
  char c_gps;

  // GPS --- lat/lon
  long latitude_long = 0;
  long longitude_long = 0;

  // GPS --- Altitud
  long alt = 0;
  word altitude = 0;

  // GPS --- HDOP (horizontal dilution of precision)
  byte hdop = 0;

  // GPS --- Numero de satélites
//  byte sats = 0;
#ifdef DEBUG_ON
  Serial.println(F("JOB para el datos GPS"));
#endif DEBUG_ON

#ifdef GPS_ON
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
//    Serial.println(F("OP_TXRXPEND"));
//    Serial.println(OP_TXRXPEND);
//    Serial.println(LMIC.opmode);
//    Serial.println("---------------");
  } else {
    // Prepare upstream data transmission at the next possible time.
    // Empty input buffer
    while (gps.available()) {
      gps.read();
      wdt_reset();
    }
    wdt_reset();
    delay(500);
    nmea.clear();

    unsigned long previousMillis = millis(); 
//    Serial.println("LEEMOS DATOS NUEVOS GPS");
    n_packets++;
//    Serial.println("NPak");
//    Serial.println(n_packets);
//    Serial.println(EEPROM.read(1));
//    Serial.println("SF");
 //   Serial.println(dr);

    

        while (!(nmea.isValid())) {

    //  Cada 5 paquetes mandamos la temperatura y al 9 cambiamos el SF 

    if (n_packets>8) {
        n_packets=0;
        dr--;
        if (dr==-1)  dr = 5 ;
        inicializar_LoRa();
        if (dr>3)  TX_INTERVAL=20;    
        if (dr==1) TX_INTERVAL=57 ;
        if (dr==0) TX_INTERVAL=116 ;

          
      
        
        break;        
        };   

    if (n_packets>4) {
        break;        
        };    //every 5 packets force to send temperature and offline positions

      
      while (gps.available()) {
        c_gps = gps.read();
#ifdef DEBUG_ON
        console.print(c_gps);
#endif DEBUG_ON
        nmea.process(c_gps);
      }
      wdt_reset();
     if ((millis()-previousMillis)>7000) break;     // after 7sec of not valid GPS break the loop and send temperature
    }
#ifdef DEBUG_ON
    Serial.println(F("================================ NAV SYSTEM: "));
    Serial.println(nmea.getNavSystem());
#endif DEBUG_ON

    if (nmea.isValid()) {

      // ----- MicroNMEA devuelve la info sin decimales
      // por lo tanto nos ahorramos el siguiente paso de pasarlo a entero
      // multiplicando por 10000000
      // Dejo el código comentado por si alguien lo necesita
      //----------------------------
      //float latitude = nmea.getLatitude();
      //float longitude = nmea.getLongitude();
      //long latitude_long = latitude * 10000000;
      //long longitude_long = longitude * 10000000;

      // GPS --- Latitud + Longitud
      latitude_long = nmea.getLatitude();
      longitude_long = nmea.getLongitude();

      // GPS --- Altitud
      altitude = 0;
      if (nmea.getAltitude(alt)) {   // Altitud correcta??
        altitude = alt / 1000;
      }

      // GPS --- HDOP (horizontal dilution of precision)
      hdop = nmea.getHDOP();

      // GPS --- Numero de satélites
//      sats = nmea.getNumSatellites();

      // -----
      // ----- Conversión de datos recogidos a PAYLOAD LORAWAN

      // [0..3] 4 bytes LATITUDE
      payload[0] = (byte) ((latitude_long & 0xFF000000) >> 24 );
      payload[1] = (byte) ((latitude_long & 0x00FF0000) >> 16 );
      payload[2] = (byte) ((latitude_long & 0x0000FF00) >> 8 );
      payload[3] = (byte) ((latitude_long & 0X000000FF));

      // [4..7] 4 bytes LONGITUDE
      payload[4] = (byte) ((longitude_long & 0xFF000000) >> 24 );
      payload[5] = (byte) ((longitude_long & 0x00FF0000) >> 16 );
      payload[6] = (byte) ((longitude_long & 0x0000FF00) >> 8 );
      payload[7] = (byte) ((longitude_long & 0X000000FF));

      // [8..9] 2 bytes ALTITUDE
      payload[8] = (byte) ((altitude & 0xFF00) >> 8);
      payload[9] = (byte) (altitude & 0x00FF);

      // [10] 1 byte HDOP
      payload[10] = (byte) hdop;  //HDOP


    // sistema "cutre" para alamacenar los datos del vuelo en la eeprom
    // latitud, longitud 


    Serial.println("day");
    Serial.println(int(nmea.getDay()));
//    Serial.println(EEPROM.read(int(nmea.getDay())*20) );

    
 
   // if (EEPROM.read(int(nmea.getDay())*20)) {
   //     for (int address = (int(nmea.getDay())*20) ; address < (int(nmea.getDay())*20)+10; address++) {
   //        Serial.println(address);
   //     }
        //

            
//         };
  



      // [11] 1 byte SATELLITES
//      payload[11] = (byte) sats; //Satelites

#ifdef DEBUG_ON
      Serial.println(F("datos GPS recogidos ------"));
      Serial.print(F("latitude: ")); Serial.println(latitude_long);
      Serial.print(F("longitude: ")); Serial.println(longitude_long);
      Serial.print(F("altitud: ")); Serial.println(altitude);
      Serial.print(F("hdop: ")); Serial.println(hdop);
//      Serial.print(F("sats: ")); Serial.println(sats);
      Serial.println(F("-----------------"));
#endif DEBUG_ON

      datos_gps = true;
    }
    else {

#ifdef DEBUG_ON
      Serial.println(F("NMEA no VALIDO"));
#endif DEBUG_ON
    }
    // FIN tratamiento datos GPS

    // --- Revisamos si hemos recogido datos nuevos del GPS
    if (datos_gps) {
#ifdef DEBUG_ON
      Serial.print(F("...envio datos del GPS LoRaWAN. Tamaño paquete: "));
      Serial.println(sizeof(payload) - 1);
#endif DEBUG_ON

      // Envío de datos por LoRaWAN
      LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
   //LMIC_setTxData2(2, payload, 1, 0);

    
    
    }  else 
    
    {
//       Serial.print("Requesting temperatures...");
//       sensors.requestTemperatures(); // Send the command to get temperatures
//       Serial.println("DONE");
//       Serial.print("Temperature for the device 1 (index 0) is: ");
//       Serial.println(sensors.getTempCByIndex(0));  
       // envio de datos sin GPS


        // enviamos temperatura o info de viaje offline 
        // uso la variable gps_set_sucess por reducir el uso de memoria
        //
  


        if (n_packets<7) {
      
//       payload[0] = (byte) round(127+sensors.getTempCByIndex(0));
       LMIC_setTxData2(2, payload, 1, 0);

        } else

       {

        //
      
      }



      
      }
     

    
  }
  // Reprogramamos el siguiente envío
  os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), enviar_datos_GPS);
#endif GPS_ON

#ifndef GPS_ON
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
//    Serial.println(F("OP_TXRXPEND"));
//    Serial.println(OP_TXRXPEND);
//    Serial.println(LMIC.opmode);
//    Serial.println("---------------");
  } else {
    // Prepare upstream data transmission at the next possible time.
#ifdef DEBUG_ON
    Serial.println(F("TTNMapper app mov"));
#endif DEBUG_ON
//    LMIC_setTxData2(1, payload, sizeof(payload) - 1, 0);
    LMIC_setTxData2(2, payload, 1, 0);
    // Reprogramamos el siguiente envío
    os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), enviar_datos_GPS);
  }
#endif GPS_ON

}

// ---CODE FROM
//https://www.thethingsnetwork.org/forum/t/can-lmic-1-6-be-set-up-to-use-a-single-channel-and-sf/5207/11
// Disables all channels, except for the one defined above, and sets the
// data rate (SF). This only affects uplinks; for downlinks the default
// channels or the configuration from the OTAA Join Accept are used.
//
// Not LoRaWAN compliant; FOR TESTING ONLY!
//
void forceTxSingleChannelDr(int channel, int dr) {
  for (int i = 0; i < 9; i++) { // For EU; for US use i<71
    if (i != channel) {
      LMIC_disableChannel(i);
    }
  }
  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(dr, 14);
}
// ---CODE




void inicializar_LoRa() {
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Set static session parameters. Instead of dynamically establishing a session
  // by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
  // On AVR, these values are stored in flash and only copied to RAM
  // once. Copy them to a temporary buffer here, LMIC_setSession will
  // copy them into a buffer of its own again.
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
#else
  // If not running an AVR with PROGMEM, just use the arrays directly
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
  // Set up the channels used by the Things Network, which corresponds
  // to the defaults of most gateways. Without this, only three base
  // channels from the LoRaWAN specification are used, which certainly
  // works, so it is good for debugging, but can overload those
  // frequencies, so be sure to configure the full frequency range of
  // your network here (unless your network autoconfigures them).
  // Setting up channels should happen after LMIC_setSession, as that
  // configures the minimal channel set.
  // NA-US channels 0-71 are configured automatically

  LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
  LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
  LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band

  // TTN defines an additional channel at 869.525Mhz using SF9 for class B
  // devices' ping slots. LMIC does not have an easy way to define set this
  // frequency and support for class B is spotty and untested, so this
  // frequency is not configured here.
#elif defined(CFG_us915)
  // NA-US channels 0-71 are configured automatically
  // but only one group of 8 should (a subband) should be active
  // TTN recommends the second sub band, 1 in a zero based count.
  // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
  LMIC_selectSubBand(1);
#endif

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(dr, 14);


#ifdef MODO_TEST
  // --------------------
  // Desactivamos todos los canales menos el canal 0 --> SOLO PARA TESTING!!!
  // Define the single channel and data rate (SF) to use
  // channel = 0   /    DR_SF7
  forceTxSingleChannelDr(0, dr);
  // ------------------
#endif MODO_TEST

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Iniciamos job de envio datos GPS
  enviar_datos_GPS(&sendjob);
}


void setup() {

  WiFi.forceSleepBegin();                  // turn off ESP8266 RF
  delay(1);     

  Serial.begin(BAUD_SERIAL);
//  Serial.println(F("Starting"));
//  Serial.begin(115200);
//  Serial.println("Dallas Temperature IC Control Library Demo");
 // pinMode(A3, OUTPUT);
 // digitalWrite(A3, LOW);  
//  delay(200);
//  pinMode(A1, OUTPUT);
//  digitalWrite(A1, HIGH);  
  // Start up the library
//  sensors.begin();


#ifdef GPS_ON
  gps.begin(BAUD_GPS);        // gps



  // very important info to avoid that GPS stop at 12Km
  //  https://ukhas.org.uk/guides:ublox6
  //  https://ava.upuaut.net/?p=750


    // START OUR SERIAL DEBUG PORT
 // Serial.begin(9600);
//  Serial.println("GPS Level Convertor Board Test Script");
//  Serial.println("03/06/2012 2E0UPU");
//  Serial.println("Initialising....");
  //
  // THE FOLLOWING COMMAND SWITCHES MODULE TO 4800 BAUD
  // THEN SWITCHES THE SOFTWARE SERIAL TO 4,800 BAUD
  //
  //gps.print("$PUBX,41,1,0007,0003,4800,0*13\r\n"); 
  //gps.begin(4800);
  //gps.flush();
/*
 * 
 
  //  THIS COMMAND SETS FLIGHT MODE AND CONFIRMS IT 
  Serial.println("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC };
  while(!gps_set_sucess)
  {
    sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
    gps_set_sucess=getUBX_ACK(setNav);
  }
  gps_set_sucess=0;
 
  // THE FOLLOWING COMMANDS DO WHAT THE $PUBX ONES DO BUT WITH CONFIRMATION
  // UNCOMMENT AS NEEDED
  /*
  Serial.println("Switching off NMEA GLL: ");
   uint8_t setGLL[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B                   };
   while(!gps_set_sucess)
   {    
   sendUBX(setGLL, sizeof(setGLL)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGLL);
   }
   gps_set_sucess=0;
   Serial.println("Switching off NMEA GSA: ");
   uint8_t setGSA[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32                   };
   while(!gps_set_sucess)
   {  
   sendUBX(setGSA, sizeof(setGSA)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGSA);
   }
   gps_set_sucess=0;
   Serial.println("Switching off NMEA GSV: ");
   uint8_t setGSV[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39                   };
   while(!gps_set_sucess)
   {
   sendUBX(setGSV, sizeof(setGSV)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setGSV);
   }
   gps_set_sucess=0;
   Serial.print("Switching off NMEA RMC: ");
   uint8_t setRMC[] = { 
   0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40                   };
   while(!gps_set_sucess)
   {
   sendUBX(setRMC, sizeof(setRMC)/sizeof(uint8_t));
   gps_set_sucess=getUBX_ACK(setRMC);
   }
   */

   



#endif GPS_ON

  inicializar_LoRa();
}



void loop() {
  wdt_reset();
  os_runloop_once();
}







// Funciones para reprogramar el GPS en modo airborne para que funcione por encima de los 12Km

#ifdef GPS_ON
// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gps.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  gps.println();
}
 
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(F(" * Reading ACK response: "));
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;  // header
  ackPacket[1] = 0x62;  // header
  ackPacket[2] = 0x05;  // class
  ackPacket[3] = 0x01;  // id
  ackPacket[4] = 0x02;  // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];  // ACK class
  ackPacket[7] = MSG[3];  // ACK id
  ackPacket[8] = 0;   // CK_A
  ackPacket[9] = 0;   // CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(F(" (SUCCESS!)"));
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(F(" (FAILED!)"));
      return false;
    }
 
    // Make sure data is available to read
    if (gps.available()) {
      b = gps.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;  // Reset and look again, invalid order
      }
 
    }
  }
}
#endif GPS_ON










