/*
    SmarteEveryting Lion RN2483 Library - sendDataABP_console

    This example shows how to configure and send messages after a ABP Join.
    Please consider the storeConfiguration example can be uset to store
    the required keys and skip the configuration part in curent example.

    created 25 Feb 2017
    by Seve (seve@ioteam.it)

    This example is in the public domain
    https://github.com/axelelettronica/sme-rn2483-library

    More information on RN2483 available here:
    http://www.microchip.com/wwwproducts/en/RN2483

 */
#include <Arduino.h>
#include <rn2483.h>

/*
 * INPUT DATA (for ABP)
 *  1) DevEUI
 *  2) AppEUI
 *  3) AppKey
 *
 * Once these 3 data are written in the network server, this one
 * provide the remaining infos for join the connection
 *
 *  1) DevAddr
 *  2) NwkSKey
 *  3) AppSKey
 *
 *  So, in the end, in the setup phase the module has to set all
 *  these 6 parameters before issuing a "mac join abp"
 *
 */

void setup() {

    errE err;
    loraDbg = false;         // Set to 'true' to enable RN2483 TX/RX traces
    bool storeConfig = true; // Set to 'false' if persistend config already in place
    Serial.begin(115200);

    lora.begin();
    delay(100);

    // Waiting for the USB serial connection
//    while (!Serial) {
//        ;
//    }

    delay(1000);
    Serial.print("FW Version :");
    Serial.println(lora.sysGetVersion());

    /* NOTICE: Current Keys configuration can be skipped if already stored
     *          with the store config Example
     */
    if (storeConfig) {
         // Write HwEUI
        Serial.println("Writing DEV EUI ...");
        lora.macSetDevEUICmd("");

        if (err != RN_OK) {
            Serial.println("\nFailed writing Dev EUI");
        }

        Serial.println("Writing APP EUI ...");
        err = lora.macSetAppEUICmd("");
        if (err != RN_OK) {
            Serial.println("\nFailed writing APP EUI");
        }
        Serial.println("Writing Network Session Key ...");
        err = lora.macSetNtwSessKeyCmd("");
        if (err != RN_OK) {
            Serial.println("\nFailed writing Network Session Key");
        }

        Serial.println("Writing Application Session Key ...");
        err = lora.macSetAppSessKeyCmd("");
        if (err != RN_OK) {
            Serial.println("\nFailed writing APP Session Key");
        }

   //     Serial.println("Writing Application Key ...");
  //      lora.macSetAppKeyCmd("ffffffffffffffffffffffffffff0000");
  //      if (err != RN_OK) {
  //          Serial.println("\nFailed writing raw APP Key");
  //      }

        Serial.println("Writing Device Address ...");
        err = lora.macSetDevAddrCmd("");
        if (err != RN_OK) {
            Serial.println("\nFailed writing Dev Address");
        }

        Serial.println("Setting ADR ON ...");
        err = lora.macSetAdrOn();
        if (err != RN_OK) {
            Serial.println("\nFailed setting ADR");
        }
    }
    /* NOTICE End: Key Configuration */

    Serial.println("Setting Automatic Reply ON ...");
    err = lora.macSetArOn();
    if (err != RN_OK) {
        Serial.println("\nFailed setting automatic reply");
    }

    Serial.println("Setting Trasmission Power to Max ...");
    lora.macPause();
    err = lora.radioSetPwr(14);
    if (err != RN_OK) {
        Serial.println("\nFailed Setting the power to max power");
    }
    lora.macResume();
    delay(5000);

    while (lora.macJoinCmd(ABP)) {
        Serial.println("\nABP JOIN FAILED ");
        delay(5000);
    }
    Serial.println("\nABP Network JOINED! ");

}

uint8_t buff_size = 100;
char buff[100] = {};
uint8_t i = 0;
char c;

void loop() {
    static int loop_cnt = 0;
    static int tx_cnt = 0;
    if (Serial.available()) {
      c = Serial.read();
      Serial.write(c);
      buff[i++] = c;
      if (c == '\n') {
          Serial.print(lora.sendRawCmdAndAnswer(buff));
          i = 0;
          memset(buff, 0, sizeof(buff));
      }
    }

    if (lora.available()) {
        //Unexpected data received from Lora Module;
        Serial.print("\nRx> ");
        Serial.print(lora.read());
    }

    if (!(loop_cnt % 2500)) {
       tx_cnt++;

       // Sending different data at any cycle
       if (tx_cnt == 1) {
           Serial.println("Sending Confirmed String ...");
     //      lora.macTxCmd(String("0123"), 1, TX_ACK); // Confirmed tx
             lora.macTxCmd("1");          // Unconfirmed tx String

       } else if (tx_cnt == 2) {
           Serial.println("Sending Unconfirmed String ...");
           lora.macTxCmd("2");          // Unconfirmed tx String
       } else {
           const char tx_size = 1;
           char tx[tx_size] = {0x37};
           Serial.println("Sending Unconfirmed Buffer ...");
           lora.macTxCmd(tx, tx_size);   // Unconfirmed tx buffer
           tx_cnt = 0;
       }
    }
    loop_cnt++;
    delay(10);
}
