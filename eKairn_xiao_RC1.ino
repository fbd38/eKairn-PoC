/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>

// Beacon uses the Manufacturer Specific Data field in the advertising
// packet, which means you must provide a valid Manufacturer ID. Update
// the field below to an appropriate value. For a list of valid IDs see:
// https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers
// 0x004C is Apple
// 0x0822 is Adafruit
// 0x0059 is Nordic
// #define MANUFACTURER_ID 0x0044

// -----------------------------------------------------------------
//    eKairn specific default setup for Orienteering trainning
//                                  by: fbd38
// -----------------------------------------------------------------
#define BLE_NAME_STR "eKairn PoC"
#define EKAIRN_MANU_STR "eKairn community"
#define EKAIRN_MODEL_STR "eKairn micral v01"
#define EKAIRN_FW_VERSION_STR "V01.0.0 07/04/2023"

#define VIK_START (1111)
#define VIK_END (9999)

uint16_t eKairnMarker = 0;  // Default Marker number  --> START
int16_t eKairnTx = 4;       // Default Tx power       --> -8dBm
int16_t eKairnPeriod = 160; // Default Beacon period  --> 100ms

int8_t nRF_TX[] = { -40, -20, -16, -12, -8, -4, 0, +2, +3, +4, +5, +6, +7, +8 };

uint8_t beaconUuid[16] = {
  0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78,
  0x89, 0x9a, 0xab, 0xbc, 0xcd, 0xde, 0xef, 0xf0
};

// A valid Beacon packet consists of the following information:
// UUID, Major, Minor, RSSI @ 1M
BLEBeacon beacon(beaconUuid, 0, 0, -54);
BLEDis bledis;    // device information (not mandatory)
BLEUart bleuart;  // uart over ble

// -----------------------------------------------------------------
//  Vikazimut force start and end beacon convention
//
#define VIK_FORCE_START (eKairnMarker <= 30)
#define VIK_FORCE_END (eKairnMarker >= 256)

uint16_t checkVikMarker(void) {
  if (VIK_FORCE_START) return VIK_START;
  if (VIK_FORCE_END) return VIK_END;
  return eKairnMarker;
}

int8_t checkTxPower(void) {
  if (eKairnTx <= 0) return nRF_TX[0];
  if (eKairnTx >= sizeof(nRF_TX)) return nRF_TX[sizeof(nRF_TX)];
  return nRF_TX[eKairnTx];
}

// XIAO VBAT Measurement
//
// Set the GPIO PIN and ADC configuration for VBat measurements
void SetVBatMeasurement(void) {
  pinMode(PIN_VBAT, INPUT);          //Battery Voltage monitoring pin
  pinMode(VBAT_ENABLE, OUTPUT);      //Enable Battery Voltage monitoring pin
  digitalWrite(VBAT_ENABLE, LOW);    //Enable
  analogReference(AR_INTERNAL_2_4);  //Vref=2.4V
  analogReadResolution(12);          //12bits
}

// Perform a VBat voltage Read
#define VBatAdj (3.970 / 3.892)  // 2% Uncertainty: in specifications
float ReadVbat(void) {
  // Read VBAT voltage
  pinMode(VBAT_ENABLE, OUTPUT);    //Enable Battery Voltage monitoring pin
  digitalWrite(VBAT_ENABLE, LOW);  //Enable
                                   //  digitalWrite(LED_RED, LOW);      // turn the LED on (HIGH is the voltage level)
  delay(200);                      // To be sure the pin voltage is stable
  int VBatMin = 4096;
  int VBatMax = 0;
  int VBat = 0;
  for (int i = 0; i < 10; i++) {
    int v = analogRead(PIN_VBAT);
    VBatMin = min(VBatMin, v);
    VBatMax = max(VBatMax, v);
    VBat += v;
    delay(50);
  }
  VBat -= VBatMin;  // Remove min ADC value
  VBat -= VBatMax;  // Remove max ADC value
  VBat /= 8;
  pinMode(VBAT_ENABLE, INPUT);  //Disable Battery Voltage monitoring pin
                                //  digitalWrite(LED_RED, HIGH);  // turn the LED off by making the voltage LOW
  float Vbattery = ((510e3 + 1000e3) / 510e3) * 2.4 * VBat / 4096;
  return (Vbattery * VBatAdj);
}
// Convert VBat in %
// [TODO] use a 10 points table from measurement of the specific battery
//        to convert voltage une Battery percent.
//        Require a specific measurent campain
//        (it could be more interesting to express un remaing days of battery)
int ConvertVBatInPercent(float v) {
  long int iv = (long int)(v * 1000.0);
  long int val = map(iv, 3100, 4100, 0, 100);
  return ((int)val);
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_BLUE, LOW);    // turn the LED on (HIGH is the voltage level)
  delay(400);                     // wait for a second
  digitalWrite(LED_BLUE, HIGH);   // turn the LED off by making the voltage LOW
  delay(100);                     // wait for a second
  digitalWrite(LED_GREEN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(400);                     // wait for a second
  digitalWrite(LED_GREEN, HIGH);  // turn the LED off by making the voltage LOW
  delay(100);                     // wait for a second

  Serial.begin(115200);
  // Uncomment to blocking wait for Serial connection

  Serial.println("-----------------");
  Serial.println("--- eKain PoC ---");
  Serial.println("-----------------");
  Serial.println(EKAIRN_FW_VERSION_STR);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  // off Blue LED for lowest power consumption
  Bluefruit.autoConnLed(false);

  Bluefruit.begin();

  Bluefruit.setTxPower(checkTxPower());  // Check bluefruit.h for supported values
  Bluefruit.setName(BLE_NAME_STR);
  // Manufacturer ID is required for Manufacturer Specific Data
  bledis.setManufacturer(EKAIRN_MANU_STR);
  bledis.setModel(EKAIRN_MODEL_STR);
  bledis.setFirmwareRev(EKAIRN_FW_VERSION_STR);
  beacon.setMajorMinor(0, checkVikMarker());  // Vikazimut Marker value

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Enable the VBat measurement
  SetVBatMeasurement();
  float IniVBat = ReadVbat();
  int ib = 10 - (ConvertVBatInPercent(IniVBat) / 10);
  Serial.print("VBat = ");
  Serial.print(IniVBat, 2);
  Serial.println(" V");
  // at power up perform RED flashes according with battery remaining content
  if (ib > 0) {
    for (int ii = 0; ii < ib; ii++) {
      digitalWrite(LED_RED, LOW);   // turn the LED on (HIGH is the voltage level)
      delay(500);                   // wait for a second
      digitalWrite(LED_RED, HIGH);  // turn the LED off by making the voltage LOW
      delay(500);                   // wait for a second
    }
  }
  // Configure and Start BLE Uart Service
  bleuart.begin();
  // Setup the advertising packet
  startAdv();

  // Suspend Loop() to save power, since we didn't have any code there
  suspendLoop();
}

void startAdv(void) {
  // Advertising packet
  // Set the beacon payload using the BLEBeacon class populated
  // earlier in this example
  Bluefruit.Advertising.setBeacon(beacon);
  Bluefruit.Advertising.addTxPower();

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * Apple Beacon specs
   * - Type: Non connectable, undirected
   * - Fixed interval: 100 ms -> fast = slow = 100 ms
   */
  //Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_ADV_NONCONN_IND);
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(eKairnPeriod, eKairnPeriod);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);                       // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                                 // 0 = Don't stop advertising after n seconds
}

/**
 * Callback invoked when a connection is establisehd
 * @param conn_handle connection where this event happens
 */
void connect_callback(uint16_t conn_handle) {

  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));
  Serial.print("Connected to ");
  Serial.println(central_name);

  // Set BLUE LED
  digitalWrite(LED_RED, HIGH);    // Clear RED LED
  digitalWrite(LED_GREEN, HIGH);  // Clear GREEN LED
  digitalWrite(LED_BLUE, LOW);    // turn the BLUE LED ON
                                  //
  resumeLoop();
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);
  if (reason != 0x13) {
    // Error disconnect
    digitalWrite(LED_RED, LOW);  // turn the LED on (HIGH is the voltage level)
  } else {
    // Stop Advertising and restart with new Marker value
    Bluefruit.Advertising.stop();  // Stop advertising after n seconds
    delay(200);                    // To be sure it ends
    // Assign new values
    Bluefruit.setTxPower(checkTxPower());
    beacon.setMajorMinor(0, checkVikMarker());
    // Resartr the eBeacon
    Bluefruit.Advertising.setBeacon(beacon);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.setInterval(eKairnPeriod, eKairnPeriod);  // in unit of 0.625 ms
    Bluefruit.Advertising.start(0);                                 // 0 = Don't stop advertising after n seconds
    // Clear BLUE LED
    digitalWrite(LED_BLUE, HIGH);  // turn the BLUE LED OFF
  }
  // Then we need no more the main loop, so we goe in low power state
  suspendLoop();
}

void help() {
  bleuart.write("eKairn list of command \n", 24);
  bleuart.write("?: Help (this list)    \n", 24);
  bleuart.write("I: Get Informations    \n", 24);
  bleuart.write("S: Get Setup           \n", 24);
  bleuart.write("Mxyz: Set Marker       \n", 24);
  bleuart.write("Txy: Set Tx power      \n", 24);
  bleuart.write("Pxyzt: Set Period      \n", 24);
  bleuart.write("V: Get Battery Voltage \n", 24);
  bleuart.write("G, g: Manage Green LED \n", 24);
  bleuart.write("R, r: Manage Red LED   \n", 24);
}

void loop() {
  // loop is already suspended, CPU will not run loop() at all
  // Forward from BLEUART to HW Serial
  while (bleuart.available()) {
    uint8_t ch;
    ch = (uint8_t)bleuart.read();
    Serial.print("Rx command = [");
    Serial.print((char)ch);
    Serial.println("]");
    // Parser
    // for debug purpose
    if (ch == 'V') {
      // Perform a VBat acquisition
      float IniVBat = ReadVbat();
      Serial.print("VBat = ");
      char cvbat[10];
      sprintf(cvbat, "%4.2f", IniVBat);
      Serial.print(cvbat);
      Serial.println(" V");
      bleuart.write(cvbat, 4);
    }
    if (ch == 'I') {
      bleuart.write(EKAIRN_MODEL_STR, sizeof(EKAIRN_MODEL_STR));
      bleuart.write(EKAIRN_FW_VERSION_STR, sizeof(EKAIRN_FW_VERSION_STR));
    }
    if (ch == 'S') {
      // Write Marker Number
      char st[16] = "";
      sprintf(st, "Marker = %3d", eKairnMarker);
      if (VIK_FORCE_START) sprintf(st, "Marker = START\n");
      if (VIK_FORCE_END) sprintf(st, "Marker = END\n");
      bleuart.write(st, sizeof(st));
      // Write current Tx  Power
      char st2[16] = "";
      sprintf(st2, "Tx = %2d dBm", nRF_TX[eKairnTx]);
      bleuart.write(st2, sizeof(st2));
      // Write current Period
      char st3[20] = "";
      sprintf(st3, "Period = %5.0f ms\n", ((float) eKairnPeriod)*0.62);
      bleuart.write(st3, sizeof(st3));
    }
    if (ch == 'T') {
      char st[16] = "";
      bleuart.read(st, 2);
      Serial.print("  (");
      Serial.print(st);
      Serial.print(") = ");
      int16_t nval;
      sscanf(st, "%2d", &nval);
      Serial.println(nval);
      if ((nval < 0) || (nval > 13)) {
        const char err[] = "ERROR Invalid Value, must be [0..13]";
        Serial.println(err);
        bleuart.write(err, sizeof(err));
      } else {
        eKairnTx = nval;
      }
    }
    if (ch == 'M') {
      char st[8] = "";
      bleuart.read(st, 3);
      Serial.print("  (");
      Serial.print(st);
      Serial.print(") = ");
      uint16_t nval;
      sscanf(st, "%3d", &nval);
      Serial.println(nval);
      if ((nval < 0) || (nval > 999)) {
        const char err[] = "ERROR Invalid Value, must be [0..999]";
        Serial.println(err);
        bleuart.write(err, sizeof(err));
      } else {
        eKairnMarker = nval;
      }
    }
    if (ch == 'P') {
      char st[8] = "";
      bleuart.read(st, 4);
      Serial.print("  (");
      Serial.print(st);
      Serial.print(") = ");
      int16_t nval;
      sscanf(st, "%4d", &nval);
      Serial.println(nval);
      if ((nval < 100) || (nval > 1600)) {
        const char err[] = "ERROR Invalid Value, must be [100..1600]";
        Serial.println(err);
        bleuart.write(err, sizeof(err));
      } else {
        eKairnPeriod = nval;
      }
    }
    if (ch == 'G') digitalWrite(LED_GREEN, LOW);   // turn the GREEN LED ON
    if (ch == 'G') digitalWrite(LED_GREEN, LOW);   // turn the GREEN LED ON
    if (ch == 'g') digitalWrite(LED_GREEN, HIGH);  // turn the GREEN LED OFF
    if (ch == 'R') digitalWrite(LED_RED, LOW);     // turn the RED LED ON
    if (ch == 'r') digitalWrite(LED_RED, HIGH);    // turn the RED LED OFF
    if (ch == '?') help();
  }
}
