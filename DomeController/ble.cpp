/* 
  ble features 
*/

#include <bluefruit.h>
#include "ble.h"
#include "shutterStatus.h"

BLEClientDis clientDis;    // device information client
BLEClientUart clientUart;  // bleuart client


/**
 * Callback invoked when scanner pick up an advertising data
 * @param report Structural advertising data
 */
void scan_callback(ble_gap_evt_adv_report_t* report) {
  uint8_t target_adress1[6] = { 0x62, 0xB0, 0xEB, 0x66, 0x65, 0xFA };  // Chelles Obs
  uint8_t target_adress2[6] = { 0xDB, 0xB6, 0x18, 0x93, 0x54, 0x94 };  // Rigado

  // Check if advertising contain BleUart service
  if (Bluefruit.Scanner.checkReportForService(report, clientUart)) {
    Serial.print("BLE UART service detected @ MAC address = ");
    // MAC is in little endian --> print reverse
    Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
    Serial.print("\n");

    //if ( (memcmp(report->peer_addr.addr,target_adress1,6) == 0) || (memcmp(report->peer_addr.addr,target_adress2,6) == 0))
    if ((memcmp(report->peer_addr.addr, target_adress1, 6) == 0)) {
      Serial.println("MAC adress match.");
      // Connect to device with bleuart service in advertising
      Serial.print("Connecting ... ");
      Bluefruit.Central.connect(report);
    } else {
      Serial.print("MAC adress differt from target MAC =:  ");
      Serial.printBufferReverse(target_adress1, 6, ':');
      Serial.println();
      Bluefruit.Scanner.resume();
    }
  } else {
    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();
  }
}

/**
 * Callback invoked when an connection is established
 * @param conn_handle
 */
void connect_callback(uint16_t conn_handle) {

  //Serial.println("Connected");
  //Serial.print("Dicovering Device Information ... ");
  if (clientDis.discover(conn_handle)) {
    //Serial.println("Found it");
    char buffer[32 + 1];

    // read and print out Manufacturer
    memset(buffer, 0, sizeof(buffer));
    if (clientDis.getManufacturer(buffer, sizeof(buffer))) {
      //Serial.print("Manufacturer: ");
      //Serial.println(buffer);
    }

    // read and print out Model Number
    memset(buffer, 0, sizeof(buffer));
    if (clientDis.getModel(buffer, sizeof(buffer))) {
      //Serial.print("Model: ");
      //Serial.println(buffer);
    }

    //Serial.println();
  } else {
    //Serial.println("Found NONE");
  }

  Serial.print("Discovering BLE Uart Service ... ");
  if (clientUart.discover(conn_handle)) {
    //Serial.println("Found it");

    //Serial.println("Enable TXD's notify");
    clientUart.enableTXD();

    //Serial.println("Ready to receive from slave dome peripheral");
  } else {
    //Serial.println("Found NONE");

    // disconnect since we couldn't find bleuart service
    Bluefruit.disconnect(conn_handle);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  //Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
}

/**
 * Callback invoked when uart received data
 * @param uart_svc Reference object to the service where the data 
 * arrived. In this example it is clientUart
 */
void bleuart_rx_callback(BLEClientUart& clientUart) {
  uint8_t receive_buffer[100];
  size_t nb_bytes;

  nb_bytes = clientUart.read(receive_buffer, 100);
  if (nb_bytes >= 2) {
    char header = (char)receive_buffer[0];
    switch (header) {
      case 's':
        {
          char c;
          // <'s'> <ShutterStatus>
          c = (char)receive_buffer[1];
          if ((c >= '0') && (c <= ('0' + SS_ERROR))) {
            shutterStatus_SetStatus((ShutterStatus)(c - '0'));
          }
          break;
        }

      case 'v':
        {
          int adc = 0;
          if (nb_bytes >= 5) {
            // <'v'><4 chars length for adc value>
            receive_buffer[5] = 0;
            adc = atoi((const char*)(receive_buffer + 1));
            shutterStatus_SetVbat(adc);
          }
          break;
        }

      default:
        {
          break;
        }
    }
  }
}


void ble_setup(void) {
  // Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4); 

  Bluefruit.setName("Dome Controler Main board");

  // Configure DIS client
  clientDis.begin();

  // Init BLE Central Uart Serivce
  clientUart.begin();
  clientUart.setRxCallback(bleuart_rx_callback);

  // Increase Blink rate to different from PrPh advertising mode
  Bluefruit.setConnLedInterval(250);

  // Callbacks for Central
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);

  /* Start Central Scanning
   * - Enable auto scan if disconnected
   * - Interval = 100 ms, window = 80 ms
   * - Don't use active scan
   * - Start(timeout) with timeout = 0 will scan forever (until connected)
   */
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);  // in unit of 0.625 ms
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);  // // 0 = Don't stop scanning after n seconds
}