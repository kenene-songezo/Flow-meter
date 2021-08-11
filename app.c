/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"


//edited from the thermometer client project
#include <stdbool.h>
#include <math.h>
#include "sl_app_log.h"
#ifdef SL_COMPONENT_CATALOG_PRESENT
#include "sl_component_catalog.h"
#endif // SL_COMPONENT_CATALOG_PRESENT
#ifdef SL_CATALOG_CLI_PRESENT
#include "sl_cli.h"
#endif // SL_CATALOG_CLI_PRESENT
#include "app.h"
#include "httpget.h"

// connection parameters
#define CONN_INTERVAL_MIN             80   //100ms
#define CONN_INTERVAL_MAX             80   //100ms
#define CONN_SLAVE_LATENCY            0    //no latency
#define CONN_TIMEOUT                  100  //1000ms
#define CONN_MIN_CE_LENGTH            0
#define CONN_MAX_CE_LENGTH            0xffff

#define SCAN_INTERVAL                 16   //10ms
#define SCAN_WINDOW                   16   //10ms
#define SCAN_PASSIVE                  0
#define SCAN_ACTIVE                   1

#define COUNTER_WINDOW                5

#if SL_BT_CONFIG_MAX_CONNECTIONS < 1
  #error At least 1 connection has to be enabled!
#endif

// Macro to translate the Flags to Celsius (C) or Fahrenheit (F). Flags is the first byte of the
// Temperature Measurement characteristic value according to the Bluetooth SIG
#define translate_flags_to_temperature_unit(flags) (((flags) & 1) ? UNIT_FAHRENHEIT : UNIT_CELSIUS)

typedef enum {
  scanning,
  opening,
  discover_services,
  discover_characteristics,
  enable_indication,
  running
} conn_state_t;

conn_state_t conn_state;

// read the count from the data received
static char* read_data_count(uint8_t *count_d);
// verify that the data being read is from the correct transmitting device
void verify_data_packet(uint8_t *data);

static bd_addr* read_and_cache_bluetooth_address (uint8_t *address_type_out);

void create_url (int count_int);

//// State of the connection under establishment
//static conn_state_t conn_state;
//
//static void print_bluetooth_address(void);
//// Print RSSI and temperature values
//static void print_values(void);
// reverse array
void reverseArray(uint8_t arr[], int start, int end);
//end of edited from the thermometer client project


uint8_t identifier[] = {0x54, 0x58, 0x33};
uint8_t RV[] = {0x52, 0x56};
bool correct_packet = false;
char count_data[4];
char *countdata_ptr = &count_data[0];
float floatCount = 0;
char urltosendtoserver[];

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  init_GPIO();
  //Turn on modem
  on_PWRKEY();
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////



}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);


      ////LISTEN FOR ADVERTISEMENTS////

      sc = sl_bt_scanner_set_mode (gap_1m_phy, SCAN_ACTIVE);

      if (sc != SL_STATUS_OK)
        {
          printf ("[E: 0x%04x] Failed to set discovery type\n", (int) sc);
          return;
        }

      // Set scan interval and scan window
      sc = sl_bt_scanner_set_timing (gap_1m_phy, SCAN_INTERVAL, SCAN_WINDOW);
      if (sc != SL_STATUS_OK)
        {
          printf ("[E: 0x%04x] Failed to set discovery type\n", (int) sc);
          return;
        }
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);

      // Set the default connection parameters for subsequent connections
      sc = sl_bt_connection_set_default_parameters (CONN_INTERVAL_MIN,
                                                    CONN_INTERVAL_MAX,
                                                    CONN_SLAVE_LATENCY,
                                                    CONN_TIMEOUT,
                                                    CONN_MIN_CE_LENGTH,
                                                    CONN_MAX_CE_LENGTH);
      sl_app_assert(sc == SL_STATUS_OK,
                      "[E: 0x%04x] Failed to set connection timing\n",
                      (int)sc);

      // Start scanning - looking for thermometer devices
      sc = sl_bt_scanner_start (gap_1m_phy, scanner_discover_observation);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      conn_state = scanning;
      printf ("BLE Scanning started successfully.\n", 0);
      printf("--------------------------------------\n");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);

      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    case sl_bt_evt_scanner_scan_report_id:

      verify_data_packet(&(evt->data.evt_scanner_scan_report.data.data[0]));
      sl_app_assert(sc == SL_STATUS_OK,
                          "[E: 0x%04x] Failed to start advertising\n",
                          (int)sc);

      if (correct_packet) { // Scannable undirected advertising
        printf ("******** item found:"); // Print BLE address
        int8_t rssi = evt->data.evt_scanner_scan_report.rssi;


        bd_addr a = evt->data.evt_scanner_scan_report.address;
        reverseArray(a.addr, 0, 5);


        uint8array data = evt->data.evt_scanner_scan_report.data;
        printf(", rssi: %i, len %i ********\n",rssi, data);
        // Get advertisement data


        countdata_ptr = read_data_count(&(evt->data.evt_scanner_scan_report.data.data[18]));
        floatCount = (int)count_data[0];
        create_url(floatCount);
        uint8_t *url_ptr = (uint8_t*) urltosendtoserver;
        getRequest(url_ptr);


      }
      break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}


void reverseArray(uint8_t arr[], int start, int end)
{
    while (start < end)
    {
        int temp = arr[start];
        arr[start] = arr[end];
        arr[end] = temp;
        start++;
        end--;
    }
}


void verify_data_packet(uint8_t *data){


  int data_index = 0;     // the identifier for this data starts at data position 5 (startposition = 0)

  // identifier is 24 bits long
  if (memcmp(identifier,&data[data_index],3)==0) {
    data_index += 5;
    if (memcmp(RV,&data[data_index],2)==0) {
        correct_packet = true;
    }
  }else{
      printf("..not a match..");
  }


}

static char* read_data_count(uint8_t *count_d){
  //the meaningful part of the data (count) starts at position 18
  memcpy(count_data, count_d, 4);
  int start = 0;
  int end = 3;
  while (start < end)
  {
    int temp = count_data[start];
    count_data[start] = count_data[end];
    count_data[end] = temp;
    start++;
    end--;
  }
  return count_data;
  }

void create_url(int count_int)
{
  uint8_t address_type;
  bd_addr *address = read_and_cache_bluetooth_address (&address_type);

  sprintf(urltosendtoserver,"http://41.78.128.25/httpds?%02X:%02X:%02X:%02X:%02X:%02X=%d",
          address->addr[5],address->addr[4],
          address->addr[3], address->addr[2],
          address->addr[1], address->addr[0], count_int);
}

static bd_addr* read_and_cache_bluetooth_address (uint8_t *address_type_out)
{
  static bd_addr address;
  static uint8_t address_type;
  static bool cached = false;

  if (!cached)
    {
      sl_status_t sc = sl_bt_system_get_identity_address (&address,
                                                          &address_type);
      if (sc != SL_STATUS_OK)
        {
          printf ("[E: 0x%04x] Failed to get Bluetooth address\n", (int) sc);
          return NULL;
        }

      cached = true;
    }

  if (address_type_out)
    {
      *address_type_out = address_type;
    }

  return &address;
}
