// header of module
#include "imsl_network.h"

// global C++ headers
//#include <ETH.h>
//#include <WiFi.h>

// local C++ headers
#include "mrlogger.h"

// configuration headers
//#include "network_config.h" // definition of network and hardware parameters

//namespace imsl {
static bool wifiConnected = false;
static bool ethConnected = false;

bool networkIsConnected() {
    return (wifiConnected || ethConnected);
}

//void WiFiEvent(WiFiEvent_t event) {
//  switch (event) {
//// --------------- Ethernet ------------------
//    case SYSTEM_EVENT_ETH_START:
//      log_message(log_info, "ETH Started");
//      //set eth hostname here
//      ETH.setHostname(esp32Hostname);
//      break;
//    case SYSTEM_EVENT_ETH_CONNECTED:
//      log_message(log_info, "ETH Connected");
//      break;
//    case SYSTEM_EVENT_ETH_GOT_IP:
//      log_message(log_info, "ETH MAC: %s, IPv4: %s, %sMbps",
//                            String(ETH.macAddress()).c_str(), ETH.localIP().toString().c_str(),
//                            String(ETH.linkSpeed()).c_str());
////      if (ETH.fullDuplex()) {
////        log_message(log_info, ", FULL_DUPLEX");
////      }
//      ethConnected = true;
//      break;
//    case SYSTEM_EVENT_ETH_DISCONNECTED:
//      log_message(log_info, "ETH Disconnected");
//      ethConnected = false;
//      break;
//    case SYSTEM_EVENT_ETH_STOP:
//      log_message(log_info, "ETH Stopped");
//      ethConnected = false;
//      break;
//
//// --------------- WiFi ----------------------
//    case SYSTEM_EVENT_STA_START:
//      log_message(log_info, "WiFi Client Started");
//      //set eth hostname here
//      WiFi.setHostname(esp32Hostname);
//      break;
//    case SYSTEM_EVENT_STA_CONNECTED:
//      log_message(log_info, "WiFi Client Connected");
//      break;
//    case SYSTEM_EVENT_STA_GOT_IP:
//      log_message(log_info, "WiFi MAC: %s, IPv4: %s", String(WiFi.macAddress()).c_str(), WiFi.localIP().toString().c_str());
//      wifiConnected = true;
//      break;
//    case SYSTEM_EVENT_STA_DISCONNECTED:
//      log_message(log_info, "WiFi Client Disconnected");
//      wifiConnected = false;
//      break;
//    case SYSTEM_EVENT_STA_STOP:
//      log_message(log_info, "WiFi Client Stopped");
//      wifiConnected = false;
//      break;
//    default:
//      break;
//  }
//}

void ethInit() {



  WiFi.onEvent(WiFiEvent);
  if (ETH.begin(ethAddr, ethPowerPin, ethMDCPin, ethMDIOPin, phyType, clkMode)) {
    log_message(log_info, "ETH.begin ok");
  } else {
    log_message(log_error, "ETH.begin error");
  }
}

//void wifiClientInit() {
//  WiFi.onEvent(WiFiEvent);
//  if (WiFi.begin(ssid, pass)) {
//    log_message(log_info, "WiFi.begin ok");
//  } else {
//    log_message(log_error, "WiFi.begin error");
//  }
//}

//} // namespace imsl

// -------------------- some old stuff / trash:
//#include "WiFi.h"     // does not work correctly .isConnected failed
//#include "EspIdfWiFi.h" // own implementation of WiFi

/*
WiFi:
0  SYSTEM_EVENT_WIFI_READY               < ESP32 WiFi ready
1  SYSTEM_EVENT_SCAN_DONE                < ESP32 finish scanning AP
2  SYSTEM_EVENT_STA_START                < ESP32 station start
3  SYSTEM_EVENT_STA_STOP                 < ESP32 station stop
4  SYSTEM_EVENT_STA_CONNECTED            < ESP32 station connected to AP
5  SYSTEM_EVENT_STA_DISCONNECTED         < ESP32 station disconnected from AP
6  SYSTEM_EVENT_STA_AUTHMODE_CHANGE      < the auth mode of AP connected by ESP32 station changed
7  SYSTEM_EVENT_STA_GOT_IP               < ESP32 station got IP from connected AP
8  SYSTEM_EVENT_STA_LOST_IP
*/

/* class ETHClass
 * bool begin(uint8_t phy_addr=ETH_PHY_ADDR, int power=ETH_PHY_POWER, int mdc=ETH_PHY_MDC, int mdio=ETH_PHY_MDIO, eth_phy_type_t type=ETH_PHY_TYPE, eth_clock_mode_t clk_mode=ETH_CLK_MODE);
 */

/*
    EspIdfWiFi.begin(ssid, pass);
    while(!EspIdfWiFi.isConnected()) {
      printf("."); delay(500);
    }
*/

//  delete old config
//    WiFi.disconnect(true);
//  delay(1000);
//  WiFi.onEvent(WiFiEvent);

//    WiFi.begin(ssid, pass);

    // dirty solution for status detection
/*
    IPAddress result;
    while (WiFi.hostByName("www.google.com", result) != 1) {
      printf("."); delay(500);
    }

    printf("Connected with IP address: %s\n", WiFi.localIP().toString().c_str());
    WiFi._setStatus(WL_CONNECTED);
*/
/*
    while(!WiFi.isConnected()) {
      printf("."); fflush(stdout); delay(500);
    }

    printf("Connected with IP address: %s\n", WiFi.localIP().toString().c_str());
*/
//    WiFi.waitForConnectResult();

/*
    IPAddress result;
    while (WiFi.hostByName("www.google.com", result) != 1) {
      printf("."); delay(500);
    }

    printf("Connected with IP address: %s\n", WiFi.localIP().toString().c_str());
    WiFi._setStatus(WL_CONNECTED);
*/

