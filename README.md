
To run main.c you first need to create a file called "connection_info.h" with the following:

```
#define WLAN_SSID      "apname"
#define WLAN_PASS      "appassword"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY  WLAN_SEC_WPA2
```
