#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/*
 * Local deployments may override these safe defaults in app_config.local.h.
 * Defining APP_CONFIG_IGNORE_LOCAL skips that file so a build (notably the CI
 * Wi-Fi configuration) gets exactly the settings passed on the command line,
 * whatever happens to be in the working tree.
 */
#if defined(__has_include) && !defined(APP_CONFIG_IGNORE_LOCAL)
#  if __has_include("app_config.local.h")
#    include "app_config.local.h"
#  endif
#endif

#ifndef APP_WIFI_ENABLED
#define APP_WIFI_ENABLED 0
#endif
#ifndef APP_WIFI_SSID
#define APP_WIFI_SSID ""
#endif
#ifndef APP_WIFI_PASSWORD
#define APP_WIFI_PASSWORD ""
#endif
#ifndef APP_SERVER_ADDRESS
#define APP_SERVER_ADDRESS "192.0.2.1"
#endif
#ifndef APP_SERVER_PORT
#define APP_SERVER_PORT 2005U
#endif
#ifndef APP_ALLOW_REMOTE_RESET
#define APP_ALLOW_REMOTE_RESET 0
#endif
#ifndef APP_MEASUREMENT_SEND_INTERVAL_MS
#define APP_MEASUREMENT_SEND_INTERVAL_MS 1500U
#endif
#ifndef APP_SCD30_AMBIENT_PRESSURE_MBAR
#define APP_SCD30_AMBIENT_PRESSURE_MBAR 0U
#endif

#endif /* APP_CONFIG_H */
