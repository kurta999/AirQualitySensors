#ifndef WEB_ASSETS_H
#define WEB_ASSETS_H
#include <stddef.h>
#include <stdint.h>
typedef struct { const char* path; const char* type; const uint8_t* data; uint32_t length; } WebAsset;
extern const WebAsset web_assets[];
extern const size_t web_asset_count;
#endif
