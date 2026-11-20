#ifndef BSEC_PROCESSING_H
#define BSEC_PROCESSING_H

#include "bme680/bme680.h"

typedef struct
{
    uint8_t version[4];
    int32_t init_status;
    int32_t control_status;
    int32_t process_status;
    int32_t state_status;
    uint32_t sample_count;
    uint32_t output_count;
    uint32_t state_sequence;
    uint32_t state_save_count;
    uint32_t save_requested;
    uint32_t last_output_ms;
    uint8_t state_restored;
    uint8_t accuracy;
    uint8_t stabilized;
    uint8_t run_in;
    float iaq;
    float static_iaq;
    float co2_equivalent_ppm;
    float breath_voc_equivalent_ppm;
    float compensated_temperature_c;
    float compensated_humidity_percent;
} BsecDiagnostics;

extern volatile BsecDiagnostics bsec_diagnostics;
#define BSEC_SERVICE_IDLE INT8_C(127)
int8_t BsecProcessing_Init(void);
int8_t BsecProcessing_Service(struct bme680_dev* sensor);

#endif
