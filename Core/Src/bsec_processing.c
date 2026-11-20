#include "app_config.h"

#if APP_BSEC_ENABLED
#include "bsec_processing.h"
#include "bsec_interface.h"
#include "bsec_state_record.h"
#include "bsec_state_store.h"
#include "bosch_sensor.h"
#include "cmsis_os.h"
#include "measurement_store.h"
#include "device_config.h"

#include <string.h>

static const uint8_t configuration[] = {
#include "bsec_iaq.txt"
};

volatile BsecDiagnostics bsec_diagnostics;
static uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
static uint8_t state_blob[BSEC_MAX_STATE_BLOB_SIZE];
static bsec_bme_settings_t settings;
static IaqMeasurement latest;
static uint32_t configuration_tag;
static uint32_t last_tick;
static uint64_t uptime_ms;
static int64_t trigger_time_ns;
static int64_t read_time_ns;
static uint8_t measurement_pending;
static uint32_t last_state_save_ms;
static uint8_t saved_accuracy;

/* Extend the RTOS's 32-bit millisecond tick before converting to nanoseconds. */
static int64_t now_ns(void)
{
    const uint32_t tick = osKernelSysTick();
    uptime_ms += (uint32_t)(tick - last_tick);
    last_tick = tick;
    return (int64_t)(uptime_ms * UINT64_C(1000000));
}

int8_t BsecProcessing_Init(void)
{
    memset(&settings, 0, sizeof(settings));
    memset(&latest, 0, sizeof(latest));
    measurement_pending = 0U;
    last_tick = osKernelSysTick();
    uptime_ms = last_tick;
    last_state_save_ms = last_tick;
    saved_accuracy = 0U;
    bsec_library_return_t status = bsec_init();
    bsec_diagnostics.init_status = status;
    if(status < BSEC_OK) { return BME680_E_COM_FAIL; }
    bsec_version_t version;
    status = bsec_get_version(&version);
    if(status < BSEC_OK) { bsec_diagnostics.init_status = status; return BME680_E_COM_FAIL; }
    bsec_diagnostics.version[0] = version.major;
    bsec_diagnostics.version[1] = version.minor;
    bsec_diagnostics.version[2] = version.major_bugfix;
    bsec_diagnostics.version[3] = version.minor_bugfix;
    configuration_tag = BsecState_Crc32(configuration, sizeof(configuration)) ^
        ((uint32_t)version.major << 24U) ^ ((uint32_t)version.minor << 16U) ^
        ((uint32_t)version.major_bugfix << 8U) ^ version.minor_bugfix;

    status = bsec_set_configuration(configuration, sizeof(configuration), work_buffer, sizeof(work_buffer));
    bsec_diagnostics.init_status = status;
    if(status < BSEC_OK) { return BME680_E_COM_FAIL; }

    uint32_t sequence = 0U;
    const int32_t state_length = BsecState_Load(configuration_tag, state_blob, sizeof(state_blob), &sequence);
    bsec_diagnostics.state_restored = 0U;
    bsec_diagnostics.state_status = state_length < 0 ? -1 : 0;
    if(state_length > 0)
    {
        status = bsec_set_state(state_blob, (uint32_t)state_length, work_buffer, sizeof(work_buffer));
        bsec_diagnostics.state_status = status;
        bsec_diagnostics.state_restored = status == BSEC_OK;
        bsec_diagnostics.state_sequence = sequence;
    }

    static const uint8_t outputs[] = {
        BSEC_OUTPUT_IAQ, BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT, BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_STABILIZATION_STATUS, BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        /* Keep the existing raw measurement record fully populated too. */
        BSEC_OUTPUT_RAW_PRESSURE, BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_HUMIDITY, BSEC_OUTPUT_RAW_GAS
    };
    bsec_sensor_configuration_t requested[sizeof(outputs)];
    bsec_sensor_configuration_t required[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t required_count = BSEC_MAX_PHYSICAL_SENSOR;
    for(size_t i = 0U; i < sizeof(outputs); ++i)
    {
        requested[i].sample_rate = BSEC_SAMPLE_RATE_LP;
        requested[i].sensor_id = outputs[i];
    }
    status = bsec_update_subscription(requested, sizeof(outputs), required, &required_count);
    bsec_diagnostics.init_status = status;
    return status < BSEC_OK ? BME680_E_COM_FAIL : BME680_OK;
}

static void save_state_if_due(void)
{
    const uint32_t now = osKernelSysTick();
    DeviceConfig config;
    DeviceConfig_Get(&config);
    const uint8_t requested = bsec_diagnostics.save_requested != 0U;
    const uint8_t due = latest.accuracy > saved_accuracy ||
        (latest.accuracy != 0U && (now - last_state_save_ms) >= config.bsec_save_interval_s * 1000U);
    if(!requested && !due) { return; }
    bsec_diagnostics.save_requested = 0U;
    uint32_t length = 0U;
    bsec_library_return_t status = bsec_get_state(0U, state_blob, sizeof(state_blob),
        work_buffer, sizeof(work_buffer), &length);
    bsec_diagnostics.state_status = status;
    if(status == BSEC_OK)
    {
        uint32_t sequence = 0U;
        const int result = BsecState_Save(configuration_tag, state_blob, length, &sequence);
        bsec_diagnostics.state_status = result;
        if(result == 0)
        {
            bsec_diagnostics.state_sequence = sequence;
            ++bsec_diagnostics.state_save_count;
            last_state_save_ms = now;
            saved_accuracy = latest.accuracy;
        }
        else
        {
            /* Do not erase repeatedly if storage is unavailable. */
            last_state_save_ms = now;
            saved_accuracy = latest.accuracy;
        }
    }
}

static void input_add(bsec_input_t* inputs, uint8_t* count, uint8_t id, float value)
{
    if((settings.process_data & (UINT32_C(1) << (id - 1U))) != 0U)
    {
        inputs[*count].sensor_id = id;
        inputs[*count].signal = value;
        inputs[*count].signal_dimensions = 1U;
        inputs[*count].time_stamp = trigger_time_ns;
        ++*count;
    }
}

static int8_t process_measurement(const struct bme680_field_data* data)
{
    if((data->status & BME680_GASM_VALID_MSK) == 0U) { return BME680_OK; }
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR] = {0};
    bsec_output_t outputs[16] = {0};
    uint8_t count = 0U;
    input_add(inputs, &count, BSEC_INPUT_PRESSURE, data->pressure);
    input_add(inputs, &count, BSEC_INPUT_TEMPERATURE, data->temperature / 100.0F);
    input_add(inputs, &count, BSEC_INPUT_HUMIDITY, data->humidity / 1000.0F);
    input_add(inputs, &count, BSEC_INPUT_GASRESISTOR, (float)data->gas_resistance);
    DeviceConfig config;
    DeviceConfig_Get(&config);
    input_add(inputs, &count, BSEC_INPUT_HEATSOURCE, config.bsec_heat_offset / 100.0F);
    input_add(inputs, &count, BSEC_INPUT_PROFILE_PART, 0.0F);
    uint8_t output_count = sizeof(outputs) / sizeof(outputs[0]);
    const bsec_library_return_t status = bsec_do_steps(inputs, count, outputs, &output_count);
    bsec_diagnostics.process_status = status;
    ++bsec_diagnostics.sample_count;
    if(status < BSEC_OK) { return BME680_E_COM_FAIL; }
    uint8_t have_iaq = 0U;
    for(uint8_t i = 0U; i < output_count; ++i)
    {
        const float value = outputs[i].signal;
        switch(outputs[i].sensor_id)
        {
        case BSEC_OUTPUT_IAQ: latest.index = value; latest.accuracy = outputs[i].accuracy; have_iaq = 1U; break;
        case BSEC_OUTPUT_STATIC_IAQ: latest.static_index = value; break;
        case BSEC_OUTPUT_CO2_EQUIVALENT: latest.co2_equivalent_ppm = value; break;
        case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT: latest.breath_voc_equivalent_ppm = value; break;
        case BSEC_OUTPUT_STABILIZATION_STATUS: latest.stabilized = value != 0.0F; break;
        case BSEC_OUTPUT_RUN_IN_STATUS: latest.run_in = value != 0.0F; break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE: latest.temperature_c = value; break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY: latest.relative_humidity_percent = value; break;
        default: break;
        }
    }
    if(have_iaq)
    {
        bsec_diagnostics.iaq = latest.index;
        bsec_diagnostics.static_iaq = latest.static_index;
        bsec_diagnostics.co2_equivalent_ppm = latest.co2_equivalent_ppm;
        bsec_diagnostics.breath_voc_equivalent_ppm = latest.breath_voc_equivalent_ppm;
        bsec_diagnostics.compensated_temperature_c = latest.temperature_c;
        bsec_diagnostics.compensated_humidity_percent = latest.relative_humidity_percent;
        bsec_diagnostics.accuracy = latest.accuracy;
        bsec_diagnostics.stabilized = latest.stabilized;
        bsec_diagnostics.run_in = latest.run_in;
        bsec_diagnostics.last_output_ms = osKernelSysTick();
        ++bsec_diagnostics.output_count;
        Measurements_UpdateIaq(&latest, osKernelSysTick());
        save_state_if_due();
    }
    return BME680_OK;
}

int8_t BsecProcessing_Service(struct bme680_dev* sensor)
{
    const int64_t now = now_ns();
    if(measurement_pending && now >= read_time_ns)
    {
        measurement_pending = 0U;
        struct bme680_field_data data = {0};
        const int8_t status = bme680_get_sensor_data(&data, sensor);
        bme680_diagnostics.read_status = status;
        bme680_diagnostics.field_status = data.status;
        if(status != BME680_OK) { return status; }
        ++bme680_diagnostics.sample_count;
        bme680_diagnostics.last_sample_ms = osKernelSysTick();
        Measurements_UpdateBme680(data.temperature / 100.0F, data.humidity / 1000.0F,
            data.pressure / 100.0F, data.gas_resistance, osKernelSysTick());
        return process_measurement(&data);
    }
    if(measurement_pending || now < settings.next_call) { return BSEC_SERVICE_IDLE; }
    const bsec_library_return_t result = bsec_sensor_control(now, &settings);
    bsec_diagnostics.control_status = result;
    if(result < BSEC_OK) { return BME680_E_COM_FAIL; }
    if(!settings.trigger_measurement) { return BSEC_SERVICE_IDLE; }
    if(settings.op_mode != BME680_FORCED_MODE) { return BME680_E_COM_FAIL; }
    sensor->tph_sett.os_hum = settings.humidity_oversampling;
    sensor->tph_sett.os_pres = settings.pressure_oversampling;
    sensor->tph_sett.os_temp = settings.temperature_oversampling;
    sensor->tph_sett.filter = BME680_FILTER_SIZE_3;
    sensor->gas_sett.run_gas = settings.run_gas;
    sensor->gas_sett.heatr_temp = settings.heater_temperature;
    sensor->gas_sett.heatr_dur = settings.heater_duration;
    sensor->power_mode = BME680_FORCED_MODE;
    int8_t status = bme680_set_sensor_settings(BME680_OST_SEL | BME680_OSP_SEL |
        BME680_OSH_SEL | BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL, sensor);
    if(status == BME680_OK) { status = bme680_set_sensor_mode(sensor); }
    bme680_diagnostics.trigger_status = status;
    if(status != BME680_OK) { return status; }
    uint16_t duration_ms = 0U;
    bme680_get_profile_dur(&duration_ms, sensor);
    trigger_time_ns = now;
    read_time_ns = now_ns() + (int64_t)(duration_ms + 10U) * INT64_C(1000000);
    measurement_pending = 1U;
    return BSEC_SERVICE_IDLE;
}
#endif
