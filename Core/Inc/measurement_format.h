#ifndef MEASUREMENT_FORMAT_H
#define MEASUREMENT_FORMAT_H

#include <stddef.h>

#include "measurement_store.h"

int Measurements_Format(char* output, size_t output_capacity,
                        const MeasurementSnapshot* measurement);

#endif /* MEASUREMENT_FORMAT_H */
