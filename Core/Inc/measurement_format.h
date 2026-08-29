#ifndef MEASUREMENT_FORMAT_H
#define MEASUREMENT_FORMAT_H

#include <stddef.h>

#include "measurement_store.h"

/**
 * Buffer size that always holds a formatted measurement line, including the
 * terminating null. Verified by the host test suite against a worst-case
 * snapshot.
 */
#define MEASUREMENT_FORMAT_MAX_LEN 256U

/**
 * Formats a snapshot as one protocol line. Returns the number of characters
 * written, or -1 when the arguments are invalid or the output was truncated.
 */
int Measurements_Format(char* output, size_t output_capacity,
                        const MeasurementSnapshot* measurement);

#endif /* MEASUREMENT_FORMAT_H */
