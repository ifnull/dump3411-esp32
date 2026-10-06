#ifndef SELFTEST_H
#define SELFTEST_H

#include <stdbool.h>

/* Decode a known reference-encoder frame on the device; true if every field matches. */
bool odid_selftest(void);

#endif
