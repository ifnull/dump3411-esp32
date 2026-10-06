/* Small JSON writers shared by the host tools. */
#ifndef JSON_OUT_H
#define JSON_OUT_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* bytes.decode('ascii', errors='replace') as a JSON string, like dump3411. */
static inline void json_ascii(const uint8_t *b, size_t n)
{
    putchar('"');
    for (size_t i = 0; i < n; i++) {
        uint8_t c = b[i];
        if (c >= 0x80) {
            fputs("\\ufffd", stdout);
        } else if (c == '"' || c == '\\') {
            printf("\\%c", c);
        } else if (c < 0x20 || c == 0x7F) {
            printf("\\u%04x", c);
        } else {
            putchar(c);
        }
    }
    putchar('"');
}

#endif
