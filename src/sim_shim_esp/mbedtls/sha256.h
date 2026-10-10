#pragma once
// sim_shim_esp/mbedtls/sha256.h — host-native stub of mbedTLS SHA-256 for
// [env:sim_superb]. API-compatible subset of what src/superb/main.cpp uses.
// INFERRED: NOT real SHA-256 (zero-fill / passthrough). Crypto is not
// exercised in sim; auth/OTA logic paths compile and run, hashes verify
// only on hardware.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t total[2];
    uint32_t state[8];
    unsigned char buffer[64];
    int is224;
} mbedtls_sha256_context;

void mbedtls_sha256_init(mbedtls_sha256_context* ctx);
void mbedtls_sha256_free(mbedtls_sha256_context* ctx);
int mbedtls_sha256_starts_ret(mbedtls_sha256_context* ctx, int is224);
int mbedtls_sha256_update_ret(mbedtls_sha256_context* ctx,
                              const unsigned char* input, size_t ilen);
int mbedtls_sha256_finish_ret(mbedtls_sha256_context* ctx,
                              unsigned char* output);
int mbedtls_sha256_ret(const unsigned char* input, size_t ilen,
                       unsigned char* output, int is224);

#ifdef __cplusplus
}
#endif
