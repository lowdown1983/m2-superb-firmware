#pragma once
// sim_shim_esp/mbedtls/ecdsa.h — host-native stub of mbedTLS ECDSA for
// [env:sim_superb]. API-compatible subset of what src/superb/main.cpp uses.
// INFERRED: verify always succeeds in sim. Signature verification is only
// meaningful on hardware (see docs/SIM.md Phase 11 note).

#include "ecp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    int dummy;
} mbedtls_ecdsa_context;

void mbedtls_ecdsa_init(mbedtls_ecdsa_context* ctx);
void mbedtls_ecdsa_free(mbedtls_ecdsa_context* ctx);
int mbedtls_ecdsa_read_signature(mbedtls_ecdsa_context* ctx,
                                 const unsigned char* hash, size_t hlen,
                                 const unsigned char* sig, size_t slen);

#ifdef __cplusplus
}
#endif
