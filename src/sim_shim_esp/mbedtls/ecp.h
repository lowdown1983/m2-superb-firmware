#pragma once
// sim_shim_esp/mbedtls/ecp.h — host-native stub of mbedTLS ECP for
// [env:sim_superb]. API-compatible subset of what src/superb/main.cpp uses.
// INFERRED: no real elliptic-curve math; all calls succeed.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MBEDTLS_ECP_DP_NONE = 0,
    MBEDTLS_ECP_DP_SECP256R1 = 1,
} mbedtls_ecp_group_id;

typedef struct {
    int dummy;
} mbedtls_ecp_group;

typedef struct {
    int dummy;
} mbedtls_ecp_point;

int mbedtls_ecp_group_load(mbedtls_ecp_group* grp,
                           mbedtls_ecp_group_id id);
int mbedtls_ecp_point_read_binary(const mbedtls_ecp_group* grp,
                                  mbedtls_ecp_point* pt,
                                  const unsigned char* buf, size_t ilen);

#ifdef __cplusplus
}
#endif
