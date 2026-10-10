// sim_shim_esp/mbedtls_stub.cpp — host-native stub implementations for the
// mbedtls subset used by src/superb/main.cpp in [env:sim_superb].
// INFERRED: NOT real crypto. sha256_finish_ret zero-fills; ecdsa verify
// always succeeds. Auth/OTA code paths compile and run in sim; real
// verification happens on hardware only.

#include "mbedtls/sha256.h"
#include "mbedtls/ecdsa.h"

#include <string.h>

extern "C" {

void mbedtls_sha256_init(mbedtls_sha256_context* ctx)
{
    if (ctx) memset(ctx, 0, sizeof(*ctx));
}
void mbedtls_sha256_free(mbedtls_sha256_context* ctx) { (void)ctx; }
int mbedtls_sha256_starts_ret(mbedtls_sha256_context* ctx, int is224)
{
    (void)ctx; (void)is224;
    return 0;
}
int mbedtls_sha256_update_ret(mbedtls_sha256_context* ctx,
                              const unsigned char* input, size_t ilen)
{
    (void)ctx; (void)input; (void)ilen;
    return 0;
}
int mbedtls_sha256_finish_ret(mbedtls_sha256_context* ctx,
                              unsigned char* output)
{
    (void)ctx;
    if (output) memset(output, 0, 32);
    return 0;
}
int mbedtls_sha256_ret(const unsigned char* input, size_t ilen,
                       unsigned char* output, int is224)
{
    (void)input; (void)ilen; (void)is224;
    if (output) memset(output, 0, 32);
    return 0;
}

int mbedtls_ecp_group_load(mbedtls_ecp_group* grp, mbedtls_ecp_group_id id)
{
    (void)grp; (void)id;
    return 0;
}
int mbedtls_ecp_point_read_binary(const mbedtls_ecp_group* grp,
                                  mbedtls_ecp_point* pt,
                                  const unsigned char* buf, size_t ilen)
{
    (void)grp; (void)pt; (void)buf; (void)ilen;
    return 0;
}

void mbedtls_ecdsa_init(mbedtls_ecdsa_context* ctx)
{
    if (ctx) memset(ctx, 0, sizeof(*ctx));
}
void mbedtls_ecdsa_free(mbedtls_ecdsa_context* ctx) { (void)ctx; }
int mbedtls_ecdsa_read_signature(mbedtls_ecdsa_context* ctx,
                                 const unsigned char* hash, size_t hlen,
                                 const unsigned char* sig, size_t slen)
{
    (void)ctx; (void)hash; (void)hlen; (void)sig; (void)slen;
    return 0;  // sim: always "valid"
}

}  // extern "C"

// Update singleton for the sim (mirrors ESP32 Arduino's global Update).
#include "Update.h"
UpdateClass Update;
