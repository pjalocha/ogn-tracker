#pragma once

#include <stdint.h>

#include "mbedtls/sha256.h"

class SHA256
{ public:
   mbedtls_sha256_context Context;

  public:
   SHA256() { Init(); }
  ~SHA256() { Free(); }
   void Init(void)                               { mbedtls_sha256_init(&Context); }
   void Free(void)                               { mbedtls_sha256_free(&Context); }
   int  Start(void)                              { return mbedtls_sha256_starts_ret(&Context, 0); }
   int  Update(const uint8_t *Input, size_t Len) { return mbedtls_sha256_update_ret(&Context, Input, Len); }
   void Clone(const SHA256 &Src)                 { mbedtls_sha256_clone(&Context, &Src.Context); }
   int  Finish(uint8_t CheckSum[32])             { return mbedtls_sha256_finish_ret(&Context, CheckSum); }

} ;

class SHA512
{ public:
   mbedtls_sha512_context Context;

  public:
   SHA512() { Init(); }
  ~SHA512() { Free(); }
   void Init(void)                              { mbedtls_sha512_init(&Context); }
   void Free(void)                              { mbedtls_sha512_free(&Context); }
   int Start(void)                              { return mbedtls_sha512_starts_ret(&Context, 0); }
   int Update(const uint8_t *Input, size_t Len) { return mbedtls_sha512_update_ret(&Context, Input, Len); }
   int Finish(uint8_t CheckSum[64])             { return mbedtls_sha512_finish_ret(&Context, CheckSum); }

} ;

