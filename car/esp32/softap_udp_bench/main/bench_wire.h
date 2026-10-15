/* Diagnostic protocol only. NOT MC-Link and never forwarded to a UART. */
#ifndef BENCH_WIRE_H
#define BENCH_WIRE_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#define SAP_REQUEST_SIZE 32u
#define SAP_REPLY_SIZE 64u
#define SAP_PORT 46010
static inline uint32_t sap_u32(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static inline uint64_t sap_u64(const uint8_t *p)
{ return sap_u32(p)|((uint64_t)sap_u32(p+4)<<32); }
static inline void sap_put32(uint8_t *p,uint32_t v)
{ for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i)); }
static inline void sap_put64(uint8_t *p,uint64_t v)
{ sap_put32(p,(uint32_t)v);sap_put32(p+4,(uint32_t)(v>>32)); }
static inline bool sap_valid(const uint8_t *p,size_t n)
{
    return n==SAP_REQUEST_SIZE && !memcmp(p,"SAP1",4) && p[4]==1 && p[5]==1
        && p[6]==0 && p[7]==0 && sap_u32(p+8)>0 && sap_u32(p+12)>0
        && sap_u64(p+24)==0;
}
static inline void sap_reply(uint8_t *out,const uint8_t *in,uint64_t rx,
    uint64_t tx,uint32_t boot,uint32_t count,uint64_t max_gap)
{
    memset(out,0,SAP_REPLY_SIZE);memcpy(out,"SAR1",4);out[4]=1;out[5]=2;
    memcpy(out+8,in+8,16);sap_put64(out+24,rx);sap_put64(out+32,tx);
    sap_put32(out+40,boot);sap_put32(out+44,count);sap_put64(out+48,max_gap);
}
#endif
