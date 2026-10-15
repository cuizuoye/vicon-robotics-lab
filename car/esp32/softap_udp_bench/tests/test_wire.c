#include <assert.h>
#include "bench_wire.h"
int main(void)
{
    uint8_t in[SAP_REQUEST_SIZE]={0},out[SAP_REPLY_SIZE];
    memcpy(in,"SAP1",4);in[4]=1;in[5]=1;
    sap_put32(in+8,1);sap_put32(in+12,123);sap_put64(in+16,9876543210123ULL);
    assert(sap_valid(in,sizeof(in)));assert(!sap_valid(in,31));assert(!sap_valid(in,33));
    in[4]=2;assert(!sap_valid(in,32));in[4]=1;
    in[6]=1;assert(!sap_valid(in,32));in[6]=0;
    in[24]=1;assert(!sap_valid(in,32));in[24]=0;
    sap_put32(in+12,0);assert(!sap_valid(in,32));sap_put32(in+12,123);
    sap_reply(out,in,100,110,42,8,20000);
    assert(!memcmp(out,"SAR1",4));assert(out[4]==1 && out[5]==2);
    assert(sap_u32(out+8)==1 && sap_u32(out+12)==123);
    assert(sap_u64(out+16)==9876543210123ULL);
    assert(sap_u64(out+24)==100 && sap_u64(out+32)==110);
    assert(sap_u32(out+40)==42 && sap_u32(out+44)==8);
    assert(sap_u64(out+48)==20000 && sap_u64(out+56)==0);
    return 0;
}
