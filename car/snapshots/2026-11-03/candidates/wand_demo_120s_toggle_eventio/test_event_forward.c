#include "mc_event_forward.h"
#include <assert.h>
#include <stdio.h>
#include <sys/eventfd.h>

int main(void)
{
    int command[2];assert(pipe(command)==0);
    const int wake=eventfd(0,0);assert(wake>=0);
    assert(mc_forward_drain_count(0,128)==0);
    assert(mc_forward_drain_count(200,128)==127);
    assert(mc_forward_drain_count(12,128)==12);
    assert(mc_forward_drain_count(12,0)==0);
    assert(mc_forward_wait(-1,wake)==-1);
    assert(mc_forward_wait(wake,wake)==-1);
    assert(mc_forward_wait(command[0],FD_SETSIZE)==-1);
    assert(!mc_forward_notify(-1));
    assert(mc_forward_wait(command[0],wake)==0);
    assert(mc_forward_notify(wake));assert(mc_forward_notify(wake));
    assert(mc_forward_wait(command[0],wake)==0); /* UART wakes without a UDP request. */
    const char value='x';assert(write(command[1],&value,1)==1);
    assert(mc_forward_notify(wake));
    assert(mc_forward_wait(command[0],wake)==1); /* Simultaneous events preserved. */
    char received;assert(read(command[0],&received,1)==1 && received=='x');
    assert(mc_forward_wait(command[0],wake)==0); /* Counter drained, no busy wake. */
    close(wake);close(command[0]);close(command[1]);
    puts("PASS: bounded event wait, coalesced UART wake, simultaneous UDP, errors and chunk bounds");
}
