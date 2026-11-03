"""Parse diagnostic-only serial snapshots; never open a control transport."""
import re

FIELDS=('ms sample callback_delay pending enqueue_errors echo_stack tcpip_stack '
        'udp_rx udp_drop udp_memerr udp_chkerr ip_drop ip_memerr arp_memerr '
        'pools_available pool_errors pbuf_used pbuf_peak pbuf_errors '
        'arp_used arp_peak arp_errors').split()

def parse_resource_line(line):
    line=re.sub(r'\x1b\[[0-9;]*m','',line)
    marker='WIFI_RESOURCES '
    if marker not in line:return None
    body=line.split(marker,1)[1].strip();result={}
    for token in body.split():
        key,separator,value=token.partition('=')
        if not separator or key not in FIELDS or key in result or not value.isascii() or not value.isdecimal():
            raise ValueError('Malformed Wi-Fi resource snapshot')
        result[key]=int(value)
    if set(result)!=set(FIELDS) or result['pending'] not in (0,1) or result['pools_available'] not in (0,1):
        raise ValueError('Incomplete Wi-Fi resource snapshot')
    return result

def summarize_resources(records):
    usable=[r for r in records if r['sample']>0 and r['echo_stack']>0 and r['tcpip_stack']>0]
    if not usable:return {'valid_snapshots':0}
    counters=['enqueue_errors','udp_drop','udp_memerr','udp_chkerr','ip_drop','ip_memerr','arp_memerr']
    pools=all(r['pools_available'] for r in usable)
    return dict(valid_snapshots=len(usable),
        min_echo_stack_free_bytes=min(r['echo_stack'] for r in usable),
        min_tcpip_stack_free_bytes=min(r['tcpip_stack'] for r in usable),
        max_callback_delay_ms=max(r['callback_delay'] for r in usable),
        max_observed_error_counters={k:max(r[k] for r in usable) for k in counters},
        pool_statistics_available=pools,
        max_pool_errors=max(r['pool_errors'] for r in usable) if pools else None,
        note='Whole-interface lwIP counters, not command counts; unavailable pool gauges are not zero-error evidence')
