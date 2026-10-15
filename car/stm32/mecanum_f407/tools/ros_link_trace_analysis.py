"""Offline correlation of readonly ROS send records and ESP RAM-ring dump."""
import argparse
import json
from pathlib import Path

from link_timing import summarize_timing


def analyze(observation, chunks):
    # USB reads may split an ESP log line anywhere. Reassemble before parsing.
    text = ''.join(chunk['text'] for chunk in chunks)
    lines = [{'text': line} for line in text.splitlines()]
    first_fault = next((row for row in observation['topics']['fault'] if row['value']), None)
    cutoff = first_fault['time_s'] if first_fault else observation['observation_end_s']
    sent = [row for row in observation.get('host_send_timing', [])
            if row['end_s'] <= cutoff and 'error_errno' not in row]
    timing = summarize_timing(sent, lines)
    fault = timing['esp_fault']
    rx = [event for event in timing['esp_events'] if event['kind'] == 1]
    uart = [event for event in timing['esp_events'] if event['kind'] == 2]
    if not first_fault or not fault or not timing['esp_dump_complete'] or not rx:
        raise ValueError('Need a real MCU fault and complete ESP trace with RX events')
    if fault['session_id'] not in {row.get('session_id') for row in sent}:
        raise ValueError('ESP fault session does not match recorded sends')
    newer = lambda seq, old: 0 < ((seq - old) & 0xffffffff) < 0x80000000
    last = rx[-1]
    commands = [row for row in sent if row.get('kind') == 16
                and row['session_id'] == fault['session_id']]
    if not commands or not all(row.get('disabled_zero') for row in commands):
        raise ValueError('Not a disabled-zero observation')
    pairs = [dict(from_sequence=a['sequence'], to_sequence=b['sequence'],
                  esp_gap_ms=(b['at_ms']-a['at_ms']) & 0xffffffff)
             for a, b in zip(rx, rx[1:])]
    largest_gap = max(pairs, key=lambda pair: pair['esp_gap_ms']) if pairs else None
    if largest_gap:
        by_sequence = {row['sequence']: row for row in commands}
        before = by_sequence.get(largest_gap['from_sequence'])
        after = by_sequence.get(largest_gap['to_sequence'])
        largest_gap['matched_host_send_gap_ms'] = (
            (after['start_s']-before['start_s'])*1000 if before and after else None)
        largest_gap['preceding_rx_equals_mcu_last_command'] = (
            largest_gap['from_sequence'] == fault['last_command_sequence'])
    return dict(
        first_fault_host_s=first_fault['time_s'], timing=timing,
        largest_completed_esp_rx_gap=largest_gap,
        last_esp_rx=last, last_uart_write=uart[-1] if uart else None,
        last_rx_equals_mcu_last_command=last['sequence'] == fault['last_command_sequence'],
        host_sent_newer_sequences_before_fault_callback=[row['sequence'] for row in commands
            if newer(row['sequence'], last['sequence'])],
        recent_uart_writes_all_successful=bool(uart) and all(event['detail'] == 1 for event in uart),
        all_observed_sends_disabled_zero=True,
        clocks_synchronized=False,
        limitations='RX records are validated accepted UDP frames in the ESP application, not radio ingress. Newer sends precede host fault callback, not necessarily MCU fault onset. Delayed and lost packets are not distinguished. UART enqueue success alone is not MCU reception proof.',
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observation', type=Path, required=True)
    parser.add_argument('--serial-json', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze(json.loads(args.observation.read_text()), json.loads(args.serial_json.read_text()))
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'timing'}, indent=2))


if __name__ == '__main__':
    main()
