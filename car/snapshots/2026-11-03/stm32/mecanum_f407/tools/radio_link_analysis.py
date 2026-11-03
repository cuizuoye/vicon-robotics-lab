"""Offline SDK radio metadata comparison; does not sniff or open a socket."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

from ros_link_trace_analysis import analyze as analyze_application


def summarize_radio(text, gap_start_ms, gap_end_ms, fault_ms=None):
    pattern = re.compile(r'TRACE_RADIO cb_ms=(\d+) hw_us=(\d+) seq=(\d+) retry=(\d+) len=(\d+)(?: tid=(\d+))?')
    fields = ('callback_ms', 'hardware_us', 'sequence', 'retry', 'frame_length', 'tid')
    events = [dict(zip(fields, (int(value) if value is not None else None for value in match.groups())))
              for match in pattern.finditer(text)]
    ending = re.search(r'TRACE_RADIO_END count=(\d+)', text)
    if not ending or int(ending[1]) != len(events) or not 2 <= len(events) <= 128:
        raise ValueError('Missing, truncated or empty radio ring')
    if any(row['retry'] not in (0, 1) or row['sequence'] > 4095
           or row['tid'] not in (*range(16), 255, None) for row in events):
        raise ValueError('Invalid 802.11 sequence/retry metadata')
    interval = (gap_end_ms-gap_start_ms) & 0xffffffff
    if interval >= 0x80000000:
        raise ValueError('Reversed application interval')
    pairs = [dict(before=a, after=b,
        callback_gap_ms=(b['callback_ms']-a['callback_ms']) & 0xffffffff,
        hardware_gap_us=(b['hardware_us']-a['hardware_us']) & 0xffffffff)
        for a, b in zip(events, events[1:])]
    if not any(pair['hardware_gap_us'] for pair in pairs):
        raise ValueError('Hardware timestamps do not advance')
    inside = [row for row in events
              if 0 < ((row['callback_ms']-gap_start_ms) & 0xffffffff) < interval]
    return dict(radio_frame_count=len(events), radio_retry_flag_count=sum(row['retry'] for row in events),
        observed_frame_lengths=dict(Counter(row['frame_length'] for row in events)),
        observed_qos_tids=dict(Counter(str(row['tid']) if row['tid'] is not None else 'unavailable'
                                      for row in events)),
        max_callback_gap_ms=max(pair['callback_gap_ms'] for pair in pairs),
        max_hardware_gap_us=max(pair['hardware_gap_us'] for pair in pairs),
        largest_callback_gap=max(pairs, key=lambda pair: pair['callback_gap_ms']),
        app_gap_start_ms=gap_start_ms, app_gap_end_ms=gap_end_ms,
        radio_callbacks_strictly_inside_app_gap=inside,
        trailing_radio_callback_silence_at_fault_ms=(
            (fault_ms-events[-1]['callback_ms']) & 0xffffffff if fault_ms is not None else None),
        events=events,
        limitations='Only AP-to-own-STA data headers with original source matching the PC MAC. Includes ARP/retries, not one-to-one MC-Link packets. SDK metadata is not an independent over-air capture. Promiscuous observation can perturb driver behavior.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observation', type=Path, required=True)
    parser.add_argument('--serial-json', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    observation = json.loads(args.observation.read_text())
    chunks = json.loads(args.serial_json.read_text())
    application = analyze_application(observation, chunks)
    gap = application['largest_completed_esp_rx_gap']
    if not gap:
        raise ValueError('Need a completed application RX interval')
    rx = {event['sequence']: event['at_ms'] for event in application['timing']['esp_events']
          if event['kind'] == 1}
    radio = summarize_radio(''.join(row['text'] for row in chunks),
        rx[gap['from_sequence']], rx[gap['to_sequence']], application['timing']['esp_fault']['esp_ms'])
    result = dict(application=application, radio=radio)
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps(dict(first_fault_host_s=application['first_fault_host_s'], application_gap=gap,
        radio={k: v for k, v in radio.items() if k != 'events'}), indent=2))


if __name__ == '__main__':
    main()
