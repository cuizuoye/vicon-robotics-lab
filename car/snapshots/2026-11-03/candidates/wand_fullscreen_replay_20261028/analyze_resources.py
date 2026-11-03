"""Read recorded metrics only; no window, ROS or hardware access."""
import json
from pathlib import Path
import sys

from replay_diagnostic import quantiles


def analyze(directory):
    directory=Path(directory)
    result={}
    for mode in ('window','fullscreen'):
        ui_path=directory/(mode+'_ui.jsonl')
        if not ui_path.exists():continue
        ui=[json.loads(x) for x in ui_path.read_text().splitlines()]
        system=[json.loads(x) for x in (directory/(mode+'_system.jsonl')).read_text().splitlines()]
        probe=[json.loads(x) for x in (directory/(mode+'_x11_probe.jsonl')).read_text().splitlines()]
        if not ui or not system:continue
        details={}
        details['cycle_counts_over_ms']={str(threshold):sum((r.get('gap_ms') or 0)>threshold for r in ui)
                                        for threshold in (50,100,200,500,1000)}
        details['draw_cpu_ms']=quantiles([r.get('draw_cpu_ms') for r in ui])
        details['draw_map_ms']=quantiles([r.get('map_ms') for r in ui])
        details['draw_map_cpu_ms']=quantiles([r.get('map_cpu_ms') for r in ui])
        details['slow_cycles']=[]
        for i,row in enumerate(ui):
            if (row.get('gap_ms') or 0)>100:
                previous=ui[max(0,i-1)]
                nearby=[p['rtt_ms'] for p in probe if previous['steady']-.1<=p['steady']<=row['steady']+.1]
                details['slow_cycles'].append(dict(elapsed_s=row['elapsed_s'],
                    gap_ms=row['gap_ms'],previous_draw_ms=previous['draw_ms'],
                    previous_draw_cpu_ms=previous.get('draw_cpu_ms'),
                    previous_map_ms=previous.get('map_ms'),
                    previous_map_cpu_ms=previous.get('map_cpu_ms'),
                    previous_key_ms=previous['key_ms'],
                    independent_x11_max_rtt_ms=max(nearby,default=None)))
        total_busy=[];by_process={name:[] for name in ('ui','Xorg','gnome-shell')}
        for before,after in zip(system,system[1:]):
            first=list(map(int,before['cpu_stat'].split()[1:]))
            last=list(map(int,after['cpu_stat'].split()[1:]))
            delta=[b-a for a,b in zip(first,last)]
            total=sum(delta[:8])
            if total:total_busy.append(100*(1-(delta[3]+delta[4])/total))
            for name in by_process:
                p,q=before['processes'].get(name,{}),after['processes'].get(name,{})
                if 'cpu_ticks' in p and 'cpu_ticks' in q:
                    import os
                    by_process[name].append((q['cpu_ticks']-p['cpu_ticks'])/os.sysconf('SC_CLK_TCK')/
                                            (after['steady']-before['steady'])*100)
        details['system_cpu_busy_percent']=quantiles(total_busy)
        details['process_cpu_one_core_percent']={name:quantiles(values) for name,values in by_process.items()}
        details['rss_after_10s_last_mib']={}
        for name in by_process:
            rss=[int(r['processes'][name]['VmRSS'].split()[0])/1024 for r in system
                 if 'VmRSS' in r['processes'].get(name,{}) and r['steady']>=ui[0]['steady']+10]
            details['rss_after_10s_last_mib'][name]=dict(first=rss[0],last=rss[-1],growth=rss[-1]-rss[0]) if rss else None
        details['swap_used_kib_max']=max(r['swap_used_kib'] for r in system)
        cpu_temps=[temp for r in system for key,temp in r['temperatures_c'].items() if '/hwmon9/' in key]
        details['cpu_temp_c_max']=max(cpu_temps,default=None)
        details['intel_thermal_throttle_samples']=sum(r['intel_gt'].get('throttle_reason_thermal',0)!=0 for r in system)
        details['input_events_consumed']=ui[-1]['events_consumed']
        details['max_control_io_attempts']=max(r['control_io_attempts'] for r in ui)
        details['gui_wall_s']=ui[-1]['elapsed_s']
        result[mode]=details
    return result


if __name__=='__main__':
    rendered=json.dumps(analyze(sys.argv[1]),ensure_ascii=False,indent=2)+'\n'
    if len(sys.argv)==3:
        Path(sys.argv[2]).write_text(rendered)
    else:
        print(rendered,end='')
