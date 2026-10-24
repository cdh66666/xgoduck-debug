"""Pair native stability runs by seed/scenario and summarize measured changes."""
import argparse,json,statistics
from pathlib import Path


def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--baseline',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True)
 p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 baseline=json.loads(a.baseline.read_text('utf-8'));candidate=json.loads(a.candidate.read_text('utf-8'))
 if baseline['push_n']!=candidate['push_n']:raise ValueError('Force magnitudes must match')
 old={(r['seed'],r['scenario']):r for r in baseline['cases']}
 pairs=[(old[(r['seed'],r['scenario'])],r) for r in candidate['cases']]
 def change(before,after):
  return dict(baseline=before,candidate=after,change_percent=100*(after/before-1) if before and after is not None else None)
 def mean(rows,key):
  values=[r[key] for r in rows if r[key] is not None]
  return statistics.mean(values) if values else None
 output=dict(baseline=str(a.baseline),candidate=str(a.candidate),pairs=len(pairs),push_n=candidate['push_n'],
  completed_cases=sum(c['complete'] and not c['failure'] and c['resets_during_recording']==0 for _,c in pairs),scenarios={})
 stop_keys=['first_second_planar_path_m','first_second_yaw_path_rad','first_second_tilt_rms_deg','settling_s']
 for scenario in dict.fromkeys(c['scenario'] for _,c in pairs):
  before=[b for b,c in pairs if c['scenario']==scenario];after=[c for _,c in pairs if c['scenario']==scenario]
  metrics={key:change(mean(before,key),mean(after,key)) for key in ['max_tilt_deg','quiet_speed_rms_mps','quiet_yaw_rms_radps','quiet_tilt_rms_deg']}
  if before[0]['stops']:
   before_stops=[s for r in before for s in r['stops']];after_stops=[s for r in after for s in r['stops']]
   metrics['stops']={key:change(mean(before_stops,key),mean(after_stops,key)) for key in stop_keys}
   metrics['stops']['unsettled_count']=sum(s['settling_s'] is None for s in after_stops)
  output['scenarios'][scenario]=metrics
 before_stops=[s for b,_ in pairs for s in b['stops']];after_stops=[s for _,c in pairs for s in c['stops']]
 output['all_stops']={key:change(mean(before_stops,key),mean(after_stops,key)) for key in stop_keys}
 output['all_stops']['unsettled_count']=sum(s['settling_s'] is None for s in after_stops)
 a.output.parent.mkdir(parents=True,exist_ok=True)
 with a.output.open('x',encoding='utf-8') as f:json.dump(output,f,ensure_ascii=False,indent=2,allow_nan=False)
 print(json.dumps({k:v for k,v in output.items() if k!='scenarios'},ensure_ascii=False))


if __name__=='__main__':main()
