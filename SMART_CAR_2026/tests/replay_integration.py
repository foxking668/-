"""Check actual Python -> C++ frame processing on image AND recorded video."""
from pathlib import Path
import json
import csv
import math
import subprocess
import sys
import os
import cv2
import numpy as np

root=Path(__file__).resolve().parents[1]
temporary=root/'build'/'replay-fixtures'
temporary.mkdir(parents=True,exist_ok=True)
image=np.full((240,320,3),(210,90,0),np.uint8)
image[:,48:272]=(240,240,240)
image[:,156:164]=(0,0,0)
picture=temporary/'line.png'
ok,encoded=cv2.imencode('.png',image);assert ok
encoded.tofile(picture)
video=temporary/'line.avi'
os.chdir(temporary)
writer=cv2.VideoWriter(video.name,cv2.VideoWriter_fourcc(*'MJPG'),20,(320,240))
assert writer.isOpened()
for _ in range(5): writer.write(image)
writer.release()
os.chdir(root)
for source,count in [(picture,1),(video,5)]:
    output=temporary/(source.suffix[1:]+'.jsonl')
    subprocess.run([sys.executable,'tools/replay.py',str(source),'--output',str(output)],cwd=root,check=True)
    rows=[json.loads(line) for line in output.read_text().splitlines()]
    assert len(rows)==count and all(r['line_confidence']>.9 and r['speed']==0 for r in rows)
telemetry_csv=temporary/'line.csv'
telemetry_csv.write_text('time,distance,speed,yaw,yaw_valid,yaw_measured\n'+''.join(f'{i*.05},0,0,0,1,1\n' for i in range(5)))
output=temporary/'mission.jsonl'
subprocess.run([sys.executable,'tools/replay.py',str(video),'--telemetry',str(telemetry_csv),'--output',str(output)],cwd=root,check=True)
rows=[json.loads(line) for line in output.read_text().splitlines()]
assert len(rows)==5 and all(r['stage']=='Depart' and r['speed']>0 for r in rows)
for base_time in (1234567.0, 1000000000.0):
    timestamps=[base_time+i*.05 for i in range(5)]
    precision_csv=temporary/'precision.csv'
    with precision_csv.open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(('time','distance','speed','yaw','yaw_valid','yaw_measured'))
        for timestamp in timestamps:
            writer.writerow((timestamp,0,0,0,1,1))
    precision_output=temporary/'precision.jsonl'
    subprocess.run([sys.executable,'tools/replay.py',str(video),'--telemetry',str(precision_csv),
                    '--output',str(precision_output)],cwd=root,check=True)
    precision_rows=[json.loads(line) for line in precision_output.read_text().splitlines()]
    assert [row['time'] for row in precision_rows]==timestamps, 'Replay timestamps lost precision'
    assert all(math.isclose(precision_rows[i]['time']-precision_rows[i-1]['time'],.05,abs_tol=2e-7)
               for i in range(1,len(precision_rows))), 'Replay frame intervals changed'
print('PASS image/video perception replay, synchronized telemetry and large timestamp precision')

# Real recorder timing must not enable Mission or invent measured heading.
recorded_times=[.741234567, .943210987, 1.144567891, 1.342198765, 1.543210987]
frame_csv=temporary/'frames.csv'
def write_frame_times(values):
    with frame_csv.open('w',encoding='utf-8',newline='') as stream:
        writer=csv.writer(stream); writer.writerow(('frame_index','elapsed_s'))
        writer.writerows(enumerate(values))
write_frame_times(recorded_times)
frame_output=temporary/'recorded-times.jsonl'
frame_command=[sys.executable,'tools/replay.py',str(video),'--frame-times',str(frame_csv),
               '--stage','GarageReverse','--output',str(frame_output)]
subprocess.run(frame_command,cwd=root,check=True)
rows=[json.loads(line) for line in frame_output.read_text().splitlines()]
assert [r['time'] for r in rows]==recorded_times
assert all(r['reason']=='PERCEPTION_ONLY' and r['speed']==0 and r['steer']==0 for r in rows)
for values in ([.7,.9,.8,1.1,1.3],[.7,.9,float('nan'),1.1,1.3],recorded_times[:4],recorded_times+[1.7]):
    write_frame_times(values)
    result=subprocess.run(frame_command,cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    assert result.returncode!=0,'Invalid recorder clock/count was accepted'
write_frame_times(recorded_times)
result=subprocess.run(frame_command+['--telemetry',str(telemetry_csv)],cwd=root,
                      stdout=subprocess.PIPE,stderr=subprocess.PIPE)
assert result.returncode!=0,'Frame times and vehicle telemetry must be mutually exclusive'
print('PASS real recorder timing, perception-only output, invalid clocks/counts and conflicting inputs')

# Observer suggestions are diagnostic fields; executed mission commands remain zero.
for motion in ('forward','reverse'):
    subprocess.run(frame_command+['--observe-steering',motion],cwd=root,check=True)
    observed=[json.loads(line) for line in frame_output.read_text().splitlines()]
    assert all(r['speed']==r['steer']==r['actuator_writes']==0 for r in observed)
    assert [r['time'] for r in observed]==recorded_times
    assert all(r['suggested_command'] is None for r in observed[:2])
    assert all(r['suggestion_valid'] and r['reference_id']==1 and abs(r['suggested_command'])<1e-9
               for r in observed[2:])
result=subprocess.run(frame_command+['--observe-steering','reverse','--telemetry',str(telemetry_csv)],
                      cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
assert result.returncode!=0
result=subprocess.run(frame_command+['--observe-steering','forward','--stage','Depart'],
                      cwd=root,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
assert result.returncode!=0
print('PASS shared observer replay, preserved timestamps, initial reference confirmation and zero executed commands')
