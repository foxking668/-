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
