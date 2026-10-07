"""Exercise the binary frame/JSON interface without any OpenCV dependency."""
import json
from pathlib import Path
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
exe=root/('build/vision_stream.exe' if sys.platform=='win32' else 'build-linux/vision_stream')
width,height=320,240
pixels=bytearray()
for y in range(height):
    for x in range(width):
        pixels.extend((0,0,0) if 156<=x<164 else (240,240,240) if 48<=x<272 else (0,90,210))
frame=b'0 0 0 0 0 0\nP6\n320 240\n255\n'+bytes(pixels)
result=subprocess.run([str(exe),str(root/'config/competition.ini'),'Depart'],input=frame,capture_output=True,check=True)
data=json.loads(result.stdout)
assert data['line_confidence']>.9 and data['speed']==0 and data['reason']=='PERCEPTION_ONLY'
assert not data['stripe'] and not data['cones']
bad=subprocess.run([str(exe),str(root/'config/competition.ini')],input=frame[:-10],capture_output=True)
assert bad.returncode!=0 and b'Truncated frame' in bad.stderr
print('PASS binary replay, JSON output, perception-only safety, truncated input')
