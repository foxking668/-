"""Replay images/video through the ACTUAL C++ perception and mission core (no copied logic)."""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='video, single image, or image directory')
    parser.add_argument('--exe', type=Path, default=Path('build/vision_stream.exe' if sys.platform=='win32' else 'build-linux/vision_stream'))
    parser.add_argument('--config', type=Path, default=Path('config/competition.ini'))
    parser.add_argument('--telemetry', type=Path, help='CSV: time,distance,speed,yaw,yaw_valid,yaw_measured; one row per frame')
    parser.add_argument('--stage', default='Depart', help='perception-only stage when no telemetry supplied')
    parser.add_argument('--output', type=Path, default=Path('replay.jsonl'))
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    try:
        import cv2
        import numpy as np
    except ImportError:
        parser.error('Install replay dependency first: python -m pip install opencv-python')
    def read_picture(path):
        # OpenCV imread on Windows may reject Chinese absolute paths.
        return cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_COLOR)
    capture = None
    if args.input.is_dir():
        pictures=sorted(p for p in args.input.iterdir() if p.suffix.lower() in {'.png','.jpg','.jpeg','.bmp','.ppm'})
        frames=(read_picture(p) for p in pictures)
        fps=20.
    elif args.input.suffix.lower() in {'.png','.jpg','.jpeg','.bmp','.ppm'}:
        frames=iter([read_picture(args.input)]);fps=20.
    else:
        capture=cv2.VideoCapture(str(args.input))
        if not capture.isOpened():
            parser.error('Cannot open input video')
        fps=capture.get(cv2.CAP_PROP_FPS) or 20.
        def video_frames():
            while True:
                ok, image=capture.read()
                if not ok: break
                yield image
        frames=video_frames()
    telemetry=None
    if args.telemetry:
        with args.telemetry.open(encoding='utf-8-sig',newline='') as f:
            telemetry=list(csv.DictReader(f))
        required={'time','distance','speed','yaw','yaw_valid','yaw_measured'}
        if not telemetry or not required.issubset(telemetry[0]):
            parser.error('Telemetry CSV missing required columns')
    command=[str(args.exe.resolve()),str(args.config.resolve())]
    if telemetry is None: command.append(args.stage)
    # Freeze processing dimensions for the run, as the C++ config is loaded once.
    width,height=320,240
    for line in args.config.read_text(encoding='utf-8-sig').splitlines():
        key,sep,value=line.split('#',1)[0].partition('=')
        if sep and key.strip()=='process_width': width=int(value.strip())
        if sep and key.strip()=='process_height': height=int(value.strip())
    process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE)
    count=0
    try:
        with args.output.open('w',encoding='utf-8') as output:
            for index,frame in enumerate(frames):
                if frame is None: raise ValueError('Unreadable input image')
                if telemetry is not None:
                    if index>=len(telemetry): raise ValueError('Telemetry shorter than image sequence')
                    row=telemetry[index]
                    values=[float(row[k]) for k in ('time','distance','speed','yaw','yaw_valid','yaw_measured')]
                else: values=[index/fps,0,0,0,0,0]
                # Replay uses the same configured processing dimensions as the vehicle.
                small=cv2.resize(frame,(width,height),interpolation=cv2.INTER_AREA)
                rgb=cv2.cvtColor(small,cv2.COLOR_BGR2RGB)
                # 17 significant digits preserve double telemetry, including
                # subsecond intervals on large steady_clock timestamps.
                header=(' '.join(f'{v:.17g}' for v in values)+f'\nP6\n{width} {height}\n255\n').encode('ascii')
                process.stdin.write(header+rgb.tobytes());process.stdin.flush()
                result=process.stdout.readline()
                if not result: raise RuntimeError('C++ replay process exited unexpectedly')
                data=json.loads(result);output.write(result.decode('utf-8').rstrip('\r\n')+'\n');count+=1
                if args.preview:
                    for name,color in [('path',(0,0,255)),('road_path',(0,255,0))]:
                        path=data[name]
                        for y,x in enumerate(path):
                            if x>=0: cv2.circle(frame,(int(x*frame.shape[1]),int(y/len(path)*frame.shape[0])),1,color,-1)
                    cv2.putText(frame,data['stage'],(10,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
                    cv2.imshow('2026 replay',frame)
                    if cv2.waitKey(1)==27: break
            if telemetry is not None and not args.preview and count!=len(telemetry):
                raise ValueError('Telemetry longer than image sequence')
        process.stdin.close()
        if process.wait(timeout=10)!=0: raise RuntimeError('C++ replay failed')
        print(f'Replayed {count} frames -> {args.output}')
    finally:
        if process.poll() is None: process.terminate();process.wait(timeout=10)
        if capture: capture.release()
        if args.preview: cv2.destroyAllWindows()

if __name__=='__main__': main()
