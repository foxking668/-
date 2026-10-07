"""Fetch upstream OpenCV source headers for Linux compile-only checking on Windows.
This DOES NOT install/link a target OpenCV library or validate Loongson hardware.
"""
from html.parser import HTMLParser
from pathlib import Path
import tarfile
import urllib.request
from urllib.parse import urljoin,urldefrag
import hashlib

class Links(HTMLParser):
    def __init__(self): super().__init__();self.urls=[]
    def handle_starttag(self,tag,attrs):
        if tag=='a': self.urls.extend(v for k,v in attrs if k=='href')

root=Path(__file__).resolve().parent/'validation_headers'
root.mkdir(exist_ok=True)
archive=root/'opencv-source.tar.gz'
if not archive.exists():
    with urllib.request.urlopen('https://pypi.tuna.tsinghua.edu.cn/simple/opencv-python/',timeout=30) as response:
        links=Links();links.feed(response.read().decode())
    url=next(u for u in links.urls if 'opencv-python-4.10.0.84.tar.gz' in u)
    url,fragment=urldefrag(urljoin('https://pypi.tuna.tsinghua.edu.cn/simple/opencv-python/',url))
    print('Downloading OpenCV 4.10 source for header verification',flush=True)
    urllib.request.urlretrieve(url,archive)
    if fragment.startswith('sha256=') and hashlib.sha256(archive.read_bytes()).hexdigest()!=fragment[7:]:
        archive.unlink();raise RuntimeError('Source archive checksum mismatch')
with tarfile.open(archive) as source:
    members=[m for m in source.getmembers() if any('/opencv/modules/'+name+'/include/' in m.name for name in ('core','imgproc','videoio','highgui'))]
    source.extractall(root,members=members,filter='data')
print('Headers extracted:',root)
