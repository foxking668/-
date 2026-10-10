"""Compare imported ordinary-lane implementation with the user's cc(1) archive."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def tokens(source):
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.S)
    return re.findall(r'\w+|[^\s]', source)


def check_archive(archive):
    imported = ROOT / 'legacy/cc_lane'
    manifest = json.loads((imported / 'source_sha256.json').read_text(encoding='utf-8'))
    checks = 0

    def check(condition, message):
        nonlocal checks
        if not condition:
            raise RuntimeError(message)
        checks += 1

    with zipfile.ZipFile(archive) as z:
        def original(path):
            return z.read('cc/src/' + path).decode('utf-8').replace('\r\n', '\n')

        for path, digest in manifest.items():
            check(hashlib.sha256(z.read('cc/src/' + path)).hexdigest() == digest, 'Different reference archive: ' + path)
            expected = original(path)
            if path.endswith('FindLine.cpp'):
                for field in ('left_line', 'center_line', 'right_line'):
                    old = 'delete this->' + field + ';'
                    check(expected.count(old) == 1, 'Unexpected original array ownership: ' + field)
                    expected = expected.replace(old, 'delete[] this->' + field + ';')
            check((ROOT / 'legacy' / path).read_text(encoding='utf-8') == expected, 'Imported source differs: ' + path)

        nt = original('NewTrack/NewTrack.cpp')

        def block(start, end):
            a = nt.index(start)
            return nt[a:nt.index(end, a)]

        expected = block('    static cv::Mat cropSafe(', '    static bool applyBlueObstacleToLane(')
        a = expected.index('        if(method==3)')
        b = expected.index('        // 725分支', a)
        expected = expected[:a] + expected[b:]
        expected = expected.replace('int method, double centerPratio, const Config &c', 'const Config &c')
        geometry = block('    static double centerToError(', '    static bool averageCenter(')
        check(geometry in (imported / 'steering_geometry.hpp').read_text(encoding='utf-8'), 'Original geometry was changed')
        expected = expected.replace(geometry, '    using cc_detail::centerToError;\n\n')
        expected += block('    struct BlackLineResult', '    static double medianWidth(')
        actual = (imported / 'lane_core.inc').read_text(encoding='utf-8')
        actual = actual[actual.index('    static cv::Mat cropSafe('):]
        check(actual == expected, 'NewTrack ordinary-lane helper differs from original')

        core = (imported / 'lane_core.inc').read_text(encoding='utf-8')
        config = dict(re.findall(r'(\w+)\s*=\s*(\d+)', core[:core.index('    static cv::Mat cropSafe(')]))
        params = z.read('cc/image_params.txt').decode('utf-8-sig')
        settings = dict(re.findall(r'^\s*(\w+)\s*=\s*([^#\s]+)', params, flags=re.M))
        bindings = dict(re.findall(r'k=="([^"]+)"\) I\(next\.(\w+)', nt))
        for name, value in config.items():
            default = re.search(r'\b' + name + r'\s*=\s*(\d+)', nt)
            check(default is not None and default.group(1) == value, 'Config default changed: ' + name)
            for key, bound in bindings.items():
                if bound == name and key in settings:
                    check(settings[key] == value, 'Archive runtime configuration differs: ' + key)
        for key, expected_value in {'center_pratio': '.42', 'new_track_lane_method': '4',
                                     'new_track_top_crop_ratio': '.05', 'new_track_bottom_crop_ratio': '.02',
                                     'new_track_left_crop_ratio': '0', 'new_track_right_crop_ratio': '0',
                                     'new_track_boundary_close_kernel': '11'}.items():
            # The config stores these as decimal numbers, so compare numerically.
            check(key in settings and float(settings[key]) == float(expected_value), 'Camera/config setting changed: ' + key)
        check(float(settings['center_pratio']) == .42, 'Missing original center_pratio')

        adapter = (ROOT / 'tools/reference_cc_lane.cpp').read_text(encoding='utf-8')
        a = nt.index('    else if(c.centerBlackEnable && black.reliable)', nt.index('    double targetX=lane.aggregateCenter;'))
        original_branch = nt[a:nt.index('    float error=', a)].replace('else if(', 'if(', 1)
        a = adapter.index('    if(c.centerBlackEnable && black.reliable)')
        adapter_branch = adapter[a:adapter.index('    // Same float', a)]
        check(tokens(original_branch) == tokens(adapter_branch), 'Target selection differs from original ordinary-lane branch')
        check('cv::INTER_NEAREST' in original('ImageProcess/Camera/Camera.cpp'), 'Original camera interpolation differs')
        defines = original('define.hpp')
        for axis, value in [('WIDTH', 80), ('HEIGHT', 60)]:
            check(re.search(r'#define\s+CAMERA_OPENCV_' + axis + r'\s+' + str(value) + r'\b', defines), 'Original frame dimensions differ: ' + axis)
        check('cv::INTER_NEAREST' in adapter and 'cv::Size(80,60)' in adapter, 'Adapter camera settings differ')
        check('cropSafe(cameraFrame,0.05,0.02,0.00,0.00)' in adapter, 'Adapter crop differs')
        check('cv::Size(11,1)' in adapter and 'cv::MORPH_CLOSE' in adapter, 'Boundary closing differs')
        check('(float)centerToError(targetX,rawMask.rows,rawMask.cols,0.42)' in adapter, 'Float conversion or optical target differs')
    print(f'{checks} CC source/config checks passed; only three array deletes repaired')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference-zip', required=True, type=Path)
    args = parser.parse_args()
    check_archive(args.reference_zip)
