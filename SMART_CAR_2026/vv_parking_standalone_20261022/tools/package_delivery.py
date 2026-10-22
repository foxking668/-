"""Create a source-only standalone delivery; never include old or host binaries."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def source_files():
    for path in sorted(ROOT.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(ROOT)
        if any(part.startswith("build") or part in {"runs", "__pycache__", ".git"} for part in relative.parts[:-1]):
            continue
        if path.name in {"vv", "vv_parking_test", ".parking_test.lock"} or path.suffix in {".exe", ".o", ".obj", ".pdb", ".zip", ".pyc"}:
            continue
        yield path


def write_zip(path, entries):
    with zipfile.ZipFile(path, "x", zipfile.ZIP_DEFLATED) as archive:
        for source, name in entries:
            archive.write(source, name)
    with zipfile.ZipFile(path) as archive:
        assert archive.testzip() is None
        assert all(not name.endswith((".exe", ".o", ".obj")) for name in archive.namelist())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    files = list(source_files())
    write_zip(output / "01_完整源码.zip", [(p, ROOT.name + "/" + p.relative_to(ROOT).as_posix()) for p in files])
    docs = ["image_params.txt", "README_先读.md", "VALIDATION.md", "ORIGIN_SHA256.json", "build_test.sh", "run_parking_test.sh"]
    write_zip(output / "02_当前参数与使用说明.zip", [(ROOT / p, p) for p in docs])
    checks = ["VALIDATION.md", "tests/check_extraction.py", "tests/standalone_flow_test.cpp", "vv/tests/parking_controller_test.cpp"]
    write_zip(output / "03_电脑验证记录与测试源码.zip", [(ROOT / p, p) for p in checks])
    (output / "先读_交付说明.md").write_text(
        "# VV 独立入库测试交付\n\n"
        "版本 VV_STANDALONE_20261022.1。基于 vv_20261022_1809_3a31be32.zip。\n\n"
        "流程：手动 Enter → 巡线找人行道 → 原反转及停车保持 → 原巡线前进4秒 → S2 → S4 → S5 → 停止。\n\n"
        "01 是独立完整源码，解压后先看根目录 README_先读.md；02 是相同当前参数和操作说明的便于查阅副本；03 是电脑验证范围和测试源码。三个包独立解压，不是分卷。\n\n"
        "本次不包含新的龙芯可执行程序。须在原 VV 编译环境编译新入口 vv_parking_test；不能用旧 vv 或 10月11日旧测试二进制代替。\n\n"
        "运行 bash ./run_parking_test.sh，初始化完成后 Enter 开始；Q+Enter 或 Ctrl+C 停止。--mode approach-only 同样先找人行道，在前进4秒后停止，便于单独检查前进终点。\n\n"
        "完整195项参数与上传包一致，S2/S4/S5总时间仍包含200ms停驱动；横道3200ms保持仍包含800ms反转。本次未加入巡线优化。\n\n"
        "已通过原控制器回归、新流程测试、参数与源码来源核对、Linux对象编译及脚本检查；未完成龙芯链接和实车验证，详见 VALIDATION.md。\n",
        encoding="utf-8",
    )
    sums = []
    for path in sorted(output.glob("*.zip")):
        sums.append(hashlib.sha256(path.read_bytes()).hexdigest() + "  " + path.name)
    (output / "SHA256SUMS.txt").write_text("\n".join(sums) + "\n", encoding="utf-8")
    summary = {"version": "VV_STANDALONE_20261022.1", "source_file_count": len(files),
               "new_loongarch_binary_included": False, "archives": sums}
    (output / "打包核对结果.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
