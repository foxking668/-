"""Verify packaging rejects corrupt/wrong builds without invoking hardware."""
import hashlib
import importlib.util
from pathlib import Path
import shutil
import tempfile
import unittest
import zipfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('package_auto_probe', ROOT / 'tools/package_auto_probe.py')
packager = importlib.util.module_from_spec(spec)
spec.loader.exec_module(packager)


class PackageTests(unittest.TestCase):
    def setUp(self):
        build = (ROOT / 'build').resolve()
        build.mkdir(exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix='probe-package-test-', dir=build)).resolve()
        self.directory.relative_to(build)  # Verify cleanup stays in this workspace.
        self.program = self.directory / 'built-recorder'
        self.archive = self.directory / 'verified.zip'

    def tearDown(self):
        shutil.rmtree(self.directory)

    def fake_elf(self, machine=258, version=None):
        data = bytearray(64)
        data[:6] = b'\x7fELF\x02\x01'
        data[18:20] = machine.to_bytes(2, 'little')
        return bytes(data) + (version or packager.source_version()).encode('ascii')

    def test_zero_header_rejected_even_at_expected_size(self):
        self.program.write_bytes(bytes(474656))
        with self.assertRaisesRegex(ValueError, 'not an ELF'):
            packager.package_program(self.program, self.archive)
        self.assertFalse(self.archive.exists())

    def test_wrong_machine_rejected(self):
        self.program.write_bytes(self.fake_elf(machine=62))
        with self.assertRaisesRegex(ValueError, 'LoongArch'):
            packager.package_program(self.program, self.archive)

    def test_old_version_does_not_replace_existing_archive(self):
        self.archive.write_bytes(b'previous verified package')
        self.program.write_bytes(self.fake_elf(version='2026-10-07.5'))
        with self.assertRaisesRegex(ValueError, 'rebuild first'):
            packager.package_program(self.program, self.archive)
        self.assertEqual(self.archive.read_bytes(), b'previous verified package')

    def test_roundtrip_and_checksums(self):
        data = self.fake_elf()
        self.program.write_bytes(data)
        output, manifest = packager.package_program(self.program, self.archive)
        self.assertEqual(output, self.archive)
        with zipfile.ZipFile(output) as archive:
            self.assertIsNone(archive.testzip())
            self.assertEqual(set(archive.namelist()), {packager.PROGRAM_NAME, packager.SCRIPT_NAME, packager.MANIFEST_NAME})
            self.assertEqual(archive.read(packager.PROGRAM_NAME), data)
            self.assertTrue(archive.read(packager.SCRIPT_NAME).startswith(b'#!/bin/sh\n'))
            for line in manifest.splitlines():
                checksum, filename = line.split('  ', 1)
                self.assertEqual(hashlib.sha256(archive.read(filename)).hexdigest(), checksum)
            self.assertEqual(archive.read(packager.MANIFEST_NAME), manifest.encode('ascii'))
        self.assertEqual(list(self.directory.glob('auto-probe-*.zip')), [])

    def test_output_cannot_overwrite_program(self):
        data = self.fake_elf()
        self.program.write_bytes(data)
        with self.assertRaisesRegex(ValueError, 'separate from the program'):
            packager.package_program(self.program, self.program)
        self.assertEqual(self.program.read_bytes(), data)


if __name__ == '__main__':
    unittest.main()
