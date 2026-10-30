"""Pure image-descriptor fixtures; these are NOT physical deployment policies."""
from __future__ import annotations

import importlib.util
import struct
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / 'boards/sentinel_f407/check_sentinel_app.py'
spec = importlib.util.spec_from_file_location('pitch_image_audit_under_test', SCRIPT)
if spec is None or spec.loader is None:
    raise RuntimeError(f'Cannot load actual image auditor: {SCRIPT}')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def float_word(value: float) -> int:
    return struct.unpack('<I', struct.pack('<f', value))[0]


def image(enabled: bool) -> list[int]:
    values = [0x50494431, 1, int(enabled), int(enabled)] + [0] * 17
    values[14:21] = [float_word(0.0), float_word(-0.27), float_word(0.43),
                     0x00110102, 100, 200, 64]
    if enabled:
        values[4:14] = [float_word(0.02), float_word(0.10), float_word(0.03),
                        float_word(1.0), float_word(0.3), float_word(0.02),
                        100000, 3000000, 10000, 5000]
    return values


class DeploymentAuditTests(unittest.TestCase):
    def read(self, words: list[int], enabled: bool):
        return audit.read_pitch_summary(struct.pack('<21I', *words), 0x08000000, enabled)

    def test_default_disabled_and_explicit_enabled(self):
        self.assertEqual(self.read(image(False), False)['enabled'], 0)
        self.assertEqual(self.read(image(True), True)['require_capture'], 1)

    def test_selection_cannot_silently_enable_or_disable(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled), self.assertRaises(SystemExit):
                self.read(image(enabled), not enabled)

    def test_bounds_guards_and_identity_reject_mutation(self):
        for index, value in ((0, 0), (1, 2), (3, 0), (4, float_word(-0.5)),
                             (5, float_word(0.9)), (6, 0), (7, float_word(11)),
                             (8, float_word(float('inf'))), (9, float_word(float('nan'))),
                             (10, 0), (11, 0x80000000), (12, 0), (13, 5001),
                             (17, 0x00000101), (18, 150), (19, 500), (20, 44)):
            words = image(True)
            words[index] = value
            with self.subTest(index=index), self.assertRaises(SystemExit):
                self.read(words, True)

    def test_ordinary_image_rejects_latent_policy(self):
        words = image(False)
        words[6] = float_word(0.01)
        with self.assertRaises(SystemExit):
            self.read(words, False)

    def test_truncation_and_out_of_image_symbol(self):
        for data, address in ((b'\0' * 83, 0x08000000),
                              (b'\0' * 100, 0x07ffffff), (b'\0' * 100, 0x08000040)):
            with self.subTest(address=address), self.assertRaises(SystemExit):
                audit.read_pitch_summary(data, address, False)


if __name__ == '__main__':
    unittest.main()
