"""快速刷写工具契约, 不打开串口/USB/SWD."""
import hashlib
from pathlib import Path
import struct
import sys
import tempfile
import tkinter
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'boards/sentinel_f407/tools'))
import firmware_tool_model as m

class Contracts(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='固件 test [路径] ')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.settings = m.Settings(arm_root=str(self.root / 'arm'), scripts=str(self.root / 'scripts'),
            cmake=str(self.root / 'cmake'), ninja=str(self.root / 'ninja'),
            sdk_root=str(self.root / 'sdk'), python=sys.executable, openocd=str(self.root / 'openocd'))
        for file in [self.settings.cmake, self.settings.ninja, self.settings.openocd,
            self.settings.arm('gcc'), self.settings.arm('objcopy'), self.settings.arm('nm'),
            self.root / 'cmake/arm-none-eabi.cmake',
            Path(self.settings.sdk_root) / 'Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h',
            Path(self.settings.scripts) / 'interface/stlink.cfg', Path(self.settings.scripts) / 'target/stm32f4x.cfg']:
            file = Path(file)
            # CMake可执行文件与cmake目录不能重名.
            if file == self.root / 'cmake':
                self.settings.cmake = str(self.root / 'cmake.exe')
                file = Path(self.settings.cmake)
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(b'fixture')

    def profile(self, key):
        return next(p for p in m.PROFILES if p.key == key)

    def test_sector_boundaries(self):
        self.assertEqual(m.sector_span(16384), (0, 16384))
        self.assertEqual(m.sector_span(16385), (1, 32768))
        self.assertEqual(m.sector_span(196152), (5, 262144))
        self.assertEqual(m.sector_span(1048576), (11, 1048576))
        for value in (0, 7, 1048577):
            with self.assertRaises(ValueError): m.sector_span(value)

    def test_profiles_and_build_flags(self):
        self.assertEqual(len({p.build_dir for p in m.PROFILES}), 9)
        for profile in m.PROFILES:
            calls = []
            m.build_firmware(self.root, profile, self.settings, lambda _: None,
                run=lambda args, *rest: calls.append(args) or '')
            args = calls[0]
            for flag in ('EC_BUILD_SENTINEL_APP', 'EC_BUILD_SENTINEL_CAPTURE', 'EC_BUILD_SENTINEL_S1',
                'EC_APP_GM6020_BENCH', 'EC_APP_FOUR_MOTOR_BENCH', 'EC_APP_CHASSIS_COMMISSIONING',
                'EC_APP_GIMBAL_COMMISSIONING', 'EC_APP_SHOOTER_COMMISSIONING', 'EC_S5_GM6020_BENCH'):
                self.assertIn(f'-D{flag}=' + ('ON' if flag in profile.flags else 'OFF'), args)
            if profile.key == 'capture':
                self.assertIn('-DEC_FIRMWARE_ID=cap-stream-1m-v1', args)
            if profile.key == 'four':
                self.assertTrue(next(a for a in args if a.startswith('-DEC_FIRMWARE_ID=')).endswith('-id3'))
            self.assertEqual(calls[1][-3:], [profile.target, '--parallel', '6'])
        self.assertIn('1000000', self.profile('capture').port)
        self.assertIn('921600', self.profile('four').port)
        self.assertEqual(self.profile('s5').audit_flags, ('--allow-gm6020-bench',))

    def prepare(self, key='four', label='app-four-motor-test-id3', audit_fail=False):
        elf = self.root / 'source.elf'
        elf.write_bytes(b'ELF fixture')
        def fake(args, cwd, log, timeout):
            if args[0] == self.settings.arm('objcopy'):
                data = bytearray(512)
                struct.pack_into('<II', data, 0, 0x20020000, 0x08000081)
                data[128:128+len(label)] = label.encode('ascii')
                Path(args[-1]).write_bytes(data)
                return ''
            if args[0] == self.settings.arm('nm'):
                return '08000080 R g_ec_s1_build_id\n'
            if audit_fail: raise RuntimeError('role rejected')
            return '{"result":"pass"}'
        return m.prepare_image(self.root, self.profile(key), elf, self.settings, self.root / 'stage', lambda _: None, fake)

    def test_audited_staging_and_retired_images(self):
        image = self.prepare()
        self.assertEqual(image.label, 'app-four-motor-test-id3')
        self.assertEqual(image.sha256, hashlib.sha256(image.binary.read_bytes()).hexdigest())
        self.assertTrue((image.binary.parent / 'image.json').is_file())
        for key, label in [('four','app-four-motor-20260926'), ('gm','app-gm6020-id4-20260925'), ('capture','capture-wrong')]:
            with self.assertRaises(ValueError): self.prepare(key, label)
        with self.assertRaises(RuntimeError): self.prepare(audit_fail=True)

    def test_flash_requires_exactly_one_probe_and_success_marker(self):
        image = self.prepare()
        calls = []
        def run(args, cwd, log, timeout):
            calls.append((args, cwd))
            return 'FLASH_TOOL_TARGET_OK\nFLASH_TOOL_VERIFIED\nFLASH_TOOL_SUCCESS\n'
        for count in (0, 2):
            with self.assertRaises(ValueError):
                m.flash_image(image, self.settings, lambda _: None, run, lambda *args: ['probe']*count)
        self.assertEqual(calls, [])
        m.flash_image(image, self.settings, lambda _: None, run, lambda *args: ['probe'])
        self.assertEqual(calls[0][1], image.binary.parent)
        self.assertIn('target/stm32f4x.cfg', calls[0][0])
        for output in ('', 'FLASH_TOOL_VERIFIED', 'FLASH_TOOL_FAILED\nFLASH_TOOL_SUCCESS'):
            with self.assertRaises(RuntimeError):
                m.flash_image(image, self.settings, lambda _: None, lambda *args: output, lambda *args: ['probe'])
        image.binary.write_bytes(b'changed')
        with self.assertRaises(ValueError):
            m.flash_image(image, self.settings, lambda _: None, run, lambda *args: ['probe'])

    def tcl_events(self, chip=0x10016413, kib=1024, fail=None):
        tcl = tkinter.Tcl()
        events = []
        for name in ('init', 'reset', 'echo', 'shutdown'):
            tcl.createcommand(name, lambda *args, name=name: events.append((name,*args)) or '')
        def read(command, address, width, count):
            events.append(('read', address, width))
            return str(chip if int(address,0)==0xe0042000 else kib)
        tcl.createcommand('stm32f4x.cpu', read)
        for name in ('flash','verify_image'):
            if name == fail:
                tcl.eval('proc ' + name + ' {args} {error {injected failure}}')
            else:
                tcl.createcommand(name, lambda *args, name=name: events.append((name,*args)) or '')
        tcl.eval(m.programming_script(5))
        return events

    def test_tcl_target_and_write_failure_paths(self):
        good = self.tcl_events()
        self.assertIn(('flash','write_image','image.bin','0x08000000','bin'), good)
        erased = [e for e in good if e[:2] == ('flash','erase_sector')]
        self.assertEqual(erased, [('flash','erase_sector','0',str(i),str(i)) for i in range(6)])
        self.assertIn(('verify_image','image.bin','0x08000000','bin'), good)
        self.assertIn(('reset','run'), good)
        self.assertLess(next(i for i,e in enumerate(good) if e[0]=='read'), next(i for i,e in enumerate(good) if e[0]=='flash'))
        for kwargs in (dict(chip=0x450), dict(kib=512)):
            events=self.tcl_events(**kwargs)
            self.assertFalse(any(e[0]=='flash' for e in events))
            self.assertIn(('shutdown','error'), events)
        for fail in ('flash','verify_image'):
            events=self.tcl_events(fail=fail)
            self.assertNotIn(('reset','run'), events)
            self.assertNotIn(('echo','FLASH_TOOL_SUCCESS'), events)
            self.assertIn(('shutdown','error'), events)
        script=m.programming_script(5)
        for value in (-1,12,'0; shutdown'):
            with self.assertRaises(ValueError): m.programming_script(value)
        for forbidden in ('mass_erase','option_write','unlock','dump_image','image.elf'):
            self.assertNotIn(forbidden,script)

    def test_host_commands_do_not_auto_authorize(self):
        for profile in m.PROFILES:
            path=self.root/profile.host
            path.parent.mkdir(parents=True,exist_ok=True);path.write_text('# fixture',encoding='utf-8')
            cmd=m.host_command(self.root,profile,self.settings,'COM3')
            self.assertEqual(cmd[-1],'COM3')
            self.assertNotIn('--authorize',cmd)
            self.assertNotIn('--target',cmd)

if __name__ == '__main__':
    unittest.main()
