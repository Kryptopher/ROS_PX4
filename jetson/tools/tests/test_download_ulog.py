import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location('download_ulog', Path(__file__).parents[1] / 'download_ulog.py')
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


class DownloadTest(unittest.TestCase):
    def test_reordered_duplicate_packets(self):
        payload = b'ULog\x01\x12\x35' + b'x' * 173
        def packet(offset):
            return SimpleNamespace(id=3, ofs=offset, count=90, data=payload[offset:offset+90],
                                   get_type=lambda: 'LOG_DATA')
        with TemporaryDirectory() as folder:
            target = Path(folder) / 'test.ulg'
            with patch.object(tool, 'receive', side_effect=[packet(90), packet(90), packet(0)]):
                tool.download(Mock(), 1, 1, SimpleNamespace(id=3, size=180), target)
            self.assertEqual(target.read_bytes(), payload)
            self.assertFalse(target.with_suffix('.ulg.part').exists())

    def test_bad_header_never_publishes_ulg(self):
        msg = SimpleNamespace(id=3, ofs=0, count=7, data=b'invalid', get_type=lambda: 'LOG_DATA')
        with TemporaryDirectory() as folder:
            target = Path(folder) / 'test.ulg'
            with patch.object(tool, 'receive', return_value=msg):
                with self.assertRaisesRegex(RuntimeError, 'not a ULog'):
                    tool.download(Mock(), 1, 1, SimpleNamespace(id=3, size=7), target)
            self.assertFalse(target.exists())
            self.assertTrue(target.with_suffix('.ulg.part').exists())

    def test_armed_heartbeat_aborts(self):
        msg = Mock(base_mode=128)
        msg.get_srcSystem.return_value = 1
        msg.get_srcComponent.return_value = 1
        msg.get_type.return_value = 'HEARTBEAT'
        link = Mock()
        link.recv_match.return_value = msg
        with self.assertRaisesRegex(RuntimeError, 'armed'):
            tool.receive(link, ['HEARTBEAT', 'LOG_DATA'], 1, 1, 1)


if __name__ == '__main__':
    unittest.main()
