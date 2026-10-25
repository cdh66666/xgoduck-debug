import sys,unittest
from pathlib import Path
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from native_env import NativePool

class PoolCleanupTests(unittest.TestCase):
    def test_failed_worker_pipe_does_not_skip_other_worker_shutdown(self):
        pool=NativePool.__new__(NativePool)
        failed,healthy=Mock(),Mock();failed.send.side_effect=BrokenPipeError(232,'pipe closing')
        pool.connections=[failed,healthy];pool.processes=[Mock(),Mock()]
        for process in pool.processes:process.is_alive.return_value=True
        pool.close()
        healthy.send.assert_called_once_with(('close',None))
        for process in pool.processes:process.join.assert_called_once_with(timeout=15.)
        for connection in pool.connections:connection.close.assert_called_once_with()

if __name__=='__main__':unittest.main()
