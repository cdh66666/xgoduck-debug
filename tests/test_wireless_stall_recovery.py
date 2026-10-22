"""Relay identity ownership and read-only stalled-channel recovery."""
import http.client,threading,unittest,json
from types import SimpleNamespace
from unittest.mock import patch
from official_control_proxy import WirelessTunnel
from official_status_stream import OfficialStatusStream

class Process:
 def __init__(self):self.terminated=0;self.dead=False
 def poll(self):return 0 if self.dead else None
 def terminate(self):self.terminated+=1;self.dead=True
 def wait(self,timeout=None):return 0

class OwnershipTests(unittest.TestCase):
 def tunnel(self):
  t=WirelessTunnel.__new__(WirelessTunnel);t.lock=threading.Lock();t.process=Process();t.port=9000;t.official=True;t.next_attempt=999;t.error=None
  return t
 def test_matching_owner_is_retired_without_restoring_commands(self):
  t=self.tunnel();owner=t.process
  self.assertTrue(t.invalidate_stalled(9000,owner));self.assertEqual(owner.terminated,1)
  self.assertIsNone(t.port);self.assertIsNone(t.process);self.assertFalse(t.official);self.assertEqual(t.next_attempt,0)
 def test_old_failure_cannot_kill_replacement_even_if_port_is_reused(self):
  t=self.tunnel();old=t.process;t.process=Process();replacement=t.process
  self.assertFalse(t.invalidate_stalled(9000,old));self.assertEqual(replacement.terminated,0)
  self.assertIs(t.process,replacement)
 def test_old_port_cannot_kill_current_owner(self):
  t=self.tunnel();self.assertFalse(t.invalidate_stalled(8999,t.process));self.assertEqual(t.process.terminated,0)
 def test_already_exited_owner_is_reaped_without_terminating_again(self):
  t=self.tunnel();old=t.process;old.dead=True
  self.assertTrue(t.invalidate_stalled(9000,old));self.assertEqual(old.terminated,0)
 def test_shutdown_clears_only_its_owned_process(self):
  t=self.tunnel();t.stopped=threading.Event();old=t.process;t.close()
  self.assertTrue(t.stopped.is_set());self.assertIsNone(t.process);self.assertIsNone(t.port);self.assertEqual(old.terminated,1)

class ReaderRecoveryTests(unittest.TestCase):
 def reader(self,tunnel):
  with patch.object(threading.Thread,'start'):
   r=OfficialStatusStream(tunnel,reconnect_delay=0)
  r._started_at=0
  return r
 def test_three_transport_timeouts_recover_same_owner_with_gets_only(self):
  owner=Process();t=SimpleNamespace(port=9000,process=owner,error=None,official=False);r=self.reader(t);paths=[];calls=[]
  def recover(port,process):calls.append((port,process));r._stop.set();return True
  t.invalidate_stalled=recover
  class Connection:
   sock=None
   def request(self,method,path):paths.append((method,path))
   def getresponse(self):raise TimeoutError('test timeout')
   def close(self):pass
  with patch('official_status_stream.http.client.HTTPConnection',side_effect=lambda *a,**k:Connection()):r._run()
  self.assertEqual(paths,[('GET','/api/status-stream')]*3);self.assertEqual(calls,[(9000,owner)]);self.assertEqual(r._relay_recoveries,1)
 def test_http_service_error_does_not_restart_ssh(self):
  owner=Process();calls=[];t=SimpleNamespace(port=9000,process=owner,error=None,official=False,invalidate_stalled=lambda *a:calls.append(a));r=self.reader(t);paths=[]
  class Response:
   status=503
   def close(self):pass
  class Connection:
   sock=None
   def request(self,method,path):
    paths.append((method,path))
    if len(paths)==4:r._stop.set()
   def getresponse(self):return Response()
   def close(self):pass
  with patch('official_status_stream.http.client.HTTPConnection',side_effect=lambda *a,**k:Connection()):r._run()
  self.assertEqual(calls,[]);self.assertEqual(len(paths),4)
 def test_sustained_backlog_drains_same_channel_without_ssh_restart(self):
  owner=Process();calls=[];t=SimpleNamespace(port=9000,process=owner,error=None,official=False,invalidate_stalled=lambda *a:calls.append(a));r=self.reader(t)
  def state(seq,uptime):return {'deployment':{'reference':'rl/raspi_deploy'},'mcu_seq':seq,'uptime_s':uptime}
  class Response:
   status=200
   def __init__(self,states):self.rows=iter(json.dumps(s).encode()+b'\n' for s in states)
   def getheader(self,*a):return 'application/x-ndjson'
   def readline(self,*a):
    row=next(self.rows,None)
    if row is None:r._stop.set();return b''
    return row
   def close(self):pass
  class Connection:
   sock=None
   def __init__(self,states):self.response=Response(states);self.closed=False
   def request(self,*a):pass
   def getresponse(self):return self.response
   def close(self):self.closed=True
  first=Connection([state(1,10),state(2,10.02),state(3,10.04),state(4,11.25)]);connections=iter([first]);clock=iter([1000.,1001.,1001.2,1001.25])
  with patch('official_status_stream.http.client.HTTPConnection',side_effect=lambda *a,**k:next(connections)),patch('official_status_stream.time.perf_counter',side_effect=lambda:next(clock,1001.25)):
   r._run()
  self.assertTrue(first.closed);self.assertEqual(r._connects,1);self.assertEqual(r._value['mcu_seq'],4);self.assertEqual(r._generation,2);self.assertEqual(r._backlog_drops,2);self.assertEqual(calls,[])

if __name__=='__main__':unittest.main()
