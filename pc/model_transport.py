"""Pinned SSH to the bound controller, independent of UI feedback freshness.

Every remote operation checks the hardware serial before running its source.
The remote runtime owns motion admission; lost replies are never replayed.
"""
from device_config import DEVICE_SERIAL
import json
from pathlib import Path
import shlex
import subprocess
import sys
from urllib.parse import urlsplit


def _configuration(root):
    root=Path(root).resolve()
    tool_path=str(root/'tools')
    if tool_path not in sys.path:sys.path.insert(0,tool_path)
    import deploy_readonly_wifi as update
    import discover_unoq as discovery
    try:
        identity=update.strict_identity(root)
        if identity.get('serial')!=DEVICE_SERIAL:raise ValueError('controller_identity_mismatch')
        choices=discovery.cached_candidates(root)
        # The workbench can have authenticated a new DHCP address while an
        # older standalone discovery cache still points at the previous boot.
        connected=discovery.read_small_json(root/'qa/official-connection.json')
        if (connected and connected.get('ssh_pin_verified') is True
                and connected.get('sys_serial')==identity['serial']):
            current=discovery.literal_url(connected.get('network_url'))
            if current:choices=[current]+[u for u in choices if u!=current]
        if not choices:raise ValueError('controller_address_missing')
        target=discovery.literal_url(choices[0])
        if not target:raise ValueError('controller_address_invalid')
        executable,null_config=discovery.ssh_transport(root)
    except (update.UpdateError,discovery.DiscoveryError) as error:
        raise RuntimeError('controller_transport_configuration_invalid') from error
    return identity,urlsplit(target).hostname,executable,null_config


# Keep the SSH command independent of source size. Windows/Git SSH may truncate
# long -c arguments; both program and request are streamed over stdin instead.
BOOTSTRAP = """import io,json,sys
from pathlib import Path
if Path('/sys/devices/soc0/serial_number').read_text().strip() != '__PORTABLE_DEVICE_SERIAL__':raise RuntimeError('controller_identity_mismatch')
envelope=json.load(sys.stdin)
sys.stdin=io.StringIO(json.dumps(envelope['payload']))
exec(compile(envelope['source'],'<duck-model-transport>','exec'))
"""


def execute(root,source,payload,timeout=15):
    identity,address,executable,null_config=_configuration(root)
    command=BOOTSTRAP.replace(repr('__PORTABLE_DEVICE_SERIAL__'),repr(identity['serial']))
    envelope={'source':source,'payload':payload}
    args=[str(executable),'-F',null_config,'-T','-o','BatchMode=yes','-o','IdentitiesOnly=yes',
          '-o','IdentityAgent=none','-o','StrictHostKeyChecking=yes','-o','UpdateHostKeys=no',
          '-o','VerifyHostKeyDNS=no','-o','CheckHostIP=no','-o','PasswordAuthentication=no',
          '-o','KbdInteractiveAuthentication=no','-o','ConnectTimeout=3','-o','ConnectionAttempts=1',
          '-o','GlobalKnownHostsFile='+null_config,'-o','UserKnownHostsFile='+str(identity['known_hosts_path']),
          '-o','HostKeyAlias='+identity['host_key_alias'],'-i',str(identity['key_path']),
          'arduino@'+address,'python3 -c '+shlex.quote(command)]
    try:
        result=subprocess.run(args,input=json.dumps(envelope).encode('utf-8'),stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,timeout=timeout,shell=False,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    except subprocess.TimeoutExpired:
        raise TimeoutError('主控 SSH 超时；操作结果待确认，未自动重试') from None
    except OSError:
        raise RuntimeError('主控 SSH 无法启动；操作未确认') from None
    if result.returncode:
        raise RuntimeError('主控 SSH 连接或远端执行失败；操作未确认')
    if len(result.stdout)>256*1024:raise RuntimeError('主控回复过大；操作未确认')
    try:value=json.loads(result.stdout)
    except (ValueError,UnicodeError):raise RuntimeError('主控回复无效；操作未确认') from None
    if not isinstance(value,dict):raise RuntimeError('主控回复格式无效；操作未确认')
    return value
