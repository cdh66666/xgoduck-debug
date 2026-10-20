"""User-owned controller identity; no real device values ship in this candidate."""
import json,os,re
from pathlib import Path
def configured_serial():
    value=os.environ.get('XGODUCK_SERIAL','').strip()
    path=Path(__file__).resolve().parent/'device.local.json'
    if not value and path.is_file():value=json.loads(path.read_text('utf-8')).get('serial','')
    if value and not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',value):raise ValueError('Invalid device serial')
    return value
DEVICE_SERIAL=configured_serial()
