"""Only this generated package may provide 4WSC runtime symbols."""
import os
from pathlib import Path
import sys

READY=Path(__file__).resolve().parent
ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
for name in ('mecanum_wand_4wsc_control','mecanum_wand_4wsc_hardware'):
    sys.path.insert(0,str(READY/'runtime/src'/name))
sys.path.insert(0,str(ROOT/'src/mecanum_vicon_control'))

def runtime():
    from mecanum_wand_4wsc_hardware.commissioning import runtime_from_environment
    rt=runtime_from_environment()
    import mecanum_wand_4wsc_hardware as h
    if not Path(h.__file__).resolve().is_relative_to(READY/'runtime/src'):raise ValueError('private package fallback forbidden')
    return rt

def install():
    from geometry import install_geometry
    install_geometry()
