"""Same audited passive reader, with this SIM's pinned rebuilt Sign data."""
import multiprocessing as mp
import os
from live_sign import SoftwarePassiveReader
from mecanum_wand_4wsb_hardware.source_relay import passive_child

def child(channel,stop,ready):
    os.environ['ROS_DOMAIN_ID']='0'
    from geometry12 import install_geometry
    install_geometry();passive_child(channel,stop,ready)

class Reader(SoftwarePassiveReader):
    def __init__(self):
        super().__init__()
        self.process=mp.get_context('spawn').Process(target=child,args=(self.channel,self.stop,self.ready),
            name='sim12-rebuilt-sign-readonly',daemon=True)
