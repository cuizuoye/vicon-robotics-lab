import struct
import pytest
from mecanum_vicon_control.readonly_link_status import decode_readonly_status, TAG
from mecanum_vicon_control.transport_protocol import ProtocolError


def test_readonly_status():
    v=[TAG,100,100,10,10,60]+[0]*8+[1,0]
    s=decode_readonly_status(struct.pack('<6I8I2H',*v))
    assert s['tag']=='4RO1' and s['compares']==[0]*8 and s['gpio_low']==1


@pytest.mark.parametrize('field,value',[(0,0),(3,0),(4,31),(5,1001),(6,1),(13,1),(14,0),(15,1)])
def test_unsafe_status_rejected(field,value):
    v=[TAG,100,100,10,10,60]+[0]*8+[1,0]
    v[field]=value
    with pytest.raises(ProtocolError):
        decode_readonly_status(struct.pack('<6I8I2H',*v))


def test_wrong_size_rejected():
    with pytest.raises(ProtocolError): decode_readonly_status(b'4RO1')
