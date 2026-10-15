import base64
from pathlib import Path
import struct
import sys
import zlib

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from firmware_package_inspect import inspect


def package():
    image=struct.pack('<II',0x24010000,0x08020305)+b'example'
    return {'magic':'PX4FWv1','image':base64.b64encode(zlib.compress(image)).decode(),
            'image_size':len(image),'image_maxsize':1024,'board_id':1179,'git_hash':'abc'}


def test_matching_git_hash_does_not_authorize_mismatched_board_or_flash():
    data,report=inspect(package(),{'board_id':1178},'abc')
    assert len(data)==15 and report['source_commit_matches']
    assert not report['source_package_board_id_matches']
    assert not report['exact_vendor_build_reproduced'] and not report['flash_allowed']


def test_matching_package_headers_still_not_vendor_reproduction_or_approval():
    _,report=inspect(package(),{'board_id':1179},'abc')
    assert report['source_package_board_id_matches']
    assert not report['exact_vendor_build_reproduced'] and not report['flash_allowed']


@pytest.mark.parametrize('field,value',[('magic','no'),('image_size',16),('image_maxsize',8),('image','???')])
def test_invalid_packages_rejected(field,value):
    p=package();p[field]=value
    with pytest.raises(ValueError):inspect(p,{},'abc')


def test_invalid_reset_vector_rejected():
    p=package();image=struct.pack('<II',0x24010000,0x08020304)+b'example'
    p['image']=base64.b64encode(zlib.compress(image)).decode()
    with pytest.raises(ValueError,match='reset vector'):inspect(p,{},'abc')
