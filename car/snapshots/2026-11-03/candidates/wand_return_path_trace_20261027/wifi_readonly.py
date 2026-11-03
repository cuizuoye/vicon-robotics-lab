"""Unprivileged GET-only nl80211 statistics. No scans or configuration calls."""
import errno
import json
from pathlib import Path
import socket
import struct
import subprocess
import time

INTERFACE='wlx000000000000'
GETS={'NL80211_CMD_GET_INTERFACE','NL80211_CMD_GET_STATION','NL80211_CMD_GET_POWER_SAVE'}


def constants(folder):
    out=Path(folder)/'uapi_constants'
    subprocess.run(['cc','-Wall','-Wextra','-Werror',str(Path(__file__).with_name('uapi_constants.c')),
        '-o',str(out)],check=True,capture_output=True,text=True)
    return json.loads(subprocess.run([str(out)],check=True,capture_output=True,text=True).stdout)


def attr(kind,data):
    n=4+len(data)
    return struct.pack('=HH',n,kind)+data+b'\0'*((-n)%4)


def attrs(data):
    result={};at=0
    while at<len(data):
        if len(data)-at<4:raise ValueError('truncated netlink attribute header')
        size,kind=struct.unpack_from('=HH',data,at)
        if size<4 or at+size>len(data):raise ValueError('invalid netlink attribute length')
        result[kind&0x3fff]=data[at+4:at+size];at+=(size+3)&~3
    return result


class ReadOnlyWifi:
    def __init__(self,c):
        self.c=c;self.sequence=0
        self.sock=socket.socket(socket.AF_NETLINK,socket.SOCK_RAW,16)  # NETLINK_GENERIC
        self.sock.bind((0,0));self.sock.settimeout(.4)
        values=self.request(c['GENL_ID_CTRL'],c['CTRL_CMD_GETFAMILY'],
            attr(c['CTRL_ATTR_FAMILY_NAME'],b'nl80211\0'))
        self.family=struct.unpack('=H',values[0][c['CTRL_ATTR_FAMILY_ID']])[0]

    def close(self):self.sock.close()

    def request(self,family,command,payload=b'',dump=False):
        # This final request gate is independent of every caller. No arbitrary
        # netlink command, SET, scan, association, power or interface mutation.
        c=self.c
        if family==c['GENL_ID_CTRL']:
            if command!=c['CTRL_CMD_GETFAMILY']:raise ValueError('only GETFAMILY allowed')
        elif family!=getattr(self,'family',None) or command not in {c[k] for k in GETS}:
            raise ValueError('only interface/station/power GET allowed')
        self.sequence+=1
        body=struct.pack('=BBH',command,1,0)+payload
        flags=c['NLM_F_REQUEST']|(c['NLM_F_DUMP'] if dump else 0)
        self.sock.sendto(struct.pack('=IHHII',16+len(body),family,flags,
            self.sequence,self.sock.getsockname()[0])+body,(0,0))
        result=[];deadline=time.monotonic()+.5
        while time.monotonic()<deadline:
            data=self.sock.recv(65536);at=0
            while at<len(data):
                if len(data)-at<16:raise ValueError('truncated netlink message header')
                size,kind,flags,seq,pid=struct.unpack_from('=IHHII',data,at)
                if size<16 or at+size>len(data):raise ValueError('invalid netlink message size')
                body=data[at+16:at+size];at+=(size+3)&~3
                if seq!=self.sequence:continue
                if kind==c['NLMSG_ERROR']:
                    error=struct.unpack_from('=i',body)[0]
                    if error:raise OSError(-error,errno.errorcode.get(-error,'netlink error'))
                    continue
                if kind==c['NLMSG_DONE']:return result
                if kind!=family or len(body)<4:raise ValueError('unexpected netlink response')
                result.append(attrs(body[4:]))
                if not flags&c['NLM_F_MULTI']:return result
        raise TimeoutError('GET-only netlink reply deadline')

    def get(self,name,dump=False):
        c=self.c
        return self.request(self.family,c[name],attr(c['NL80211_ATTR_IFINDEX'],
            struct.pack('=I',socket.if_nametoindex(INTERFACE))),dump)

    def snapshot(self):
        c=self.c;result={}
        for name,dump in (('NL80211_CMD_GET_INTERFACE',False),
                          ('NL80211_CMD_GET_STATION',True),
                          ('NL80211_CMD_GET_POWER_SAVE',False)):
            try:
                rows=self.get(name,dump)
                if name.endswith('GET_STATION'):
                    result['stations']=[self.station(r) for r in rows]
                elif name.endswith('GET_POWER_SAVE'):
                    values=[struct.unpack('=I',r[c['NL80211_ATTR_PS_STATE']])[0] for r in rows]
                    result['power_save_states']=values
                    result['power_save_disabled']=values==[c['NL80211_PS_DISABLED']]
                else:
                    result['interfaces']=[dict(name=r.get(c['NL80211_ATTR_IFNAME'],b'').rstrip(b'\0').decode(),
                        frequency_mhz=(struct.unpack('=I',r[c['NL80211_ATTR_WIPHY_FREQ']])[0]
                            if c['NL80211_ATTR_WIPHY_FREQ'] in r else None)) for r in rows]
            except (OSError,ValueError,KeyError,struct.error,TimeoutError) as error:
                result[name+'_error']=str(error)
        return result

    def station(self,row):
        c=self.c;values=attrs(row[c['NL80211_ATTR_STA_INFO']]);out={}
        mac=row.get(c['NL80211_ATTR_MAC'])
        if mac:out['mac']=':'.join(f'{x:02x}' for x in mac)
        for name,fmt in (('SIGNAL','b'),('SIGNAL_AVG','b'),('INACTIVE_TIME','I'),
            ('RX_BYTES64','Q'),('TX_BYTES64','Q'),('RX_PACKETS','I'),('TX_PACKETS','I'),
            ('TX_RETRIES','I'),('TX_FAILED','I'),('RX_DROP_MISC','Q'),('BEACON_LOSS','I'),
            ('CONNECTED_TIME','I'),('RX_MPDUS','I'),('FCS_ERROR_COUNT','I'),('BEACON_RX','Q')):
            key=c['NL80211_STA_INFO_'+name]
            if key in values:out[name.lower()]=struct.unpack('='+fmt,values[key])[0]
        return out


def system_snapshot():
    def read(path):
        try:return Path(path).read_text().strip()
        except OSError:return None
    before=time.monotonic_ns()
    p=subprocess.run(['ethtool','-S',INTERFACE],capture_output=True,text=True,timeout=.5)
    stats={}
    if p.returncode==0:
        for line in p.stdout.splitlines()[1:]:
            key,_,value=line.strip().partition(':')
            if value.strip().isdigit():stats[key]=int(value)
    return dict(started_mono_ns=before,finished_mono_ns=time.monotonic_ns(),
        ethtool=stats,ethtool_error=p.stderr.strip() if p.returncode else '',
        proc_wireless=read('/proc/net/wireless'),
        interface={k:read('/sys/class/net/'+INTERFACE+'/statistics/'+k) for k in
            ('rx_packets','rx_errors','rx_dropped','tx_packets','tx_errors','tx_dropped')},
        usb={k:read('/sys/bus/usb/devices/3-3.4/power/'+k) for k in
            ('control','runtime_status','runtime_active_time','runtime_suspended_time')})
