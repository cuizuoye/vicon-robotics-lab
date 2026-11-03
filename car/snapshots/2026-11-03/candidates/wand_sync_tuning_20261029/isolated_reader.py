"""One-slot process-local IPC for passive Vicon input; no network motor socket.

The child performs the unchanged domain0 source/geometry checks. The control
process independently decodes the ORIGINAL packet and retains every stamp and
receipt time. IPC can only add age, never renew a source or motion authority.
"""
import copy
import multiprocessing as mp
import queue
import time

from model import ROOT  # Establish pinned import paths in a spawned process.
from packet_work import decode_packet


def publish_latest(channel, value):
    """Bounded latest input, not a command/reference FIFO. Never wait on a GUI."""
    try:
        channel.put_nowait(value)
        return True
    except queue.Full:
        try:
            channel.get_nowait()
        except queue.Empty:
            return False
        try:
            channel.put_nowait(value)
            return True
        except queue.Full:
            return False


def read_child(channel, stop, ready_consumed):
    from direct_reader import DirectReader
    reader = None
    try:
        reader = DirectReader()
        channel.put(dict(kind='ready', provenance=reader.provenance,
                         sent_steady=time.monotonic()), timeout=1.)
        if not ready_consumed.wait(timeout=8.):
            raise ValueError('PASSIVE_SOURCE_PARENT_HANDSHAKE_TIMEOUT')
        last = 0.
        while not stop.is_set():
            reader.executor.spin_once(timeout_sec=.001)
            now = time.monotonic()
            if now-last < .005:
                continue
            ros = reader.node.get_clock().now().nanoseconds*1e-9
            packet = reader(now, ros)
            publish_latest(channel, dict(kind='source', sent_steady=now,
                status=reader.status(), packet=packet.payload() if packet else None))
            last = now
    except Exception as error:
        publish_latest(channel, dict(kind='error', error=repr(error), sent_steady=time.monotonic()))
    finally:
        if reader is not None:
            reader.close()
        # No shutdown wait for a full IPC slot. It is passive data only.
        channel.cancel_join_thread()


class IsolatedReader:
    def __init__(self):
        context = mp.get_context('spawn')  # Never fork a running DDS participant.
        self.channel = context.Queue(maxsize=1)
        self.stop = context.Event()
        self.ready_consumed = context.Event()
        self.process = context.Process(target=read_child, args=(self.channel, self.stop, self.ready_consumed),
                                       name='vicon-passive-source', daemon=True)
        self.packet = None
        self.last_state = None
        self.rejected = 0
        self.raw_rejected = self.ipc_rejected = 0
        self.ipc_rejection_history = []
        self.error = ''
        self.provenance = None
        self.closed = False

    def start(self):
        self.process.start()
        # No private chain or motion authority exists during this handshake.
        first = self.channel.get(timeout=8.)
        if first.get('kind') != 'ready':
            self.close()
            raise ValueError('passive Vicon child did not become ready: '+repr(first))
        self.provenance = first['provenance']
        self.ready_consumed.set()

    def ingest(self, value, now, ros):
        if value.get('kind') != 'source':
            self.packet = None
            self.error = value.get('error', 'PASSIVE_SOURCE_IPC_INVALID')
            self.reject(self.error, now)
            return
        try:
            issued = value['sent_steady']
            if type(issued) not in (int, float) or not 0 <= now-issued <= .2:
                raise ValueError('PASSIVE_SOURCE_IPC_AGE')
            self.last_state = value
            self.raw_rejected = max(self.raw_rejected, value['status']['rejected'])
            self.rejected = self.raw_rejected+self.ipc_rejected
            self.packet = decode_packet(value['packet'], now, ros) if value['packet'] else None
            self.error = ''
        except (ValueError, TypeError, KeyError) as error:
            self.packet = None
            self.error = str(error)
            self.reject(self.error, now)

    def reject(self, reason, now):
        self.ipc_rejected += 1
        self.rejected = self.raw_rejected+self.ipc_rejected
        self.ipc_rejection_history.append(dict(at=now, reason=reason,
                                               ipc_rejected=self.ipc_rejected))
        self.ipc_rejection_history = self.ipc_rejection_history[-16:]

    def poll(self, clocks=None):
        # Receive BEFORE the source callback samples its command time. A child
        # may publish between an old caller's clock sample and Queue.get();
        # that is not a future/stale IPC fault. Check with the actual receipt
        # clock, then let Source sample its clocks AFTER this bounded poll.
        if clocks is None:
            clocks=lambda:(time.monotonic(), time.time())
        for _ in range(2):
            try:
                value = self.channel.get_nowait()
            except queue.Empty:
                break
            now,ros=clocks()
            self.ingest(value, now, ros)

    def __call__(self, now, ros):
        if self.packet is None or self.error or not self.process.is_alive():
            return None
        try:
            self.packet.pose(now, ros)
            if not self.last_state or not 0 <= now-self.last_state['sent_steady'] <= .2:
                return None
            return self.packet
        except ValueError:
            return None

    def status(self):
        now = time.monotonic()
        state = self.last_state
        fresh = bool(state and 0 <= now-state['sent_steady'] <= .2
                     and self.process.is_alive() and not self.error)
        value = copy.deepcopy(state['status']) if state else dict(flags={}, rejected=0,
            complete_packets=0, error='等待独立只读输入', domain0_command_publishers=[],
            car01_reference_only=dict(valid=False, pose=None, used_as_motion_feedback=False))
        value.update(rejected=max(value['rejected'], self.rejected),
                     raw_rejected=self.raw_rejected, ipc_rejected=self.ipc_rejected,
                     ipc_rejection_history=self.ipc_rejection_history.copy(),
                     isolated_reader=True, ipc_fresh=fresh,
                     ipc_age_s=now-state['sent_steady'] if state else None)
        if not fresh:
            value['flags'] = {k: False for k in value['flags']}
            value['car01_reference_only']['valid'] = False
            value['error'] = self.error or '独立只读接收未更新'
        else:
            car = value['car01_reference_only']
            stamp = car.get('stamp')
            car['valid'] = bool(car['valid'] and stamp and -.005 <= time.time()-stamp <= .2)
        return value

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop.set()
        self.ready_consumed.set()
        if self.process.pid:
            self.process.join(timeout=2.)
            if self.process.is_alive():
                self.process.terminate()  # Only our passive child; never a robot process.
                self.process.join(timeout=2.)
        self.channel.close()
        self.channel.cancel_join_thread()
