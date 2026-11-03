import pytest
import time
from types import SimpleNamespace

from mecanum_wand_control.x11_key_state import X11KeyState
from mecanum_wand_control.operator_console import Console
from mecanum_wand_control.operator_model import OperatorModel


@pytest.mark.parametrize('keycode', [8, 32, 65, 127, 128, 255])
def test_bitmap_is_key_specific(keycode):
    bitmap = bytearray(32)
    assert not X11KeyState.bitmap_down(bitmap, keycode)
    bitmap[keycode//8] |= 1 << (keycode % 8)
    assert X11KeyState.bitmap_down(bitmap, keycode)
    neighbour = keycode+1 if keycode < 255 else keycode-1
    assert not X11KeyState.bitmap_down(bitmap, neighbour)


@pytest.mark.parametrize('bitmap,code', [(b'', 65), (bytes(31), 65), (bytes(32), 7), (bytes(32), 256)])
def test_invalid_bitmap_fails_closed(bitmap, code):
    with pytest.raises(ValueError): X11KeyState.bitmap_down(bitmap, code)


def test_query_of_closed_connection_is_not_held():
    query = X11KeyState.__new__(X11KeyState)
    query.display = None
    with pytest.raises(RuntimeError): query.down()
    query.close()


def ready_console(down):
    app = Console.__new__(Console)
    app.model = OperatorModel()
    now = time.monotonic()
    app.model.focused = True
    app.model.receive_status(dict(state='ARMED', gate_ready=True), now)
    app.model.receive_gate(dict(fault='', hard_fault=False), now)
    app.key_state = SimpleNamespace(down=down)
    app.release_job, app.closing = None, False
    app.node = SimpleNamespace(heartbeat=lambda: None)
    return app


def test_repeat_release_cannot_clear_held_or_create_another_edge():
    app = ready_console(lambda: True)
    app.press()
    assert app.model.held
    for _ in range(3):
        app.release()  # Even if the matching repeat Press arrives much later.
        assert app.model.held and app.model.key_down
        app.press()
    assert sum(e['event'] == 'enable_pressed' for e in app.model.events) == 1


def test_real_release_revokes_without_debounce_delay():
    app = ready_console(lambda: True)
    app.press()
    app.key_state = SimpleNamespace(down=lambda: False)
    app.release()
    assert not app.model.held and not app.model.key_down


def test_queued_press_after_physical_release_cannot_enable():
    app = ready_console(lambda: False)
    app.press()
    assert not app.model.held


def test_failed_query_revokes_and_closes_instead_of_guessing_held():
    def broken(): raise RuntimeError('query failed')
    app = ready_console(lambda: True)
    app.press()
    app.key_state = SimpleNamespace(down=broken)
    app.node.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    app.close = lambda: setattr(app, 'closing', True)
    app.press()
    assert not app.model.held and app.closing and app.model.blocked
