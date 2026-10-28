"""Read-only X11 key bitmap; never changes desktop keyboard/autorepeat settings.

Core X11 autorepeat may emit Release/Press pairs separated by scheduling delay.
A fixed 5 ms debounce cannot identify a true physical release reliably. Querying
the server bitmap lets us reject those pairs and also detect a missed release.
This is an extra input check, never a replacement for focus/lease/deadline gates.
"""
import ctypes


class X11KeyState:
    def __init__(self, display_name):
        self.x11 = ctypes.CDLL('libX11.so.6')
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_ubyte
        self.x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_char)]
        self.x11.XQueryKeymap.restype = ctypes.c_int
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay.restype = ctypes.c_int
        self.display = self.x11.XOpenDisplay(display_name.encode())
        if not self.display: raise RuntimeError('cannot open read-only X11 keyboard connection')
        self.keycode = int(self.x11.XKeysymToKeycode(self.display, 0x20))
        if not 8 <= self.keycode <= 255:
            self.close()
            raise RuntimeError('space key is not mapped on this X11 display')

    @staticmethod
    def bitmap_down(bitmap, keycode):
        if len(bitmap) != 32 or not 8 <= keycode <= 255:
            raise ValueError('invalid X11 key bitmap/keycode')
        return bool(bitmap[keycode//8] & (1 << (keycode % 8)))

    def down(self):
        if not self.display: raise RuntimeError('X11 keyboard connection is closed')
        bitmap = ctypes.create_string_buffer(32)
        if not self.x11.XQueryKeymap(self.display, bitmap):
            raise RuntimeError('X11 keyboard query failed')
        return self.bitmap_down(bitmap.raw, self.keycode)

    def close(self):
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None
