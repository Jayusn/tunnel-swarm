"""Tile experiment windows inside the X11 work area, including decorations."""
import ctypes
import ctypes.util
import os
import re
import subprocess
import time


def properties(environment, *arguments):
    return subprocess.check_output(['xprop', *arguments], env=environment, text=True, timeout=5)


def work_area(environment):
    text = properties(environment, '-root', '_NET_WORKAREA', '_NET_CURRENT_DESKTOP')
    area = re.search(r'_NET_WORKAREA\(CARDINAL\) = ([\d, ]+)', text)
    current = re.search(r'_NET_CURRENT_DESKTOP\(CARDINAL\) = (\d+)', text)
    if area:
        values = [int(x) for x in re.findall(r'\d+', area.group(1))]
        offset = 4 * (int(current.group(1)) if current else 0)
        if offset + 4 <= len(values):
            return tuple(values[offset:offset+4])
    raise RuntimeError('无法读取桌面可用区域，未调整窗口。')


def layout_rectangles(area, names):
    x, y, width, height = area
    margin, gap = 4, 8
    x += margin; y += margin; width -= margin*2; height -= margin*2
    if 'pixels' in names and any(n in names for n in ('camera', 'yolo')):
        left = int((width-gap)*.42)
        top = (height-gap)//2
        rectangles = {'camera': (x, y, left, top),
                      'yolo': (x, y+top+gap, left, height-top-gap),
                      'pixels': (x+left+gap, y, width-left-gap, height)}
    elif 'pixels' in names:
        rectangles = {'pixels': (x, y, width, height)}
    else:
        half = (width-gap)//2
        rectangles = {'camera': (x, y, half, height),
                      'yolo': (x+half+gap, y, width-half-gap, height)}
    return {n:r for n,r in rectangles.items() if n in names}


class ClientData(ctypes.Union):
    _fields_ = [('b', ctypes.c_char*20), ('s', ctypes.c_short*10), ('l', ctypes.c_long*5)]


class ClientMessage(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong), ('send_event', ctypes.c_int),
                ('display', ctypes.c_void_p), ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                ('format', ctypes.c_int), ('data', ClientData)]


class XEvent(ctypes.Union):
    _fields_ = [('xclient', ClientMessage), ('pad', ctypes.c_long*24)]


def arrange(windows, environment):
    if not windows:
        raise RuntimeError('未找到实验图像窗口。')
    area = work_area(environment)
    rectangles = layout_rectangles(area, windows)
    for key in ('DISPLAY', 'XAUTHORITY'):
        if key in environment:
            os.environ[key] = environment[key]
    x11 = ctypes.CDLL(ctypes.util.find_library('X11'))
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p];x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p];x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int];x11.XInternAtom.restype = ctypes.c_ulong
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_long, ctypes.POINTER(XEvent)]
    x11.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XFlush.argtypes = [ctypes.c_void_p];x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x11.XOpenDisplay(environment['DISPLAY'].encode())
    if not display:
        raise RuntimeError('无法连接本机X11桌面。')
    try:
        root = x11.XDefaultRootWindow(display)
        atom = lambda name: x11.XInternAtom(display, name.encode(), False)
        def send(window, kind, values):
            event = XEvent();event.xclient.type = 33;event.xclient.display = display
            event.xclient.window = window;event.xclient.message_type = atom(kind);event.xclient.format = 32
            for i,value in enumerate(values):event.xclient.data.l[i] = value
            x11.XSendEvent(display, root, False, (1<<20)|(1<<19), ctypes.byref(event))
        for window in windows.values():
            send(window, '_NET_WM_STATE', [0,atom('_NET_WM_STATE_MAXIMIZED_HORZ'),atom('_NET_WM_STATE_MAXIMIZED_VERT'),2,0])
            send(window, '_NET_WM_STATE', [0,atom('_NET_WM_STATE_FULLSCREEN'),0,2,0])
        x11.XFlush(display);time.sleep(.25)
        applied = {}
        for name,(x,y,width,height) in rectangles.items():
            window = windows[name]
            props = properties(environment, '-id', hex(window), '_NET_FRAME_EXTENTS')
            match = re.search(r'= (\d+), (\d+), (\d+), (\d+)', props)
            left,right,top,bottom = map(int,match.groups()) if match else (0,0,37,0)
            client_width=max(1,width-left-right);client_height=max(1,height-top-bottom)
            # EWMH NorthWestGravity positions the decorated frame, sizes its client.
            send(window,'_NET_MOVERESIZE_WINDOW',[1|(15<<8)|(2<<12),x,y,client_width,client_height])
            x11.XMapRaised(display,window)
            applied[name] = dict(window=hex(window),frame=[x,y,width,height],
                                 client_size=[client_width,client_height],extents=[left,right,top,bottom])
        x11.XFlush(display);time.sleep(.3)
        return dict(work_area=list(area),windows=applied)
    finally:
        x11.XCloseDisplay(display)
