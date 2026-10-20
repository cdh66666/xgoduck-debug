"""One bounded command RPC on a fresh local socket; no reconnection or replay.

The general SDK Bridge remains responsible for receiving MCU reports. This
connection registers no handlers and is closed after each explicit command.
"""
import errno
import math
import os
import select
import socket
import time

import msgpack


class RouteMissing(ValueError):
    code = 2
    message = 'method duck_calibration not available'


class UnixCommandRPC:
    MAX_REPLY = 4096

    def __init__(self, address=None, *, clock=time.monotonic, socket_factory=None):
        address = address or os.environ.get('APP_SOCKET', 'unix:///var/run/arduino-router.sock')
        if (not isinstance(address, str) or not address.startswith('unix:///')
                or '\x00' in address or len(address.encode('utf-8')) > 128):
            raise ValueError('A local absolute unix:// Router socket is required')
        self.path = address[len('unix://'):]
        self.clock = clock
        self.socket_factory = socket_factory or (lambda: socket.socket(socket.AF_UNIX, socket.SOCK_STREAM))

    def _wait(self, stream, *, read, deadline):
        remaining = deadline - self.clock()
        if remaining <= 0:
            raise TimeoutError('Command dispatch deadline expired')
        readable, writable, exceptional = select.select([stream] if read else [], [] if read else [stream], [stream], remaining)
        if exceptional:
            raise ConnectionError('Router socket failed')
        if not (readable if read else writable) or self.clock() >= deadline:
            raise TimeoutError('Command dispatch deadline expired')

    def call(self, method, payload, *, timeout=.2):
        if method != 'duck_calibration' or type(payload) is not bytes or len(payload) not in (32,36,108):
            raise ValueError('Only fixed calibration command envelopes are supported')
        if type(timeout) not in (int,float) or not math.isfinite(timeout) or not 0 < timeout <= .2:
            raise ValueError('Command budget must be within 200ms')
        deadline = self.clock() + timeout
        packed = msgpack.packb([0,1,method,[payload]], use_bin_type=True)
        stream = self.socket_factory()
        try:
            stream.setblocking(False)
            result = stream.connect_ex(self.path)
            pending = {errno.EINPROGRESS, errno.EALREADY, errno.EWOULDBLOCK,
                       getattr(errno,'WSAEWOULDBLOCK',10035)}
            if result not in (0,errno.EISCONN):
                if result not in pending:
                    raise ConnectionError('Router socket is unavailable')
                self._wait(stream,read=False,deadline=deadline)
                if stream.getsockopt(socket.SOL_SOCKET,socket.SO_ERROR):
                    raise ConnectionError('Router connection failed')
            sent = 0
            while sent < len(packed):
                self._wait(stream,read=False,deadline=deadline)
                try: count = stream.send(memoryview(packed)[sent:])
                except (BlockingIOError,InterruptedError): continue
                if count <= 0: raise ConnectionError('Router closed during send')
                sent += count
            unpacker = msgpack.Unpacker(raw=False,max_buffer_size=self.MAX_REPLY,
                                       max_array_len=4,max_map_len=0,max_str_len=1024,max_bin_len=0)
            received = 0
            while True:
                self._wait(stream,read=True,deadline=deadline)
                try: data = stream.recv(min(1024,self.MAX_REPLY+1-received))
                except (BlockingIOError,InterruptedError): continue
                if not data: raise ConnectionError('Router closed before replying')
                received += len(data)
                if received > self.MAX_REPLY: raise ValueError('Oversized Router response')
                unpacker.feed(data)
                for response in unpacker:
                    if (type(response) is not list or len(response)!=4 or type(response[0]) is not int
                            or response[0]!=1 or type(response[1]) is not int or response[1]!=1):
                        raise ValueError('Unexpected Router RPC response')
                    error, result = response[2:]
                    if error is not None:
                        if (type(error) is list and len(error)==2 and type(error[0]) is int
                                and error == [2,'method duck_calibration not available'] and result is None):
                            raise RouteMissing('Command route unavailable')
                        raise RuntimeError('Router rejected command dispatch')
                    if result is not None:
                        raise ValueError('Unexpected command handler result')
                    if self.clock() >= deadline:
                        raise TimeoutError('Router reply arrived after deadline')
                    return None
        finally:
            stream.close()
