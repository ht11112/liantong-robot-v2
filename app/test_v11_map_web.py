"""HTTP framing on an SSH-like channel, without opening a socket."""
import io
import unittest

from v11_map_web import SSHHTTP


class DelayedBodyChannel:
    """The first read yields headers; explicit channel close cuts off later data."""
    def __init__(self, body):
        self.chunks = [b'HTTP/1.0 200 OK\r\nContent-Length: '+str(len(body)).encode()+b'\r\n\r\n', body]
        self.closed = False

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, data):
        pass

    def close(self):
        self.closed = True

    def makefile(self, mode):
        channel = self

        class Stream(io.RawIOBase):
            def readable(self):
                return True

            def readinto(self, buffer):
                if channel.closed or not channel.chunks:
                    return 0
                chunk = channel.chunks.pop(0)
                buffer[:len(chunk)] = chunk
                return len(chunk)

        return io.BufferedReader(Stream())


class TunnelTests(unittest.TestCase):
    def test_connection_close_does_not_discard_delayed_http_body(self):
        channel = DelayedBodyChannel(b'{"connected":true}')

        class Transport:
            def open_channel(self, *args, **kwargs):
                return channel

        remote = SSHHTTP('unused', 'cat', 'unused', None, None)
        remote.transport = Transport
        self.assertEqual(remote.request('GET', '/api/state'), (200, {'connected': True}))
        self.assertTrue(channel.closed)
        self.assertEqual(channel.chunks, [])


if __name__ == '__main__':
    unittest.main()
