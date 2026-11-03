"""Start the existing read-only viewer with explicit connection configuration.

The original app/manual_mapping_viewer.py remains byte-for-byte unchanged.
This launcher imports its cache and HTTP handler, without using its old main.
Passwords are prompted at runtime and kept only in memory. SSH host keys must
already be present in the caller's known_hosts file; unknown keys are rejected.
"""
import argparse
import getpass
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--user", required=True)
    parser.add_argument("--record", required=True, type=Path)
    parser.add_argument("--known-hosts", required=True, type=Path)
    parser.add_argument("--ssh-port", type=int, default=22)
    parser.add_argument("--port", type=int, default=8766, help="Local HTTP port; binds only to 127.0.0.1")
    args = parser.parse_args(argv)
    if not 1 <= args.ssh_port <= 65535 or not 1 <= args.port <= 65535:
        parser.error("Ports must be 1..65535")
    if not args.known_hosts.is_file():
        parser.error("--known-hosts must name an existing verified SSH host-key file")
    sys.path.insert(0, str(ROOT / "app"))
    from manual_mapping_viewer import ReadOnlyCache, checked_record, make_handler
    checked_record(args.record)
    import paramiko
    keys = paramiko.HostKeys(str(args.known_hosts))
    key_name = args.host if args.ssh_port == 22 else "[" + args.host + "]:" + str(args.ssh_port)
    if not keys.lookup(key_name):
        parser.error("The requested host/port has no pinned key in --known-hosts")
    password = getpass.getpass("RK SSH password (memory only): ")

    def connect():
        client = paramiko.SSHClient()
        client.load_host_keys(str(args.known_hosts))
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        try:
            client.connect(args.host, port=args.ssh_port, username=args.user, password=password,
                           allow_agent=False, look_for_keys=False, timeout=5,
                           banner_timeout=8, auth_timeout=8)
        except BaseException:
            client.close()
            raise
        return client

    cache = ReadOnlyCache(args.record, connect)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(cache))
    server.daemon_threads = True
    cache.thread.start()
    print(json.dumps(dict(url="http://127.0.0.1:" + str(args.port) + "/", read_only=True,
                          robot_commands=False, credential_saved=False)), flush=True)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        cache.stop.set()
        server.server_close()
        cache.thread.join(timeout=6.)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
