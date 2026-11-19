#!/usr/bin/env python3
"""x30_check: the ROS rows of tools/x30_host.sh check (standard library only, no ROS needed).

The master and every node's address from here. Exit 1 when the master or a node cannot be
reached.
"""

from __future__ import annotations

import argparse
import socket
import sys
from urllib.parse import urlparse
import xmlrpc.client

CALLER = '/x30_host_check'
TIMEOUT = 2.0


def row(name: str, status: str, text: str):
    """Print one row as x30_host.sh does."""
    print(f'{name:<8} {status:<5} {text}', flush=True)


def proxy(uri: str) -> xmlrpc.client.ServerProxy:
    """Return an XML-RPC proxy (the socket timeout is set for the whole run)."""
    return xmlrpc.client.ServerProxy(uri)


def nodes(master, state) -> int:
    """Print the nodes this host cannot reach; return how many."""
    names = sorted({node for kind in state for _, members in kind for node in members})
    unreachable = 0
    for node in names:
        code, _, uri = master.lookupNode(CALLER, node)
        if code != 1:
            continue
        address = urlparse(uri)
        try:
            socket.create_connection((address.hostname, address.port), TIMEOUT).close()
        except OSError as error:
            unreachable += 1
            row('node', 'MISS', f'{node} at {address.hostname}:{address.port}: {error}')
    if unreachable:
        print('         a name that does not resolve: "<ip> <name>" in /etc/hosts, or ROS_IP for')
        print('         that node where it starts')
    status = 'ok' if not unreachable else 'MISS'
    row('nodes', status, f'{len(names) - unreachable} of {len(names)} reachable from here')
    return unreachable


def main(argv: list | None = None) -> int:
    """Print the rows; 1 when something cannot be reached."""
    parser = argparse.ArgumentParser(prog='x30_check', description=__doc__.splitlines()[0])
    parser.add_argument('--master', required=True)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    socket.setdefaulttimeout(TIMEOUT)
    master, problems = proxy(args.master), 0
    try:
        _, _, state = master.getSystemState(CALLER)
    except (OSError, xmlrpc.client.Error) as error:
        row('master', 'MISS', f'{args.master}: {error}')
        state, problems = None, 1
    if state is not None:
        row('master', 'ok', args.master)
        problems += nodes(master, state)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
