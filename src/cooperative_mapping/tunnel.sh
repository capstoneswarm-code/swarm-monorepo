#!/usr/bin/env bash
# One SSH connection carries periodic maps and small goal/pose metadata.
set -u
peer="${1:?Usage: tunnel.sh USER@PEER_ADDRESS}"
key="$HOME/.ssh/swarm_peer_ed25519"
if [[ ! -f "$key" ]]; then
    echo "Missing $key; perform the peer SSH-key setup in README.md first." >&2
    exit 1
fi
trap 'exit 0' INT TERM HUP
while true; do
    ssh -nNT -i "$key" \
        -o BatchMode=yes -o IdentitiesOnly=yes \
        -o StrictHostKeyChecking=yes \
        -o ExitOnForwardFailure=yes -o ConnectTimeout=5 \
        -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
        -L 127.0.0.1:8769:127.0.0.1:8768 "$peer"
    echo "Peer tunnel disconnected; retrying in 3 seconds..." >&2
    sleep 3
done
