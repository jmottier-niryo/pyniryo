#!/usr/bin/env bash
# Regenerates the TLS fixtures: a test root, leaves signed by it, and a leaf from another root.
# Run from this directory. The leaves mirror what the robot serves: ECDSA P-384, the device identity as a URI SAN.
set -euo pipefail
cd "$(dirname "$0")"

DEVICE_URN="urn:niryo:device:test-device-123"
OTHER_URN="urn:niryo:device:other-device-456"

new_ca() {
    openssl ecparam -name secp384r1 -genkey -noout -out "$1-key.pem"
    openssl req -x509 -new -key "$1-key.pem" -sha384 -days 10950 -subj "/O=Niryo/CN=$2" \
        -addext "basicConstraints=critical,CA:TRUE,pathlen:0" \
        -addext "keyUsage=critical,keyCertSign,cRLSign" -out "$1.pem"
}

issue() {
    local ca=$1 name=$2 san=$3
    openssl req -new -key robot-key.pem -subj "/O=Niryo/CN=$name" -out "$name.csr"
    openssl x509 -req -in "$name.csr" -CA "$ca.pem" -CAkey "$ca-key.pem" -CAcreateserial -days 7300 -sha384 \
        -extfile <(printf "subjectAltName=%s\nextendedKeyUsage=serverAuth\nkeyUsage=critical,digitalSignature\n" "$san") \
        -out "$name.pem"
    rm "$name.csr"
}

new_ca ca "PyNiryo Test Root CA"
new_ca other-ca "PyNiryo Other Root CA"
openssl ecparam -name secp384r1 -genkey -noout -out robot-key.pem

issue ca robot "URI:$DEVICE_URN"
issue ca other-device "URI:$OTHER_URN"
issue ca no-urn "DNS:localhost"
issue ca two-urns "URI:$DEVICE_URN,URI:$OTHER_URN"
issue other-ca other-ca-robot "URI:$DEVICE_URN"
rm -f ./*.srl
