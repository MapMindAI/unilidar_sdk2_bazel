import socket
import base64
import serial
import os

# ── Config from environment (replaces hardcoded values in reference) ──────────
COM             = os.environ.get("RTK_SERIAL_PORT",   "/dev/ttyUSB0")
BPS         = int(os.environ.get("RTK_BAUDRATE",  "115200"))
NtripIP     = os.environ.get("NTRIP_HOST",        "")
NtripPort       = int(os.environ.get("NTRIP_PORT",    "2101"))
NtripUser   = os.environ.get("NTRIP_USER",        "")
NtripPwd   = os.environ.get("NTRIP_PASSWORD",    "")
NtripPoint = os.environ.get("NTRIP_MOUNTPOINT",  "")

print(COM)
print(BPS)
print(NtripIP)
print(NtripPort)
print(NtripUser)
print(NtripPwd)
print(NtripPoint)
# ------------------------------------------------------------------
RTK = serial.Serial(COM, BPS, timeout=0.01)
print("等待RTK模块定位...")
while True:
    data = RTK.readline()
    if len(data):
        strNMEA = data.decode("ascii")
        seg = strNMEA.split(',')
        if seg[0] == "$GNGGA":
            if len(seg[6]) and seg[6]!='0':
                strGNGGA = strNMEA + "\r\n\r\n"
                print(strGNGGA)
                break

ntrip = socket.socket()
ntrip.connect((NtripIP,NtripPort))

user_pwd = base64.b64encode(bytes(NtripUser+':'+NtripPwd, 'utf-8')).decode("utf-8")
httpHead = "GET /"+NtripPoint+" HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAccept: */*\r\nConnection: close\r\nAuthorization: Basic "+user_pwd+"\r\n\r\n"
ntrip.send(httpHead.encode())
data = ntrip.recv(1024)
print(data)
ntrip.send(strGNGGA.encode())

while True:
    data = RTK.read(102400)
    if len(data):
        print(data.decode("ascii"))
    data = ntrip.recv(102400)
    RTK.write(data)
exit()
