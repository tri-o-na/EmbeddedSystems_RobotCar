import time, urllib.parse, http.client

PICO_IP = "172.20.10.8"     # change to your Pico IP
PORT    = 80

def send(type_str, msg):
    conn = http.client.HTTPConnection(PICO_IP, PORT, timeout=3)
    path = "/api/send?type=" + urllib.parse.quote_plus(type_str) + \
           "&msg=" + urllib.parse.quote_plus(msg)
    conn.request("GET", path)
    conn.getresponse().read()   # ignore body
    conn.close()

i = 0
try:
    while True:
        send("pc", f"hello world #{i}")
        time.sleep(0.5)         # 2 msgs per second; adjust as needed
        i += 1
except KeyboardInterrupt:
    pass
