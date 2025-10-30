import socket
import threading
import time

# --- Configuration ---
PICO_IP = "172.20.10.7"   # ← change to your Pico W IP
PORT = 12345              # must match start_bidir_server() in main.c


# --- Receive thread: listen for data from Pico ---
def recv_loop(sock):
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                print("Disconnected from Pico.")
                break
            print(f"Pico → {data.decode().strip()}")
        except Exception as e:
            print("Receive error:", e)
            break


# --- Main connection logic ---
def main():
    print(f"Connecting to Pico at {PICO_IP}:{PORT} ...")
    sock = socket.create_connection((PICO_IP, PORT))
    print("✅ Connected (bidirectional mode). You can now chat with the Pico.")

    # start background receiver thread
    threading.Thread(target=recv_loop, args=(sock,), daemon=True).start()

    i = 0
    try:
        while True:
            # Example: send periodic messages to Pico
            msg = f"hello world #{i}"
            sock.sendall(msg.encode())
            print(f"PC → {msg}")
            i += 1
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nClosing connection...")
        sock.close()


if __name__ == "__main__":
    main()
