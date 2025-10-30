# Bidirectional Communication Test Guide

## 🎯 **What This Does**

The `bidir.py` script enables **bidirectional communication** between your PC and Pico W:
- **PC → Pico W**: Send messages from PC to Pico W
- **Pico W → PC**: Receive messages from Pico W to PC
- **Real-time**: Both directions work simultaneously

## 🚀 **How to Use**

### **Basic Usage (Default Settings)**
```bash
python3 bidir.py
```
- Uses default IP: `192.168.1.100`
- Uses default port: `80`
- Manual mode: Type messages and press Enter

### **Specify Pico W IP Address**
```bash
python3 bidir.py --ip 172.20.10.7
```

### **Specify Port Number**
```bash
python3 bidir.py --ip 192.168.1.100 --port 12345
```

### **Automatic Mode (Periodic Messages)**
```bash
python3 bidir.py --mode auto
```

### **Get Help**
```bash
python3 bidir.py --help
```

## 🔧 **Configuration Options**

| Option | Default | Description |
|--------|---------|-------------|
| `--ip` | `192.168.1.100` | Pico W IP address |
| `--port` | `80` | Port number |
| `--mode` | `manual` | `auto` (periodic) or `manual` (user input) |

## 📋 **Testing Steps**

### **Step 1: Find Your Pico W IP**
Check your Pico W console output for:
```
WiFi connected! IP: 192.168.1.100
```

### **Step 2: Run the Script**
```bash
python3 bidir.py --ip 192.168.1.100
```

### **Step 3: Test Communication**
- **Type messages** and press Enter
- **Watch for responses** from Pico W
- **Press Ctrl+C** to quit

## 🎯 **Expected Results**

### **Successful Connection:**
```
Connecting to Pico at 192.168.1.100:80 ...
✅ Connected (bidirectional mode). You can now chat with the Pico.
Type messages and press Enter to send. Press Ctrl+C to quit.
Enter message: hello
PC → hello
Pico → response from pico
```

### **Connection Failed:**
```
Connecting to Pico at 192.168.1.100:80 ...
❌ Connection timeout. Check if Pico W is running and IP is correct.
```

## 🔍 **Troubleshooting**

### **Connection Timeout:**
- Check Pico W IP address
- Ensure Pico W is running
- Check network connectivity

### **Connection Refused:**
- Check port number
- Ensure Pico W is listening on that port
- Check firewall settings

### **No Response from Pico:**
- Check Pico W console for errors
- Ensure Pico W is sending data
- Check network connectivity

## 🎉 **Success Criteria**

Your bidirectional communication is working if:
- ✅ **PC can connect** to Pico W
- ✅ **PC can send** messages to Pico W
- ✅ **PC can receive** messages from Pico W
- ✅ **Both directions work** simultaneously
- ✅ **Real-time communication** is established

## 📱 **Quick Test Commands**

```bash
# Test with default settings
python3 bidir.py

# Test with specific IP
python3 bidir.py --ip 192.168.1.100

# Test automatic mode
python3 bidir.py --mode auto

# Test with custom port
python3 bidir.py --ip 192.168.1.100 --port 12345
```

This script provides a complete bidirectional communication test between your PC and Pico W!
