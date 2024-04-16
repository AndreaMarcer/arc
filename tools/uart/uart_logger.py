import serial
import struct

def main():
    # Open serial port
    ser = serial.Serial('/dev/ttyAMA3', 921600)
    
    try:
        while True:
            # Read 12 bytes from serial port
            data = ser.read(12)
            
            # Interpret the bytes as three floats
            if len(data) == 12:
                # Unpack the bytes into three floats
                floats = struct.unpack('fff', data)
                
                # Print the interpreted floats
                print(floats)
            
    except KeyboardInterrupt:
        # Close serial port on Ctrl+C
        ser.close()

if __name__ == "__main__":
    main()
