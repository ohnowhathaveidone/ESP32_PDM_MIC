import sys
import socket
import wave

if len(sys.argv)>2:
    srate = int(sys.argv[1]);
    outName = sys.argv[2];
else:
    print("Defaulting to 192KHz sample rate and filename output_");
    srate = 192000; #48000
    outName = 'output';

try:
    doInit = sys.argv[3];
except IndexError:
    doInit = False;

WAVE_OUTPUT_FILENAME = outName + "_" + str(srate) + ".wav";

msgFromClient       = "ping";
bytesToSend         = str.encode(msgFromClient);
serverAddressPort   = ("192.168.1.3", 3333); #the esp's IP address is being desplayed in monitor console
bufferSize          = 1024;
# Create a UDP socket at client side
UDPClientSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Send to server using created UDP socket
# this initializes the server on the esp32 and provides it with a return address
if doInit:
    #UDPClientSocket.connect(serverAddressPort);
    #UDPClientSocket.send(bytesToSend);
    UDPClientSocket.sendto(bytesToSend, serverAddressPort);

print('receiving data');

frames = b'';
i = 0;
try:
    while True:
        data, addr = UDPClientSocket.recvfrom(2048) ;
        frames = frames+data;
        if ((i%100) == 0):
            print('got ', i, ' packets');
        i += 1;
        
    
except KeyboardInterrupt: 
    #convert?..
    ba = bytearray(frames);
    #ba.reverse();
    print("Writing for rate ",srate);
    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb');
    
    wf.setnchannels(1);
    wf.setsampwidth(2);
    wf.setframerate(srate);
    wf.writeframes(ba);
    wf.close();
