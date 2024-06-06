import os
import numpy as np
import subprocess
from multiprocessing import Process
import argparse
import time
import struct
import fcntl
from circuitpython_nrf24l01.rf24 import RF24
import board
import spidev
import digitalio as dio








##############################################################
# Network Setup:
def setup_network():
    # Run commands


    # Delete existing TUN interfaces
    subprocess.run("sudo ip link delete tun2", shell = True)

    time. sleep(1)

    # Create new TUN interfaces
    subprocess.run("sudo ip tuntap add mode tun dev tun2", shell = True)
    time. sleep(1)

    # Assign IP addresses to the TUN interfaces
    subprocess.run("sudo ip addr add 192.168.1.2/24 dev tun2", shell = True)
    time. sleep(1)


    # Bring up the TUN interfaces
    subprocess.run("sudo ip link set dev tun2 up", shell = True)
    #subprocess.run("sudo ip link set dev tun3 up", shell = True)

    ##Adding default route
    subprocess.run("sudo ip route add 0.0.0.0/0 via 192.168.1.2 dev tun2", shell = True)


    #subprocess.run("ip route add default via 192.168.1.2 dev tun2", shell = True)
    time. sleep(1)



    # Display the status of TUN interfaces
    subprocess.run("sudo ip addr show", shell = True)
    time. sleep(1)


##############################################################


# initilize the tun:
def tun_initialization(Tun_name = 'tun2'):
    # Defining the name of the TUN interface
    interface_name = Tun_name
    
    # Opening the TUN interface and obtaining its file descriptor
    tun_descriptor = os.open('/dev/net/tun', os.O_RDWR)
    
    # Packing the TUN interface name and flags into a struct to configure the interface
    interface_request = struct.pack('16sH', bytes(interface_name, 'utf-8'), 0x0001)
    
    # Configuring the TUN interface using the file descriptor and the packed interface request
    fcntl.ioctl(tun_descriptor, 0x400454ca, interface_request)  # TUNSETIFF
    
    # Returning the initialized TUN interface
    return tun_descriptor

##############################################################

# SPI intitialization

def init_spi(bus):
    spi = spidev.SpiDev()
    spi.open(bus, 0)
    return spi

##############################################################

# TX Initialization
def tx(nrf, channel, address, count, tun, size = 1):
    nrf.open_tx_pipe(address)  # set address of RX node into a TX pipe
    nrf.listen = False
    nrf.channel = channel
    status = []
    start = time.monotonic()


    # Segmentation Process

    # Initialize header and footer
    header = bytes([0x00])
    footer = bytes([0xFF])
    mtu = 1500
    #count = 10  # Number of packets to send
    status = []  # List to store the status of packet sending

    # Start time measurement
    start = time.monotonic()
    #count = 10000000
    while True:
        # Read packet from tun interface
        packet = os.read(tun, mtu)

        # Add header and footer to the packet
        packet = header + packet + footer

        # Display the size of the packet
        print("Packet size is: ", len(packet))

        i = 0 
        while i < len(packet):
            # Split packet into fragments of size 32 bytes
            fragment = packet[i:i+32]

            # Display the size of the fragment
            print("Fragment size is :", len(fragment))

            # Send fragment via nrf communication
            result = nrf.send(fragment)

            i += 32

        if not result:
            print("send() failed or timed out")
            status.append(False)
        else:
            print("send() successful")
            status.append(True)

        #count -= 1

    # Calculate total time taken
    total_time = time.monotonic() - start
    print('{} successfull transmissions, {} failures, {} bps'.format(sum(status), len(status)-sum(status), 32*8*len(status)/total_time))

# RX Initialization
def rx(nrf, tun, channel, address, count, size=1):
    nrf.open_rx_pipe(1, address)
    nrf.listen = True  # put radio into RX mode and power up
    nrf.channel = channel

    print('Rx NRF24L01+ started w/ power {}, SPI freq: {} hz'.format(nrf.pa_level, nrf.spi_frequency))

    received = []
    #count = 100000
    start_time = None
    start = time.monotonic()
    i=0
    
    while True:
        # Uncomment the following line if you want to add a time limit
        # if (time.monotonic() - start) >= 6:
        #     break
        
        if nrf.update() and nrf.pipe is not None:
            if start_time is None:
                start_time = time.monotonic()
            
            rx = nrf.read()
            received.append(rx)
            
            print("\nRecieved Data:\n", received, "\n")
            
            i += 1
            
            last_packet = received[-1]
            last_byte = last_packet[-1]
            if last_byte == 0xff:

                print("\nHeader and Footer Included:\n", received, "\n")
                
                
                Byte_list = []
                Byte_combined = b"".join(received)
                # Removing Header and Footer
                
                Byte_combined_without_headers = Byte_combined[1:-1]
                
            
                os.write(tun, Byte_combined_without_headers)
                
                print("\nWrite Receved Data Back to TUN", Byte_combined_without_headers, "\n")
                                
                received = []  # Clearing the data list
                
            #count -= 1  # Decrementing count


    total_time = time.monotonic() - start_time

    print('{} received, {} average, {} bps'.format(len(received), np.mean(received), np.sum(received)*8/total_time))
if __name__ == "__main__":
    
    setup_network()
    tun = tun_initialization("tun2")

   
    parser = argparse.ArgumentParser(description='NRF24L01+ test')
    parser.add_argument('--src', dest='src', type=str, default='rrrrr', help='NRF24L01+\'s source address')
    parser.add_argument('--dst', dest='dst', type=str, default='ttttt', help='NRF24L01+\'s destination address')
    parser.add_argument('--count', dest='cnt', type=int, default=10, help='Number of transmissions')
    parser.add_argument('--size', dest='size', type=int, default=32, help='Packet size')
    parser.add_argument('--txchannel', dest='txchannel', type=int, default=66, help='Tx channel', choices=range(0,125))
    parser.add_argument('--rxchannel', dest='rxchannel', type=int, default=66, help='Rx channel', choices=range(0,125))

    args = parser.parse_args()

    # SPI Configurations
    config0 = {
        'spi_bus': 0,
        'csn_pin': 0,
        'ce_pin': board.D17
    }

    config1 = {
        'spi_bus': 1,
        'csn_pin': 10,
        'ce_pin': board.D27
    }

    # Initialize the nRF24L01 modules
    spi0 = init_spi(config0['spi_bus'])
    spi1 = init_spi(config1['spi_bus'])

    CE0 = dio.DigitalInOut(config0['ce_pin'])
    CE1 = dio.DigitalInOut(config1['ce_pin'])

    rx_nrf = RF24(spi0, config0['csn_pin'], CE0)
    tx_nrf = RF24(spi1, config1['csn_pin'], CE1)
    
    for nrf in [rx_nrf, tx_nrf]:
        nrf.data_rate = 2
        nrf.auto_ack = True
        #nrf.dynamic_payloads = True
        nrf.payload_length = 32
        nrf.crc = True
        nrf.ack = True
        nrf.pa_level = 0
        nrf.spi_speed = 8000000
        nrf.spi_frequency = 2400000000
    #call teh tun function:

    tx_process = Process(target=rx, args=(rx_nrf, tun, args.rxchannel, bytes(args.dst, 'utf-8'), args.cnt))
    rx_process = Process(target=tx, args=(tx_nrf, args.txchannel, bytes(args.src, 'utf-8'), args.cnt, tun))
    tx_process.start()
    rx_process.start()
    time.sleep(1)
    tx_process.join()
    rx_process.join()

