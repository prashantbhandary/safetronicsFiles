#!/usr/bin/env python3
"""
SafeTonics BLE Data Receiver Test Script
This script connects to the ESP32 BLE server and receives sensor data
"""

import asyncio
import sys
from bleak import BleakClient, BleakScanner
import datetime

# BLE Configuration (matches ESP32 code)
DEVICE_NAME = "SafeTonics-Helmet01"
SERVICE_UUID = "1234"
CHARACTERISTIC_UUID = "5678"

class SafeTonicsBLEClient:
    def __init__(self):
        self.client = None
        self.device_address = None
        
    async def find_device(self):
        """Scan for SafeTonics BLE device"""
        print("Scanning for SafeTonics device...")
        devices = await BleakScanner.discover(timeout=10.0)
        
        for device in devices:
            if device.name and DEVICE_NAME in device.name:
                print(f"Found device: {device.name} ({device.address})")
                self.device_address = device.address
                return True
        
        print("SafeTonics device not found!")
        return False
    
    def notification_handler(self, sender, data):
        """Handle incoming BLE notifications"""
        try:
            payload = data.decode('utf-8')
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            
            # Parse the data: HeartRate,SPO2,AmbientTemp,BodyTemp,MPUStatus,FallStatus
            parts = payload.split(',')
            
            if len(parts) >= 6:
                heart_rate = parts[0]
                spo2 = parts[1] 
                ambient_temp = parts[2]
                body_temp = parts[3]
                mpu_status = parts[4]
                fall_status = parts[5]
                
                print(f"\n[{timestamp}] SafeTonics Data Received:")
                print(f"   Heart Rate: {heart_rate} BPM")
                print(f"   SpO2: {spo2}%")
                print(f"   Ambient Temp: {ambient_temp}C")
                print(f"   Body Temp: {body_temp}C")
                print(f"   MPU Status: {mpu_status}")
                print(f"   Fall Status: {fall_status}")
                print(f"   Raw: {payload}")
            else:
                print(f"[{timestamp}] Incomplete data: {payload}")
                
        except Exception as e:
            print(f"Error parsing notification: {e}")
    
    async def connect_and_receive(self):
        """Connect to device and start receiving data"""
        if not self.device_address:
            if not await self.find_device():
                return False
        
        try:
            print(f"Connecting to {self.device_address}...")
            self.client = BleakClient(self.device_address)
            await self.client.connect()
            
            if self.client.is_connected:
                print("Connected successfully!")
                
                # Subscribe to notifications
                await self.client.start_notify(CHARACTERISTIC_UUID, self.notification_handler)
                print("Listening for sensor data...")
                print("Press Ctrl+C to stop\n")
                
                # Keep listening
                while True:
                    await asyncio.sleep(1)
                    
        except Exception as e:
            print(f"Connection error: {e}")
            return False
        finally:
            if self.client and self.client.is_connected:
                await self.client.disconnect()
                print("\nDisconnected from device")

async def main():
    print("SafeTonics BLE Test Receiver")
    print("=" * 40)
    
    client = SafeTonicsBLEClient()
    
    try:
        await client.connect_and_receive()
    except KeyboardInterrupt:
        print("\nTest stopped by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    # Check if bleak is installed
    try:
        import bleak
        asyncio.run(main())
    except ImportError:
        print("Bleak library not found!")
        print("Install with: pip install bleak")
        sys.exit(1)