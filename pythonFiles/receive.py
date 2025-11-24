from bleak import BleakClient
import asyncio

address = "F4:65:0B:49:8F:66"  # Helmet01
UUID = "00005678-0000-1000-8000-00805f9b34fb"  # Full UUID for characteristic with notify property

def callback(sender, data):
    try:
        # Try to decode as UTF-8
        decoded_text = data.decode('utf-8')
        print(f"DATA (UTF-8): {decoded_text}")
    except UnicodeDecodeError:
        # If UTF-8 fails, try other common encodings
        try:
            decoded_text = data.decode('ascii')
            print(f"DATA (ASCII): {decoded_text}")
        except UnicodeDecodeError:
            # If all text decoding fails, show as hex and try latin-1
            hex_data = data.hex()
            latin1_data = data.decode('latin-1', errors='replace')
            print(f"DATA (HEX): {hex_data}")
            print(f"DATA (Latin-1): {latin1_data}")
    
    # Also show raw bytes and length
    # print(f"RAW BYTES: {list(data)}")
    # print(f"LENGTH: {len(data)} bytes")
    print("-" * 40)

async def main():
    async with BleakClient(address) as client:
        print("CONNECTED")
        # Get and display services
        services = client.services
        for service in services:
            print(f"\nSERVICE: {service.uuid}")  #print the service UUID
            for char in service.characteristics:
                props = ",".join(char.properties)
                print(f"  CHAR: {char.uuid}  PROPS: {props}")   #print service id and proper
        
        # First try to read the current value
        try:
            current_value = await client.read_gatt_char(UUID)
            print(f"Current value: {current_value}")
            callback(UUID, current_value)
        except Exception as e:
            print(f"Could not read characteristic: {e}")
        
        # Set up notification if UUID exists
        try:
            print(f"Attempting to start notifications for {UUID}...")
            await client.start_notify(UUID, callback)
            print(f"✓ Notifications started successfully for {UUID}")
            
            # Keep the connection alive and periodically read if notifications fail
            while True:
                await asyncio.sleep(2)
                # Optionally read the value periodically as backup
                try:
                    value = await client.read_gatt_char(UUID)
                    if value:
                        print("Polling mode - ")
                        callback(UUID, value)
                except:
                    pass
                    
        except Exception as e:
            # print(f"Could not start notifications for {UUID}: {e}")
            # print("Falling back to polling mode...")
            # Fallback: poll the characteristic every second
            try:
                for i in range(30):  # Poll for 30 seconds
                    value = await client.read_gatt_char(UUID)
                    print(f"Poll #{i+1} - ")
                    callback(UUID, value)
                    await asyncio.sleep(1)
            except Exception as read_error:
                print(f"Polling also failed: {read_error}")
            
            print("Connection will close...")
            await asyncio.sleep(2)

asyncio.run(main())
