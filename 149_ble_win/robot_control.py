import keyboard
import asyncio
import sys
from getpass import getpass

from bleak import BleakClient, BleakError

from ble_utils import parse_ble_args, handle_sigint
args = parse_ble_args('Print advertisement data from a BLE device')
addr = args.addr.lower()
timeout = args.timeout
handle_sigint()

SERVICE_UUID = "4607eda0-f65e-4d59-a9ff-84420d87a4ca"
CHAR_UUIDS = {
    "up": "4607108a-f65e-4d59-a9ff-84420d87a4ca",
    "down": "4607208a-f65e-4d59-a9ff-84420d87a4ca",
    "left": "4607308a-f65e-4d59-a9ff-84420d87a4ca",
    "right": "4607408a-f65e-4d59-a9ff-84420d87a4ca",
}

class RobotController():
    def __init__(self, client):
        self.robot = client
        # keep state for keypresses
        self.pressed = {"up": False, "down": False, "right": False, "left": False}
        # TODO get services/characteristics from robot (if needed)
        
        keyboard.hook(self.on_key_event)

    def on_key_event(self, event):
        # if a key unrelated to direction keys is pressed, ignore
        if event.name not in self.pressed: return
        # if a key is pressed down
        if event.event_type == keyboard.KEY_DOWN:
            # if that key is already pressed down, ignore
            if self.pressed[event.name]: return
            # print key name
            print(event.name)
            # set state of key to pressed
            self.pressed[event.name] = True
            # TODO write to characteristic to change direction
        else:
            # set state of key to released
            self.pressed[event.name] = False
            # TODO write to characteristic to stop moving in this direction

    def __enter__(self):
        return self

async def main(address):
    print(f"searching for device {address} ({timeout}s timeout)")
    try:
        async with BleakClient(address,timeout=timeout) as client:
            print("connected")
            robot = RobotController(client)
            try:
                getpass("Use arrow keys to control robot")
            except KeyboardInterrupt as e:
                sys.exit(0)
            finally:
                await client.disconnect()
    except BleakError as e:
        print("not found")

if __name__ == "__main__":
    while True:
        asyncio.run(main(addr))