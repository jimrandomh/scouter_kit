"""Pair this PC with an arm running g1v0, in the case (g1v0 is bondable only with VBUS). LE Secure
Connections, Just Works. Pairs only if the arm's current advertisement has a G1CFW_ name (before 0.5.66: the g1v0
service UUID), so it never pairs with stock firmware. Own BlueZ agent that accepts only this address (adapter hci0).
Usage: python3 -I arm_pair.py <address> [seconds to stay connected]
Run `bluetoothctl pairable on` first and remove any stale bond of that address."""
import asyncio
import sys
import time
from bleak import BleakClient, BleakScanner
from dbus_fast import BusType, DBusError, Variant
from dbus_fast.aio import MessageBus
from dbus_fast.service import ServiceInterface, method

G1V0_SVC = "f3da4d59-277a-4fc9-a1ed-41c9c22d4400"
addr = sys.argv[1].upper()
hold = float(sys.argv[2]) if len(sys.argv) > 2 else 3
DEV_PATH = "/org/bluez/hci0/dev_" + addr.replace(":", "_")


def stamp():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)


def check(device):
    print(stamp(), "agent request for", device, flush=True)
    if device != DEV_PATH:
        raise DBusError("org.bluez.Error.Rejected", "not this arm")


class Agent(ServiceInterface):
    def __init__(self):
        super().__init__("org.bluez.Agent1")

    @method()
    def Release(self):
        pass

    @method()
    def RequestPinCode(self, device: "o") -> "s":
        raise DBusError("org.bluez.Error.Rejected", "no pin")

    @method()
    def DisplayPinCode(self, device: "o", pincode: "s"):
        pass

    @method()
    def RequestPasskey(self, device: "o") -> "u":
        raise DBusError("org.bluez.Error.Rejected", "no passkey")

    @method()
    def DisplayPasskey(self, device: "o", passkey: "u", entered: "q"):
        pass

    @method()
    def RequestConfirmation(self, device: "o", passkey: "u"):
        check(device)

    @method()
    def RequestAuthorization(self, device: "o"):
        check(device)

    @method()
    def AuthorizeService(self, device: "o", uuid: "s"):
        check(device)

    @method()
    def Cancel(self):
        print(stamp(), "agent cancel", flush=True)


async def find_g1v0():
    found = asyncio.Future()

    def cb(dev, adv):
        name = adv.local_name or ""
        ours = name.startswith("G1CFW_") or G1V0_SVC in [u.lower() for u in adv.service_uuids]  # the UUID: before 0.5.66
        if dev.address.upper() == addr and ours and not found.done():
            found.set_result((dev, adv))

    async with BleakScanner(cb, scanning_mode="active"):
        return await asyncio.wait_for(found, 30)


async def main():
    dev, adv = await find_g1v0()
    print(stamp(), "g1v0 advertising:", dev.address, adv.local_name, flush=True)
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    bus.export("/g1arm/agent", Agent())
    intro = await bus.introspect("org.bluez", "/org/bluez")
    mgr = bus.get_proxy_object("org.bluez", "/org/bluez", intro).get_interface("org.bluez.AgentManager1")
    await mgr.call_register_agent("/g1arm/agent", "NoInputNoOutput")
    await mgr.call_request_default_agent("/g1arm/agent")
    print(stamp(), "agent registered", flush=True)
    try:
        lost = asyncio.Event()
        client = BleakClient(dev, timeout=20, disconnected_callback=lambda c: lost.set())
        await client.connect()
        print(stamp(), "connected, mtu", client.mtu_size, flush=True)
        try:
            await client.pair()
            print(stamp(), "pair() returned", flush=True)
        except Exception as e:
            print(stamp(), "pair():", repr(e), flush=True)
        dintro = await bus.introspect("org.bluez", DEV_PATH)
        props = bus.get_proxy_object("org.bluez", DEV_PATH, dintro).get_interface("org.freedesktop.DBus.Properties")
        for _ in range(50):
            if (await props.call_get("org.bluez.Device1", "Paired")).value:
                break
            await asyncio.sleep(0.1)
        paired = (await props.call_get("org.bluez.Device1", "Paired")).value
        bonded = (await props.call_get("org.bluez.Device1", "Bonded")).value
        print(stamp(), "paired", paired, "bonded", bonded, flush=True)
        if bonded:
            await props.call_set("org.bluez.Device1", "Trusted", Variant("b", True))
        try:
            await asyncio.wait_for(lost.wait(), hold)
            print(stamp(), "arm disconnected us", flush=True)
        except asyncio.TimeoutError:
            print(stamp(), "still connected after %.0f s" % hold, flush=True)
        if client.is_connected:
            await client.disconnect()
    finally:
        await mgr.call_unregister_agent("/g1arm/agent")


asyncio.run(main())
