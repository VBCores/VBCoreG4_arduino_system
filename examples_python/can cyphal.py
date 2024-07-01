import pycyphal
import pycyphal.application
from pycyphal.application import heartbeat_publisher
import uavcan.si.unit.voltage  
import uavcan.si.sample.angle
import uavcan.node
import asyncio
import os
import sys
#import UAVCAN__PUB__ANGLE_SETPOINT__ID
#import UAVCAN__SUB__ANGLE_SETPOINT__ID

def sign(x):
    if x>0: return 1
    elif x<0: return -1
    else: return 0
class Controller:
    REGISTER_FILE = "controller.db"

    def __init__(self) -> None:
        node_info = uavcan.node.GetInfo_1.Response(
            software_version=uavcan.node.Version_1(major=1, minor=0),
            name="org.opencyphal.pycyphal.cyphal_test",
        )
        self.voltage_output = 0.1
        self._node = pycyphal.application.make_node(node_info, Controller.REGISTER_FILE)
        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1.OPERATIONAL  # type: ignore
        self._node.heartbeat_publisher.vendor_specific_status_code = os.getpid() % 100
       

        self._sub_ang = self._node.make_subscriber(uavcan.si.sample.angle.Scalar_1, 1111)
        self._pub_volt = self._node.make_publisher(uavcan.si.unit.voltage.Scalar_1, 1112)
        
        self._node.start()

    def close(self) -> None:
        """
        This will close all the underlying resources down to the transport interface and all publishers/servers/etc.
        All pending tasks such as serve_in_background()/receive_in_background() will notice this and exit automatically.
        """
        self._node.close()

    async def run(self) -> None:
        """
        The main method that runs the business logic. It is also possible to use the library in an IoC-style
        by using receive_in_background() for all subscriptions if desired.
        """
        angle_setpoint = 0.0
        async for m, _metadata in self._sub_ang:
            assert isinstance(m, uavcan.si.sample.angle.Scalar_1)
            print("angle:",m.radian)
            err = angle_setpoint - m.radian
            self.voltage_output = -5*sign(err)
            await self._pub_volt.publish(uavcan.si.unit.voltage.Scalar_1(self.voltage_output))
            


async def main() -> None:
    #logging.root.setLevel(logging.INFO)
    app = Controller()
    try:
        await app.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.close()


if __name__ == "__main__":
    asyncio.run(main())
  