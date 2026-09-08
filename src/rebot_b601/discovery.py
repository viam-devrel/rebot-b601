"""Discovery service: propose ready-to-paste configs for every attached B601.

Identification is by USB vendor/product id from sysfs, so discovery never opens
a serial port and cannot disturb a device another driver is using. The emitted
configs carry the stable /dev/serial/by-id path, so a machine with several
USB serial devices never depends on enumeration order.
"""

from typing import Any, List, Mapping, Optional, Sequence, Tuple

from viam.proto.app.robot import ComponentConfig, Frame
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.services.discovery import Discovery
from viam.utils import ValueTypes, dict_to_struct

from . import bus

ARM_MODEL = "devrel:rebot-b601:arm"
GRIPPER_MODEL = "devrel:rebot-b601:gripper"


class B601Discovery(Discovery, EasyResource):
    MODEL = "devrel:rebot-b601:discovery"

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Tuple[Sequence[str], Sequence[str]]:
        return [], []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        pass

    async def discover_resources(
        self, *, extra: Optional[Mapping[str, ValueTypes]] = None, timeout: Optional[float] = None
    ) -> List[ComponentConfig]:
        configs: List[ComponentConfig] = []
        for i, board in enumerate(bus.find_b601_ports()):
            suffix = "" if i == 0 else f"-{i + 1}"
            arm_name = f"rebot-arm{suffix}"
            arm = ComponentConfig(
                name=arm_name,
                api="rdk:component:arm",
                model=ARM_MODEL,
                attributes=dict_to_struct({"port": board.path}),
                frame=Frame(parent="world"),
            )
            gripper = ComponentConfig(
                name=f"rebot-gripper{suffix}",
                api="rdk:component:gripper",
                model=GRIPPER_MODEL,
                attributes=dict_to_struct({"arm": arm_name}),
                depends_on=[arm_name],
                frame=Frame(parent=arm_name),
            )
            configs += [arm, gripper]
        return configs

    async def do_command(
        self, command: Mapping[str, ValueTypes], *, timeout: Optional[float] = None, **kwargs
    ) -> Mapping[str, Any]:
        """``{"serial_ports": true}`` lists every USB serial device with its USB identity."""
        if command.get("serial_ports"):
            return {
                "serial_ports": [
                    {
                        "device": p.device,
                        "by_id": p.by_id or "",
                        "vid": p.vid,
                        "pid": p.pid,
                        "serial": p.serial,
                        "product": p.product,
                        "b601": (p.vid, p.pid) == ("2e88", "4603"),
                    }
                    for p in bus.usb_serial_ports()
                ]
            }
        raise ValueError(f"unknown command(s): {sorted(command)}; supported: serial_ports")
