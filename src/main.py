import asyncio

from viam.module.module import Module

# The Python SDK's arm servicer lacks MoveThroughJointPositions (which the
# motion service needs) and Get3DModels; swap in ours before the server is built.
from .rebot_b601 import arm_service

arm_service.install()

# Importing the models registers them (EasyResource).
from .rebot_b601.arm import B601Arm  # noqa: E402,F401
from .rebot_b601.gripper import B601Gripper  # noqa: E402,F401

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
