import asyncio

from viam.module.module import Module

# Importing the models registers them (EasyResource).
from .rebot_b601.arm import B601Arm  # noqa: F401
from .rebot_b601.gripper import B601Gripper  # noqa: F401

if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
