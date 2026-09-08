"""Arm gRPC servicer with the RPCs the Python SDK (0.80.x) leaves unimplemented.

viam-server's motion service executes plans through ``MoveThroughJointPositions``
(the RDK arm client's ``GoToInputs`` delegates to it, with no fallback), and the
app's 3D view asks for ``Get3DModels``. The stock ``ArmRPCService`` answers both
with UNIMPLEMENTED, so this subclass adds them and ``install()`` swaps it into
the SDK registry before the module server is built.

The swap touches ``Registry._APIS`` (private) because ``register_api`` refuses
duplicates. ``tests/test_service.py`` pins the behaviour so an SDK upgrade that
changes the registry fails loudly rather than silently reverting.
"""

from grpclib.server import Stream
from viam.components.arm import Arm
from viam.components.arm.client import ArmClient
from viam.components.arm.service import ArmRPCService
from viam.proto.common import Get3DModelsRequest, Get3DModelsResponse
from viam.proto.component.arm import (
    MoveThroughJointPositionsRequest,
    MoveThroughJointPositionsResponse,
    MoveThroughJointPositionsStreamedRequest,
    MoveThroughJointPositionsStreamedResponse,
)
from viam.resource.registry import Registry, ResourceRegistration
from viam.utils import struct_to_dict


def _duration_seconds(d) -> float:
    return float(d.seconds) + float(d.nanos) / 1e9


class B601ArmRPCService(ArmRPCService):
    """ArmRPCService plus MoveThroughJointPositions(+Streamed) and Get3DModels."""

    async def MoveThroughJointPositions(
        self, stream: Stream[MoveThroughJointPositionsRequest, MoveThroughJointPositionsResponse]
    ) -> None:
        request = await stream.recv_message()
        assert request is not None
        arm = self.get_resource(request.name)
        timeout = stream.deadline.time_remaining() if stream.deadline else None
        options = request.options if request.HasField("options") else None
        await arm.move_through_joint_positions(  # type: ignore[attr-defined]
            list(request.positions),
            options,
            extra=struct_to_dict(request.extra),
            timeout=timeout,
            metadata=stream.metadata,
        )
        await stream.send_message(MoveThroughJointPositionsResponse())

    async def MoveThroughJointPositionsStreamed(
        self, stream: Stream[MoveThroughJointPositionsStreamedRequest, MoveThroughJointPositionsStreamedResponse]
    ) -> None:
        import asyncio

        session = None
        arm = None
        try:
            async for request in stream:
                if arm is None:
                    arm = self.get_resource(request.name)
                which = request.WhichOneof("message")
                if which == "init":
                    session = arm.start_streamed_move(struct_to_dict(request.init.extra))  # type: ignore[attr-defined]
                elif which == "batch":
                    if session is None:
                        session = arm.start_streamed_move(None)  # type: ignore[attr-defined]
                    points: list = []
                    for p in request.batch.points:
                        points.append((_duration_seconds(p.time), list(p.positions.values)))
                    session.feed(points)
                    await stream.send_message(
                        MoveThroughJointPositionsStreamedResponse(
                            ack=MoveThroughJointPositionsStreamedResponse.BatchAck()
                        )
                    )
            if session is not None:
                await asyncio.to_thread(session.finish)
        except BaseException:
            if session is not None:
                session.abort()
            raise

    async def Get3DModels(self, stream: Stream[Get3DModelsRequest, Get3DModelsResponse]) -> None:
        request = await stream.recv_message()
        assert request is not None
        arm = self.get_resource(request.name)
        timeout = stream.deadline.time_remaining() if stream.deadline else None
        models = await arm.get_3d_models(extra=struct_to_dict(request.extra), timeout=timeout)  # type: ignore[attr-defined]
        await stream.send_message(Get3DModelsResponse(models=models))


def install() -> None:
    """Replace the SDK's arm servicer with B601ArmRPCService. Idempotent."""
    with Registry._lock:
        current = Registry._APIS.get(Arm.API)
        if current is not None and current.rpc_service is B601ArmRPCService:
            return
        Registry._APIS[Arm.API] = ResourceRegistration(
            Arm, B601ArmRPCService, lambda name, channel: ArmClient(name, channel)
        )


def installed() -> bool:
    return Registry.lookup_api(Arm.API).rpc_service is B601ArmRPCService
