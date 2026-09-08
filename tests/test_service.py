"""The servicer swap and the RPC handlers the SDK lacks."""

import math

from google.protobuf.duration_pb2 import Duration
from viam.components.arm import Arm
from viam.proto.common import Get3DModelsRequest
from viam.proto.component.arm import (
    JointPositions,
    MoveOptions,
    MoveThroughJointPositionsRequest,
    MoveThroughJointPositionsStreamedRequest,
    TrajectoryPoint,
)
from viam.resource.registry import Registry
from viam.utils import dict_to_struct

from src.rebot_b601 import arm_service
from src.rebot_b601.arm import B601Arm


class FakeStream:
    def __init__(self, *requests):
        self._requests = list(requests)
        self.sent = []
        self.deadline = None
        self.metadata = {}

    async def recv_message(self):
        return self._requests.pop(0)

    async def send_message(self, msg):
        self.sent.append(msg)

    def __aiter__(self):
        return self

    async def __anext__(self):
        if not self._requests:
            raise StopAsyncIteration
        return self._requests.pop(0)


class FakeManager:
    def __init__(self, resources):
        self._resources = resources

    def get_resource(self, of_type, name):
        return self._resources[name.name]


def make_service(arm):
    svc = arm_service.B601ArmRPCService.__new__(arm_service.B601ArmRPCService)
    svc.get_resource = lambda name: arm  # type: ignore[method-assign]
    return svc


def test_registry_swap_is_installed_and_idempotent():
    assert arm_service.installed()
    arm_service.install()
    reg = Registry.lookup_api(Arm.API)
    assert reg.rpc_service is arm_service.B601ArmRPCService
    # the swapped servicer still serves every RPC in the arm service mapping
    mapping = arm_service.B601ArmRPCService.__mapping__
    assert callable(mapping)
    for rpc in ("MoveThroughJointPositions", "MoveThroughJointPositionsStreamed", "Get3DModels"):
        assert getattr(arm_service.B601ArmRPCService, rpc) is not getattr(arm_service.ArmRPCService, rpc), (
            f"{rpc} must be overridden"
        )


async def test_move_through_joint_positions_rpc(fast_arm):
    arm, ctrl = fast_arm
    svc = make_service(arm)
    req = MoveThroughJointPositionsRequest(
        name="arm",
        positions=[JointPositions(values=[10, 0, 0, 0, 0, 0]), JointPositions(values=[10, -10, 0, 0, 0, 0])],
        options=MoveOptions(max_vel_degs_per_sec=180),
        extra=dict_to_struct({"acceleration_d": 1000}),
    )
    stream = FakeStream(req)
    await svc.MoveThroughJointPositions(stream)
    assert len(stream.sent) == 1
    assert math.isclose(math.degrees(ctrl.motors[2].pos), -10.0, abs_tol=1.0)


async def test_streamed_rpc(fast_arm):
    arm, ctrl = fast_arm
    svc = make_service(arm)

    def point(t, q):
        d = Duration()
        d.FromNanoseconds(int(t * 1e9))
        return TrajectoryPoint(time=d, positions=JointPositions(values=q))

    reqs = [
        MoveThroughJointPositionsStreamedRequest(name="arm", init=MoveThroughJointPositionsStreamedRequest.Init()),
        MoveThroughJointPositionsStreamedRequest(
            name="arm",
            batch=MoveThroughJointPositionsStreamedRequest.TrajectoryBatch(
                points=[point(0.0, [0] * 6), point(0.05, [4, 0, 0, 0, 0, 0])]
            ),
        ),
        MoveThroughJointPositionsStreamedRequest(
            name="arm",
            batch=MoveThroughJointPositionsStreamedRequest.TrajectoryBatch(points=[point(0.1, [8, 0, 0, 0, 0, 0])]),
        ),
    ]
    stream = FakeStream(*reqs)
    await svc.MoveThroughJointPositionsStreamed(stream)
    assert len(stream.sent) == 2  # one ack per batch
    assert math.isclose(math.degrees(ctrl.motors[1].pos), 8.0, abs_tol=1.0)


async def test_get_3d_models_rpc(fast_arm):
    arm, _ = fast_arm
    svc = make_service(arm)
    stream = FakeStream(Get3DModelsRequest(name="arm"))
    await svc.Get3DModels(stream)
    assert "link1" in stream.sent[0].models


def test_module_entrypoint_installs_servicer(factory):
    import importlib

    import src.main  # noqa: F401

    importlib.reload(src.main)
    assert arm_service.installed()
    assert str(B601Arm.MODEL) == "devrel:rebot-b601:arm"
