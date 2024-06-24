import time
import copy
import os

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

from pydrake.geometry import (
    ClippingRange,
    ColorRenderCamera,
    DepthRange,
    DepthRenderCamera,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
    RenderLabel,
    Role,
    StartMeshcat,
)
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import BodyIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import (
    CameraInfo,
    RgbdSensor,
)
from pydrake.visualization import (
    AddDefaultVisualization,
    ColorizeDepthImage,
    ColorizeLabelImage,
)


meshcat = StartMeshcat()


def xyz_rpy_deg(xyz, rpy_deg):
    """Shorthand for defining a pose."""
    rpy_deg = np.asarray(rpy_deg)
    return RigidTransform(RollPitchYaw(rpy_deg * np.pi / 180), xyz)


builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)

iiwa_url = (
    "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf"
)

(left_iiwa,) = Parser(plant, "left").AddModels(url=iiwa_url)
plant.WeldFrames(
    frame_on_parent_F=plant.world_frame(),
    frame_on_child_M=plant.GetFrameByName("iiwa_link_0", left_iiwa),
    X_FM=xyz_rpy_deg([0, -0.5, 0], [0, 0, 0]),
)

(right_iiwa,) = Parser(plant, "right").AddModels(url=iiwa_url)
plant.WeldFrames(
    frame_on_parent_F=plant.world_frame(),
    frame_on_child_M=plant.GetFrameByName("iiwa_link_0", right_iiwa),
    X_FM=xyz_rpy_deg([0, 0.5, 0], [0, 0, 0]),
)

renderer_name = "renderer"
scene_graph.AddRenderer(
    renderer_name, MakeRenderEngineVtk(RenderEngineVtkParams()))

plant.Finalize()
AddDefaultVisualization(builder=builder, meshcat=meshcat)

diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()

Simulator(diagram).Initialize()

step = 500

for i in range(step):
    displacement = np.sin(i * 2 * np.pi) * 0.1  # 根据正弦函数计算平移距离，这里乘以2*pi实现一次往返
    translation = [displacement, 0, 0]
    rotation = RollPitchYaw(0, 0, 0)  # 没有旋转
    X_WL = RigidTransform(rotation, translation)
    plant.SetFreeBodyPose(
    
    time.sleep(0.1)
while True:
    time.sleep(1)
