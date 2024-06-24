import time
import pydrake
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.parsing import Parser
from pydrake.all import (
    StartMeshcat,
    AddMultibodyPlantSceneGraph,
    AddDefaultVisualization,
)
import numpy as np

meshcat = StartMeshcat()

# 创建DiagramBuilder
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
parser = Parser(plant)
urdf_file = (
    "/home/rongman/workspace/drake-docker/box.urdf"
)
parser.AddModels(urdf_file)
plant.Finalize()

# # 添加MeshcatVisualizer
AddDefaultVisualization(builder=builder, meshcat=meshcat)

diagram = builder.Build()
context = diagram.CreateDefaultContext()
plant_context = plant.GetMyContextFromRoot(context)

diagram.ForcedPublish(context)

num_positions = plant.num_positions()
print("Number of positions:", num_positions)

num_velocities = plant.num_velocities()
print("Number of velocities:", num_velocities)

num_actuators = plant.num_actuators()
print("Number of actuators:", num_actuators)

num_joints = plant.num_actuated_dofs()
print("Number of joints:", num_joints)

# 设置初始关节状态
q0 = np.array([0.0])  # 初始关节位置
v0 = np.array([0.0])  # 初始关节速度
plant.SetPositions(plant_context, q0)
plant.SetVelocities(plant_context, v0)

# 创建仿真器并进行仿真
simulator = Simulator(diagram)
simulator.Initialize()

# 控制box模型运动的关节索引（假设只有一个关节）
joint_index = plant.GetJointByName("box_joint").index()
control_input = pydrake.systems.framework.BasicVector(0)  # 大小为0的控制输入
plant.get_actuation_input_port().FixValue(plant_context, control_input)

simulator.AdvanceTo(1.0)

time.sleep(1e5)
