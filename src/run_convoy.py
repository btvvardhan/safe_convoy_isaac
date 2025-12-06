# src/run_convoy.py
#
# Leader–follower convoy using REAL Isaac assets:
#   - Environment: Simple Warehouse (warehouse_multiple_shelves.usd)
#   - Robots: NVIDIA Jetbot (differential drive)
#
# Control (still simple P-control for now):
#   - Single-integrator style formation control in 2D (x, y)
#   - Converted to (v, ω) unicycle commands
#   - Sent to Jetbot wheels via DifferentialController
#
# Goal: Confirm that warehouse + Jetbots + controllers are wired correctly.

from isaacsim import SimulationApp  # Standalone launcher

# Launch Isaac Sim before other Omniverse imports
simulation_app = SimulationApp({"headless": False})

import numpy as np

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path  # matches mobile_robot_controllers example 

from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)


def quat_to_yaw(quat: np.ndarray) -> float:
    """Convert quaternion (x, y, z, w) to yaw (rotation around z)."""
    x, y, z, w = quat
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


def si_to_unicycle(u_xy: np.ndarray, theta: float, v_max: float, w_max: float):
    """
    Map single-integrator velocity u = [u_x, u_y] in world frame
    to unicycle (v, omega) given current heading theta.
    """
    if np.linalg.norm(u_xy) < 1e-6:
        return 0.0, 0.0

    desired_dir = np.arctan2(u_xy[1], u_xy[0])
    angle_err = np.arctan2(
        np.sin(desired_dir - theta), np.cos(desired_dir - theta)
    )  # wrap to [-pi, pi]

    v = np.linalg.norm(u_xy) * np.cos(angle_err)
    v = float(np.clip(v, -v_max, v_max))

    omega = 2.0 * angle_err  # simple gain
    omega = float(np.clip(omega, -w_max, w_max))

    return v, omega


def create_world_and_environment():
    """Create a World and load the Simple Warehouse environment."""
    # Create world
    world = World()
    world.scene.add_default_ground_plane()

    assets_root = get_assets_root_path()
    if assets_root is None:
        raise RuntimeError(
            "Could not find Nucleus assets root. "
            "Open Isaac Sim GUI once and connect to the /Isaac content server."
        )

    # Simple Warehouse environment
    # Check in the Asset Browser that this path exists:
    # Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd 
    env_usd_path = (
        assets_root
        + "/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    )

    add_reference_to_stage(usd_path=env_usd_path, prim_path="/World/Warehouse")

    print("[INFO] Assets root:", assets_root)
    print("[INFO] Loaded warehouse from:", env_usd_path)

    return world, assets_root


def create_jetbot_convoy(world: World, assets_root: str, num_followers: int = 3):
    """
    Spawn a Jetbot leader + followers using WheeledRobot.

    Returns:
        robots    : list[WheeledRobot] (robots[0] is leader)
        ctrls     : list[DifferentialController]
        offsets   : list[np.ndarray] follower formation offsets relative to leader
    """
    robots = []
    controllers = []

    # IMPORTANT: Jetbot path (matches docs) 
    jetbot_usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"

    print("[INFO] Using Jetbot asset:", jetbot_usd)

    # Leader initial pose (left side of warehouse)
    leader_start_pos = np.array([-4.0, 0.0, 0.0])  # x, y, z

    leader = world.scene.add(
        WheeledRobot(
            prim_path="/World/Jetbot_0",
            name="jetbot_0",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jetbot_usd,
            position=leader_start_pos,
        )
    )
    robots.append(leader)

    # Followers behind leader in x-direction
    spacing = 0.7
    for i in range(num_followers):
        x = leader_start_pos[0] - (i + 1) * spacing
        y = leader_start_pos[1]
        follower = world.scene.add(
            WheeledRobot(
                prim_path=f"/World/Jetbot_{i+1}",
                name=f"jetbot_{i+1}",
                wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                create_robot=True,
                usd_path=jetbot_usd,
                position=np.array([x, y, leader_start_pos[2]]),
            )
        )
        robots.append(follower)

    # Differential controllers for each robot (values from docs) 
    for i, _ in enumerate(robots):
        ctrl = DifferentialController(
            name=f"diff_ctrl_{i}",
            wheel_radius=0.03,
            wheel_base=0.1125,
            max_linear_speed=0.5,
            max_angular_speed=2.0,
        )
        controllers.append(ctrl)

    # Formation offsets relative to leader (in world frame)
    formation_offsets = []
    for i in range(num_followers):
        formation_offsets.append(np.array([-(i + 1) * spacing, 0.0]))

    return robots, controllers, formation_offsets


def main():
    # 1) World + environment
    world, assets_root = create_world_and_environment()

    # 2) Jetbot convoy
    num_followers = 3
    robots, controllers, formation_offsets = create_jetbot_convoy(
        world, assets_root, num_followers=num_followers
    )
    leader = robots[0]

    # 3) Reset world (initializes physics & articulations)
    world.reset()

    # 4) Convoy control parameters
    goal_xy = np.array([4.0, 0.0])  # right side of warehouse
    leader_speed = 0.6
    follower_k = 1.5
    v_max = 0.7
    w_max = 2.0


    def get_xy_theta(robot: WheeledRobot):
        # WheeledRobot provides get_world_pose(), not get_world_poses()
        pos, quat = robot.get_world_pose()  # pos: (3,), quat: (4,)
        xy = np.array([pos[0], pos[1]])
        theta = quat_to_yaw(quat)
        return xy, theta


    def convoy_step(step_size: float):
        # Leader
        leader_xy, leader_theta = get_xy_theta(leader)
        to_goal = goal_xy - leader_xy
        dist_to_goal = np.linalg.norm(to_goal)

        if dist_to_goal > 0.2:
            u_leader = leader_speed * to_goal / (dist_to_goal + 1e-6)
        else:
            u_leader = np.zeros(2)

        v_leader, w_leader = si_to_unicycle(
            u_leader, leader_theta, v_max=v_max, w_max=w_max
        )
        leader_action = controllers[0].forward(command=[v_leader, w_leader])
        leader.apply_action(leader_action)

        # Followers
        leader_xy, leader_theta = get_xy_theta(leader)

        for i in range(1, len(robots)):
            follower = robots[i]
            follower_xy, follower_theta = get_xy_theta(follower)

            offset = formation_offsets[i - 1]
            ref_xy = leader_xy + offset

            error = follower_xy - ref_xy
            u_follower = -follower_k * error

            v_f, w_f = si_to_unicycle(
                u_follower, follower_theta, v_max=v_max, w_max=w_max
            )
            follower_action = controllers[i].forward(command=[v_f, w_f])
            follower.apply_action(follower_action)

    # Register physics callback
    world.add_physics_callback("convoy_step", convoy_step)

    # 5) Main loop
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
