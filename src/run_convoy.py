# src/run_convoy.py
#
# Simple leader–follower convoy in a "warehouse" made from boxes.
# Uses single-integrator style kinematics (like Robotarium):
#   x_{k+1} = x_k + u * dt
#
# Step 1: Start Isaac Sim and create a World.
# Step 2: Add ground plane and shelf-like obstacles.
# Step 3: Add leader + follower "robots" as colored cubes.
# Step 4: Every frame:
#         - compute leader velocity toward goal
#         - compute follower velocities toward formation positions
#         - integrate positions and set world poses
#         - step physics + render

from isaacsim import SimulationApp  # Isaac Sim standalone helper 

# Launch Isaac Sim before any other Omniverse imports
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
import numpy as np


def create_warehouse_layout(world: World):
    """
    Create a simple warehouse-like corridor layout using tall boxes as shelves.
    All dimensions are in meters (default stage units).
    """
    shelf_height = 1.8
    shelf_thickness = 0.1
    shelf_length = 4.0

    color_shelf = np.array([0.4, 0.4, 0.4])

    # Row 1 (y = +0.7)
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/shelf_row_1",
            name="shelf_row_1",
            position=np.array([0.0, 0.7, shelf_height / 2.0]),
            scale=np.array([shelf_length, shelf_thickness, shelf_height]),
            color=color_shelf,
        )
    )

    # Row 2 (y = -0.7)
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/shelf_row_2",
            name="shelf_row_2",
            position=np.array([0.0, -0.7, shelf_height / 2.0]),
            scale=np.array([shelf_length, shelf_thickness, shelf_height]),
            color=color_shelf,
        )
    )

    # Shorter shelves near goal to look like a loading zone
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/shelf_goal_left",
            name="shelf_goal_left",
            position=np.array([1.8, 0.4, shelf_height / 2.0]),
            scale=np.array([0.8, shelf_thickness, shelf_height]),
            color=color_shelf,
        )
    )
    world.scene.add(
        DynamicCuboid(
            prim_path="/World/shelf_goal_right",
            name="shelf_goal_right",
            position=np.array([1.8, -0.4, shelf_height / 2.0]),
            scale=np.array([0.8, shelf_thickness, shelf_height]),
            color=color_shelf,
        )
    )


def create_convoy(world: World, num_followers: int = 3):
    """
    Create leader + followers as colored cubes.

    Returns:
        robots: list[DynamicCuboid] with robots[0] = leader
    """
    robots = []

    # Z position slightly above ground plane
    z_robot = 0.3

    # Leader (red cube) starting near x = -1.8, y = 0
    leader = world.scene.add(
        DynamicCuboid(
            prim_path="/World/robot_0",
            name="robot_0",
            position=np.array([-1.8, 0.0, z_robot]),
            scale=np.array([0.25, 0.25, 0.25]),
            color=np.array([1.0, 0.0, 0.0]),  # red
        )
    )
    robots.append(leader)

    # Followers (blue / green / yellow / magenta cycling)
    follower_colors = [
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([1.0, 1.0, 0.0]),
        np.array([1.0, 0.0, 1.0]),
    ]

    spacing = 0.5  # spacing along x-axis behind leader

    for i in range(num_followers):
        color = follower_colors[i % len(follower_colors)]
        x = -1.8 - (i + 1) * spacing
        y = 0.0
        follower = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/robot_{i+1}",
                name=f"robot_{i+1}",
                position=np.array([x, y, z_robot]),
                scale=np.array([0.25, 0.25, 0.25]),
                color=color,
            )
        )
        robots.append(follower)

    return robots


def run_simulation():
    # 1) Create world and basic environment
    world = World()
    world.scene.add_default_ground_plane()

    # Warehouse shelves
    create_warehouse_layout(world)

    # 2) Spawn robots
    num_followers = 3
    robots = create_convoy(world, num_followers=num_followers)
    leader = robots[0]

    # 3) Reset world once everything is added (Hello-World style) 
    world.reset()

    # Physics timestep (approx) for integration
    try:
        dt = world.get_physics_dt()
    except Exception:
        dt = 1.0 / 60.0

    # 4) Leader–follower parameters
    goal_xy = np.array([2.0, 0.0])  # "unloading zone" on the right
    leader_speed = 0.5              # m/s
    follower_k = 1.5                # gain for followers
    max_speed = 1.0                 # clamp

    # Formation offsets (in world frame, relative to leader)
    # Simple single-file line behind leader:
    formation_offsets = []
    spacing = 0.5
    for i in range(num_followers):
        # offset in x only: follower i is (i+1)*spacing behind leader
        formation_offsets.append(np.array([-(i + 1) * spacing, 0.0]))

    # Helper: extract xy from robot pose
    def get_xy(robot):
        pos, _ = robot.get_world_pose()
        return np.array([pos[0], pos[1]]), pos[2]

    # Helper: saturate 2D velocity
    def clamp_vel(v):
        norm = np.linalg.norm(v)
        if norm > max_speed:
            return v * (max_speed / norm)
        return v

    # 5) Main simulation loop
    num_steps = 2000  # ~num_steps * dt seconds total
    for _ in range(num_steps):
        # --- Leader control: move toward goal ---
        leader_xy, leader_z = get_xy(leader)
        to_goal = goal_xy - leader_xy
        dist_to_goal = np.linalg.norm(to_goal)

        if dist_to_goal > 0.1:
            v_leader = leader_speed * to_goal / (dist_to_goal + 1e-6)
        else:
            v_leader = np.zeros(2)

        v_leader = clamp_vel(v_leader)

        # Integrate leader position
        new_leader_xy = leader_xy + v_leader * dt
        leader.set_world_pose(
            position=np.array([new_leader_xy[0], new_leader_xy[1], leader_z]),
            orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        )

        # --- Followers: track formation around leader ---
        # Update leader_xy after we moved it
        leader_xy, leader_z = get_xy(leader)

        for i in range(1, len(robots)):
            follower = robots[i]
            follower_xy, follower_z = get_xy(follower)

            # desired position = leader position + offset_i
            offset = formation_offsets[i - 1]
            ref_xy = leader_xy + offset

            error = follower_xy - ref_xy

            # simple proportional law u = -k * error
            v_follower = -follower_k * error
            v_follower = clamp_vel(v_follower)

            new_follower_xy = follower_xy + v_follower * dt

            follower.set_world_pose(
                position=np.array(
                    [new_follower_xy[0], new_follower_xy[1], follower_z]
                ),
                orientation=np.array([0.0, 0.0, 0.0, 1.0]),
            )

        # Step physics + render one frame
        world.step(render=True)

    # Clean shutdown
    simulation_app.close()


if __name__ == "__main__":
    run_simulation()
