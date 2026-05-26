
import mink
import mujoco as mj
import numpy as np
import json
from scipy.spatial.transform import Rotation as R
from .params import ROBOT_XML_DICT, IK_CONFIG_DICT
from rich import print

class GeneralMotionRetargeting:
    """General Motion Retargeting (GMR).

    Frozen joints (parallel-mechanism filtering)
    --------------------------------------------
    An IK config JSON may declare an optional top-level field ``frozen_joints``
    listing joint names that must remain at their initial value throughout
    every IK solve. This is used by robots that have parallel four-bar
    mechanisms (e.g. LimX OLI D04's ankle / waist achilles linkages) without
    modifying the original MuJoCo XML.

    Concretely, for every joint name listed in ``frozen_joints``:

    * Its velocity bound in ``mink.VelocityLimit`` is set to ~0, so the QP
      solver does not produce updates along that degree of freedom.
    * After each call to :meth:`retarget`, the corresponding ``qpos`` slice
      is force-reset to its initial value (``0`` for hinge joints, identity
      quaternion ``[1, 0, 0, 0]`` for ball joints) and ``mj_kinematics`` is
      re-run so derived body transforms stay consistent.

    Deployment caveat
    ~~~~~~~~~~~~~~~~~
    The returned ``qpos`` is a *logical* pose: the parallel-mechanism joints
    that GMR keeps at zero are still present in the MuJoCo model along with
    their equality constraints. Feeding the full ``qpos`` directly into a
    physics ``mj_step`` will violate those equality constraints (the
    simulator will explode). Downstream users SHOULD extract only the series
    joint angles (the ones not listed in ``frozen_joints``) and use them as
    motor targets. Forward-kinematics-only visualization (``vis_robot_motion``)
    is unaffected.
    """
    def __init__(
        self,
        src_human: str,
        tgt_robot: str,
        actual_human_height: float = None,
        solver: str="daqp", # change from "quadprog" to "daqp".
        damping: float=5e-1, # change from 1e-1 to 1e-2.
        verbose: bool=True,
        use_velocity_limit: bool=False,
    ) -> None:

        self.tgt_robot = tgt_robot
        # load the robot model
        self.xml_file = str(ROBOT_XML_DICT[tgt_robot])
        if verbose:
            print("Use robot model: ", self.xml_file)
        self.model = mj.MjModel.from_xml_path(self.xml_file)
        
        # Print DoF names in order
        print("[GMR] Robot Degrees of Freedom (DoF) names and their order:")
        self.robot_dof_names = {}
        for i in range(self.model.nv):  # 'nv' is the number of DoFs
            dof_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_JOINT, self.model.dof_jntid[i])
            self.robot_dof_names[dof_name] = i
            if verbose:
                print(f"DoF {i}: {dof_name}")
            
            
        print("[GMR] Robot Body names and their IDs:")
        self.robot_body_names = {}
        for i in range(self.model.nbody):  # 'nbody' is the number of bodies
            body_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_BODY, i)
            self.robot_body_names[body_name] = i
            if verbose:
                print(f"Body ID {i}: {body_name}")
        
        print("[GMR] Robot Motor (Actuator) names and their IDs:")
        self.robot_motor_names = {}
        for i in range(self.model.nu):  # 'nu' is the number of actuators (motors)
            motor_name = mj.mj_id2name(self.model, mj.mjtObj.mjOBJ_ACTUATOR, i)
            self.robot_motor_names[motor_name] = i
            if verbose:
                print(f"Motor ID {i}: {motor_name}")

        # Load the IK config
        with open(IK_CONFIG_DICT[src_human][tgt_robot]) as f:
            ik_config = json.load(f)
        if verbose:
            print("Use IK config: ", IK_CONFIG_DICT[src_human][tgt_robot])
        
        # compute the scale ratio based on given human height and the assumption in the IK config
        if actual_human_height is not None:
            ratio = actual_human_height / ik_config["human_height_assumption"]
        else:
            ratio = 1.0
            
        # adjust the human scale table
        for key in ik_config["human_scale_table"].keys():
            ik_config["human_scale_table"][key] = ik_config["human_scale_table"][key] * ratio
    

        # used for retargeting
        self.ik_match_table1 = ik_config["ik_match_table1"]
        self.ik_match_table2 = ik_config["ik_match_table2"]
        self.human_root_name = ik_config["human_root_name"]
        self.robot_root_name = ik_config["robot_root_name"]
        self.use_ik_match_table1 = ik_config["use_ik_match_table1"]
        self.use_ik_match_table2 = ik_config["use_ik_match_table2"]
        self.human_scale_table = ik_config["human_scale_table"]
        self.ground = ik_config["ground_height"] * np.array([0, 0, 1])

        self.max_iter = 10

        self.solver = solver
        self.damping = damping

        self.human_body_to_task1 = {}
        self.human_body_to_task2 = {}
        self.pos_offsets1 = {}
        self.rot_offsets1 = {}
        self.pos_offsets2 = {}
        self.rot_offsets2 = {}

        self.task_errors1 = {}
        self.task_errors2 = {}

        # Parse optional frozen_joints field (parallel-mechanism filtering).
        # See class docstring for full semantics.
        frozen_joint_names = ik_config.get("frozen_joints", [])
        self.frozen_qpos_addr = self._resolve_frozen_joints(frozen_joint_names)
        # mink.VelocityLimit expects shape (nv_of_joint,) per joint: scalar for
        # hinge/slide (1 dof), length-3 array for ball (3 rotational dofs).
        frozen_velocity_limits = self._build_frozen_velocity_limits(frozen_joint_names)

        self.ik_limits = [mink.ConfigurationLimit(self.model)]
        if use_velocity_limit:
            VELOCITY_LIMITS = {k: 3*np.pi for k in self.robot_motor_names.keys()}
            # Override frozen joints to ~0 velocity within the same limit object.
            VELOCITY_LIMITS.update(frozen_velocity_limits)
            self.ik_limits.append(mink.VelocityLimit(self.model, VELOCITY_LIMITS))
        elif frozen_velocity_limits:
            # Attach a dedicated zero-velocity limit just for the frozen joints.
            self.ik_limits.append(mink.VelocityLimit(self.model, frozen_velocity_limits))

        self.setup_retarget_configuration()
        
        self.ground_offset = 0.0

    def _resolve_frozen_joints(self, joint_names):
        """Resolve a list of joint names into (qadr, dim, init_value) tuples.

        Hinge joints freeze to ``0.0`` (single qpos slot). Ball joints freeze
        to the identity quaternion ``[1, 0, 0, 0]`` (four qpos slots). Free
        joints are rejected because freezing the floating base would defeat
        the whole retargeting pipeline.

        Raises
        ------
        ValueError
            If a joint name is not present in the loaded MuJoCo model, or if
            a joint with an unsupported type (free) is listed.
        """
        resolved = []
        for jname in joint_names:
            try:
                jid = self.model.joint(jname).id
            except KeyError:
                raise ValueError(
                    f"frozen_joints contains unknown joint '{jname}' "
                    f"for robot '{self.tgt_robot}'."
                )
            qadr = int(self.model.jnt_qposadr[jid])
            jtype = int(self.model.jnt_type[jid])
            if jtype == mj.mjtJoint.mjJNT_HINGE or jtype == mj.mjtJoint.mjJNT_SLIDE:
                dim = 1
                init_value = np.array([0.0], dtype=np.float64)
            elif jtype == mj.mjtJoint.mjJNT_BALL:
                dim = 4
                init_value = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
            elif jtype == mj.mjtJoint.mjJNT_FREE:
                raise ValueError(
                    f"frozen_joints may not contain free joint '{jname}' "
                    f"for robot '{self.tgt_robot}'."
                )
            else:
                raise ValueError(
                    f"frozen_joints contains joint '{jname}' with "
                    f"unsupported MuJoCo type {jtype} for robot "
                    f"'{self.tgt_robot}'."
                )
            resolved.append((qadr, dim, init_value))
        return resolved

    def _build_frozen_velocity_limits(self, joint_names):
        """Build the per-joint velocity-limit mapping for ``mink.VelocityLimit``.

        mink requires the limit value shape to match the joint's nv:
          * hinge / slide -> scalar (1 dof)
          * ball          -> array of shape (3,) (3 rotational dofs)
        """
        if not joint_names:
            return {}
        limits = {}
        eps = 1e-9
        for jname in joint_names:
            jid = self.model.joint(jname).id
            jtype = int(self.model.jnt_type[jid])
            if jtype == mj.mjtJoint.mjJNT_BALL:
                limits[jname] = np.array([eps, eps, eps], dtype=np.float64)
            else:
                limits[jname] = eps
        return limits

    def _restore_frozen_joints(self):
        """Force-restore every frozen joint's qpos slice to its initial value
        and refresh derived body transforms via ``mj_kinematics``."""
        if not self.frozen_qpos_addr:
            return
        qpos = self.configuration.data.qpos
        for qadr, dim, init_value in self.frozen_qpos_addr:
            qpos[qadr:qadr + dim] = init_value
        mj.mj_kinematics(self.model, self.configuration.data)

    def setup_retarget_configuration(self):
        self.configuration = mink.Configuration(self.model)
    
        self.tasks1 = []
        self.tasks2 = []
        
        for frame_name, entry in self.ik_match_table1.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task1[body_name] = task
                self.pos_offsets1[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets1[body_name] = R.from_quat(
                    rot_offset, scalar_first=True
                )
                self.tasks1.append(task)
                self.task_errors1[task] = []
        
        for frame_name, entry in self.ik_match_table2.items():
            body_name, pos_weight, rot_weight, pos_offset, rot_offset = entry
            if pos_weight != 0 or rot_weight != 0:
                task = mink.FrameTask(
                    frame_name=frame_name,
                    frame_type="body",
                    position_cost=pos_weight,
                    orientation_cost=rot_weight,
                    lm_damping=1,
                )
                self.human_body_to_task2[body_name] = task
                self.pos_offsets2[body_name] = np.array(pos_offset) - self.ground
                self.rot_offsets2[body_name] = R.from_quat(
                    rot_offset, scalar_first=True
                )
                self.tasks2.append(task)
                self.task_errors2[task] = []

  
    def update_targets(self, human_data, offset_to_ground=False):
        # scale human data in local frame
        human_data = self.to_numpy(human_data)
        human_data = self.scale_human_data(human_data, self.human_root_name, self.human_scale_table)
        human_data = self.offset_human_data(human_data, self.pos_offsets1, self.rot_offsets1)
        human_data = self.apply_ground_offset(human_data)
        if offset_to_ground:
            human_data = self.offset_human_data_to_ground(human_data)
        self.scaled_human_data = human_data

        if self.use_ik_match_table1:
            for body_name in self.human_body_to_task1.keys():
                task = self.human_body_to_task1[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
        
        if self.use_ik_match_table2:
            for body_name in self.human_body_to_task2.keys():
                task = self.human_body_to_task2[body_name]
                pos, rot = human_data[body_name]
                task.set_target(mink.SE3.from_rotation_and_translation(mink.SO3(rot), pos))
            
            
    def retarget(self, human_data, offset_to_ground=False):
        # Update the task targets
        self.update_targets(human_data, offset_to_ground)

        if self.use_ik_match_table1:
            # Solve the IK problem
            curr_error = self.error1()
            dt = self.configuration.model.opt.timestep
            vel1 = mink.solve_ik(
                self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
            )
            self.configuration.integrate_inplace(vel1, dt)
            next_error = self.error1()
            num_iter = 0
            while curr_error - next_error > 0.001 and num_iter < self.max_iter:
                curr_error = next_error
                dt = self.configuration.model.opt.timestep
                vel1 = mink.solve_ik(
                    self.configuration, self.tasks1, dt, self.solver, self.damping, self.ik_limits
                )
                self.configuration.integrate_inplace(vel1, dt)
                next_error = self.error1()
                num_iter += 1

        if self.use_ik_match_table2:
            curr_error = self.error2()
            dt = self.configuration.model.opt.timestep
            vel2 = mink.solve_ik(
                self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
            )
            self.configuration.integrate_inplace(vel2, dt)
            next_error = self.error2()
            num_iter = 0
            while curr_error - next_error > 0.001 and num_iter < self.max_iter:
                curr_error = next_error
                # Solve the IK problem with the second task
                dt = self.configuration.model.opt.timestep
                vel2 = mink.solve_ik(
                    self.configuration, self.tasks2, dt, self.solver, self.damping, self.ik_limits
                )
                self.configuration.integrate_inplace(vel2, dt)
                
                next_error = self.error2()
                num_iter += 1
                
        # Force frozen joints (e.g. parallel four-bar mechanisms) back to
        # their initial value so QP numerical drift does not accumulate.
        self._restore_frozen_joints()

        return self.configuration.data.qpos.copy()


    def error1(self):
        return np.linalg.norm(
            np.concatenate(
                [task.compute_error(self.configuration) for task in self.tasks1]
            )
        )
    
    def error2(self):
        return np.linalg.norm(
            np.concatenate(
                [task.compute_error(self.configuration) for task in self.tasks2]
            )
        )


    def to_numpy(self, human_data):
        for body_name in human_data.keys():
            human_data[body_name] = [np.asarray(human_data[body_name][0]), np.asarray(human_data[body_name][1])]
        return human_data


    def scale_human_data(self, human_data, human_root_name, human_scale_table):
        
        human_data_local = {}
        root_pos, root_quat = human_data[human_root_name]
        
        # scale root
        scaled_root_pos = human_scale_table[human_root_name] * root_pos
        
        # scale other body parts in local frame
        for body_name in human_data.keys():
            if body_name not in human_scale_table:
                continue
            if body_name == human_root_name:
                continue
            else:
                # transform to local frame (only position)
                human_data_local[body_name] = (human_data[body_name][0] - root_pos) * human_scale_table[body_name]
            
        # transform the human data back to the global frame
        human_data_global = {human_root_name: (scaled_root_pos, root_quat)}
        for body_name in human_data_local.keys():
            human_data_global[body_name] = (human_data_local[body_name] + scaled_root_pos, human_data[body_name][1])

        return human_data_global
    
    def offset_human_data(self, human_data, pos_offsets, rot_offsets):
        """the pos offsets are applied in the local frame"""
        offset_human_data = {}
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            offset_human_data[body_name] = [pos, quat]
            # apply rotation offset first
            updated_quat = (R.from_quat(quat, scalar_first=True) * rot_offsets[body_name]).as_quat(scalar_first=True)
            offset_human_data[body_name][1] = updated_quat
            
            local_offset = pos_offsets[body_name]
            # compute the global position offset using the updated rotation
            global_pos_offset = R.from_quat(updated_quat, scalar_first=True).apply(local_offset)
            
            offset_human_data[body_name][0] = pos + global_pos_offset
           
        return offset_human_data
            
    def offset_human_data_to_ground(self, human_data):
        """find the lowest point of the human data and offset the human data to the ground"""
        offset_human_data = {}
        ground_offset = 0.1
        lowest_pos = np.inf

        for body_name in human_data.keys():
            # only consider the foot/Foot
            if "Foot" not in body_name and "foot" not in body_name:
                continue
            pos, quat = human_data[body_name]
            if pos[2] < lowest_pos:
                lowest_pos = pos[2]
                lowest_body_name = body_name
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            offset_human_data[body_name] = [pos, quat]
            offset_human_data[body_name][0] = pos - np.array([0, 0, lowest_pos]) + np.array([0, 0, ground_offset])
        return offset_human_data

    def set_ground_offset(self, ground_offset):
        self.ground_offset = ground_offset

    def apply_ground_offset(self, human_data):
        for body_name in human_data.keys():
            pos, quat = human_data[body_name]
            human_data[body_name][0] = pos - np.array([0, 0, self.ground_offset])
        return human_data
