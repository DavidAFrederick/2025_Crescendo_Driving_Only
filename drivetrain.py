# Aug 13, 2025
import math
from commands2.button import CommandXboxController
import wpilib.drive
from wpilib import RobotBase, DriverStation
import wpilib.simulation
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    DifferentialDriveOdometry,
    DifferentialDriveKinematics,
    ChassisSpeeds,
    DifferentialDriveWheelSpeeds,
)
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.filter import SlewRateLimiter
from wpilib import SmartDashboard, Field2d
from commands2 import Subsystem, Command, cmd, InstantCommand, PIDCommand
from phoenix6 import StatusCode
from phoenix6.configs import (
    TalonFXConfiguration,
    TalonFXConfigurator,
    MotionMagicConfigs,
    Slot0Configs,
)
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls.follower import Follower
from phoenix6.signals.spn_enums import (
    InvertedValue,
    NeutralModeValue,
    FeedbackSensorSourceValue,
    MotionMagicIsRunningValue,
    ControlModeValue,
)
from phoenix6.sim import ChassisReference
from phoenix6.controls import (
    DutyCycleOut,
    VoltageOut,
    MotionMagicVoltage,
)
from phoenix6.unmanaged import feed_enable
import navx

from typing import Callable
import constants

VISION_KP = 0.012
FEEDFORWARD = 0.1


class DriveTrain(Subsystem):
    __DRIVER_DEADBAND = 0.1
    __FORWARD_SLEW = 3  # 1/3 of a second to full speed
    __CLAMP_SPEED = 0.2   ###   SLOWED DOWN THE ROBOT
    __TURN_PID_SPEED = 0.3

    def __init__(self, test_mode=False) -> None:
        super().__init__()
        self._gyro: navx.AHRS = navx.AHRS.create_spi()

        # Create the output objects for the talons, currently one each for
        # the following modes: VoltageOut, PercentOutput, and MotionMagic
        self.__create_output_objects()

        # Create the PID controller setup for turning
        self.__create_turn_pid_objects()
        self.create_AprilTag_tracking_pid_object()    # Used for AprilTags


        # Create the FF and PID for paths
        self.__create_path_pid_objects()

        # Apply all the configurations to the left and right side Talons
        self.__configure_left_side_drive()
        self.__configure_right_side_drive()

        # Create the Odometry tracker
        self._odometry: DifferentialDriveOdometry = DifferentialDriveOdometry(
            self._gyro.getRotation2d(), 0, 0
        )

        self._kinematics: DifferentialDriveKinematics = DifferentialDriveKinematics(
            constants.DT_TRACKWIDTH_METERS
        )

        if RobotBase.isSimulation():
            self.__configure_simulation()

        self._field = Field2d()
        SmartDashboard.putData("MyField", self._field)

        SmartDashboard.putData("Navx", self._gyro)

        self._forward_limiter: SlewRateLimiter = SlewRateLimiter(self.__FORWARD_SLEW)

        self._test_mode = test_mode
        if self._test_mode:
            SmartDashboard.putNumber("ClampSpeed", 0.3)

        self._field = Field2d()
        SmartDashboard.putData("MyField", self._field)


    def __configure_simulation(self) -> None:
        self._sim_gyro = wpilib.simulation.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self._sim_gyro.getDouble("Yaw")
        # self.navx_comp = self._sim_gyro.getDouble("CompassHeading")

        self._system = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
        self._drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self._system,
            constants.DT_TRACKWIDTH_METERS,
            DCMotor.falcon500(4),
            constants.DT_GEAR_RATIO,
            constants.DT_WHEEL_RADIUS_INCHES,
        )

        self._left_leader.sim_state.orientation = ChassisReference.Clockwise_Positive
        self._right_leader.sim_state.orientation = (
            ChassisReference.CounterClockwise_Positive
        )

    def __create_output_objects(self) -> None:
        self._left_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        # self._left_volts_out.update_freq_hz = 0
        self._right_volts_out: VoltageOut = VoltageOut(0, enable_foc=False)
        # self._right_volts_out.update_freq_hz = 0

        self._left_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)
        self._right_percent_out: DutyCycleOut = DutyCycleOut(0, enable_foc=False)

        self._mm_out: MotionMagicVoltage = MotionMagicVoltage(0, enable_foc=False)

    def __create_turn_pid_objects(self) -> None:
        self._turn_setpoint = 0
        self._turn_tolerance = 2  # within 2 degrees we'll call good enough
        if RobotBase.isSimulation():
            self._turn_pid_controller: PIDController = PIDController(0.002, 0.0, 0.0001)
            self._turn_kF = 0.049
        else:
            # These must be tuned
            self._turn_pid_controller: PIDController = PIDController(0.02, 0, 0.001)
            self._turn_kF = 0.1  # TODO Tune me

        self._turn_pid_controller.enableContinuousInput(-180, 180)
        self._turn_pid_controller.setTolerance(1)
        SmartDashboard.putData("Turn PID", self._turn_pid_controller)

    def __create_path_pid_objects(self) -> None:
        if RobotBase.isSimulation():
            self._path_left_pid_controller: PIDController = PIDController(0.002, 0, 0)
            self._path_right_pid_controller: PIDController = PIDController(0.002, 0, 0)
            self._path_feedforward: SimpleMotorFeedforwardMeters = (
                SimpleMotorFeedforwardMeters(
                    constants.DT_KS_VOLTS_SIM, constants.DT_KV_VOLTSECONDS_METER_SIM, 0
                )
            )
        else:
            self._path_left_pid_controller: PIDController = PIDController(
                0.01, 0, 0.001
            )
            self._path_right_pid_controller: PIDController = PIDController(
                0.01, 0, 0.001
            )
            self._path_feedforward: SimpleMotorFeedforwardMeters = (
                SimpleMotorFeedforwardMeters(
                    constants.DT_KS_VOLTS, constants.DT_KV_VOLTSECONDS_METER, 0
                )
            )

    def __configure_left_side_drive(self) -> None:
        self._left_leader = TalonFX(constants.DT_LEFT_LEADER)
        self._left_follower = TalonFX(constants.DT_LEFT_FOLLOWER)
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        config.feedback.sensor_to_mechanism_ratio = constants.DT_GEAR_RATIO

        # Setup current limits for supply and stator
        config.current_limits.supply_current_limit = 40
        config.current_limits.supply_current_limit_enable = True
        # 1 second of spikes before limiting # TODO Tune this
        config.current_limits.supply_current_threshold = 1.0
        config.current_limits.stator_current_limit = 80
        config.current_limits.stator_current_limit_enable = False

        # Apply the configuration to the motors
        for i in range(0, 6):  # Try 5 times
            ret = self._left_leader.configurator.apply(config)
            if ret == StatusCode.is_ok:
                break

        for i in range(0, 6):  # Try 5 times
            ret = self._left_follower.configurator.apply(config)
            if ret == StatusCode.is_ok:
                break

        # self._left_follower.set_control(Follower(self._left_leader.device_id, False))
        self._left_leader.sim_state.Orientation = ChassisReference.Clockwise_Positive
        # self._left_follower.sim_state.Orientation = (
        #     ChassisReference.Clockwise_Positive
        # )

        # Set the left follower to only follow master
        follow_request = Follower(constants.DT_LEFT_LEADER, False)
        self._left_follower.set_control(follow_request)

        self._left_leader.set_position(0)

    def __configure_right_side_drive(self) -> None:
        self._right_leader = TalonFX(constants.DT_RIGHT_LEADER)
        self._right_follower = TalonFX(constants.DT_RIGHT_FOLLOWER)
        # Applying a new configuration will erase all other config settings since we start with a blank config
        # so each setting needs to be explicitly set here in the config method
        config = TalonFXConfiguration()

        # Set the left side motors to be counter clockwise positive
        config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        # Set the motors to electrically stop instead of coast
        config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # PID controls will use integrated encoder
        config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR

        config.feedback.sensor_to_mechanism_ratio = constants.DT_GEAR_RATIO

        # Setup current limits for supply and stator
        config.current_limits.supply_current_limit = 40
        config.current_limits.supply_current_limit_enable = True
        # 1 second of spikes before limiting # TODO Tune this
        config.current_limits.supply_current_threshold = 1.0
        config.current_limits.stator_current_limit = 80
        config.current_limits.stator_current_limit_enable = False

        # Apply the configuration to the motors
        for i in range(0, 6):  # Try 5 times
            ret = self._right_leader.configurator.apply(config)
            if ret == StatusCode.is_ok:
                break

        for i in range(0, 6):  # Try 5 times
            ret = self._right_follower.configurator.apply(config)
            if ret == StatusCode.is_ok:
                break

        # self._right_follower.set_control(Follower(self._right_leader.device_id, False))
        self._right_leader.sim_state.Orientation = (
            ChassisReference.CounterClockwise_Positive
        )
        # self._right_follower.sim_state.Orientation = ChassisReference.CounterClockwise_Positive

        # Set the right side follower to go with leader
        follow_request = Follower(constants.DT_RIGHT_LEADER, False)
        self._right_follower.set_control(follow_request)

        self._right_leader.set_position(0)

    ########################### Drivetrain Drive methods #######################

    def drive_teleop(self, forward: float, turn: float, percent_out=False):
        if self._test_mode:
            self.__CLAMP_SPEED = SmartDashboard.getNumber("ClampSpeed", 0.3)
        forward = self.__deadband(forward, self.__DRIVER_DEADBAND)
        turn = self.__deadband(turn, self.__DRIVER_DEADBAND)

        turn = self.__clamp(turn, self.__CLAMP_SPEED)
        forward = self.__clamp(forward, self.__CLAMP_SPEED)

        if percent_out:
            self.__drive_teleop_percent(forward, turn)
        else:
            self.__drive_teleop_volts(forward, turn)

    def drive_pid_turn(self, turn: float) -> None:
        if turn < 0:
            # Turning right
            if turn > -self._turn_kF:
                turn = -self._turn_kF
        elif turn > 0:
            # Turning left
            if turn < self._turn_kF:
                turn = self._turn_kF

        self.__drive_teleop_volts(0, turn)

    def __drive_teleop_volts(self, forward: float, turn: float) -> None:

        speeds = wpilib.drive.DifferentialDrive.curvatureDriveIK(forward, turn, True)
        # print (" forward: ", forward,  "    turn:  ", turn)

        self._left_volts_out.output = speeds.left * 12.0
        self._right_volts_out.output = speeds.right * 12.0

        self._left_leader.set_control(self._left_volts_out)
        self._right_leader.set_control(self._right_volts_out)

    def __drive_teleop_percent(self, forward: float, turn: float) -> None:

        speeds = wpilib.drive.DifferentialDrive.curvatureDriveIK(forward, turn, True)

        self._left_percent_out.output = speeds.left
        self._right_percent_out.output = speeds.right

        self._left_leader.set_control(self._left_percent_out)
        self._right_leader.set_control(self._right_percent_out)

    def drive_velocity_volts(self, left: float, right: float) -> None:
        left_pid = self._path_left_pid_controller.calculate(
            self._left_leader.get_velocity().value_as_double * constants.DT_GEAR_RATIO,
            left,
        )
        right_pid = self._path_right_pid_controller.calculate(
            self._right_leader.get_velocity().value_as_double * constants.DT_GEAR_RATIO,
            right,
        )
        self._left_volts_out.output = self._path_feedforward.calculate(left) + left_pid
        self._right_volts_out.output = (
            self._path_feedforward.calculate(right) + right_pid
        )

        if self._test_mode:
            SmartDashboard.putNumber("LeftPID", left_pid)
            SmartDashboard.putNumber("RightPID", right_pid)
            SmartDashboard.putNumber("LeftSpeed", left)
            SmartDashboard.putNumber("RightSpeed", right)

        self._left_leader.set_control(self._left_volts_out)
        self._right_leader.set_control(self._right_volts_out)

    def driveSpeeds(self, speeds: ChassisSpeeds) -> None:
        speeds: DifferentialDriveWheelSpeeds = self._kinematics.toWheelSpeeds(speeds)
        self.drive_velocity_volts(speeds.left, speeds.right)


    def turn_ccw_positive(self, speed: float) -> None:
        if RobotBase.isSimulation():
            speed = self.__clamp(speed, 0.05)
            wpilib.SmartDashboard.putNumber("TurnSpeed", speed)
        else:
            speed = self.__clamp(speed, self.__TURN_PID_SPEED)

        if speed < 0:
            # Turn CCW
            left_speed = -speed
            right_speed = speed
        elif speed > 0:
            left_speed = speed
            right_speed = -speed
        else:
            left_speed = 0
            right_speed = 0

        self._left_percent_out.output = left_speed
        self._right_percent_out.output = right_speed

        self._left_leader.set_control(self._left_percent_out)
        self._right_leader.set_control(self._right_percent_out)

    def __turn_with_pid(self) -> None:
        curr_angle = self.__get_gyro_heading()
        setpoint = self._turn_pid_controller.getSetpoint()

        pidoutput = self._turn_pid_controller.calculate(curr_angle)

        # if curr_angle > setpoint and pidoutput > 0:
        #     # We need to turn right, which is -
        #     pidoutput *= -1
        # elif curr_angle < setpoint and pidoutput < 0:
        #     # We need to turn left, which - from here
        #     pidoutput *= -1

        # Promote the value to at least the KF
        if (pidoutput < 0) and (pidoutput > -self._turn_kF):
            pidoutput = -self._turn_kF
        elif (pidoutput > 0) and (pidoutput < self._turn_kF):
            pidoutput = self._turn_kF

        if RobotBase.isSimulation():
            wpilib.SmartDashboard.putNumber("TurnPIDOut", pidoutput)
        self.turn_ccw_positive(pidoutput)

#=====(April Tag PID Controller)============================

    def create_AprilTag_tracking_pid_object(self) -> None:
        self._tag_setpoint = 0
        self._tag_tolerance = 2
        # self._tag_pid_controller: PIDController = PIDController(0.02, 0, 0.001)
        # self._tag_pid_controller: PIDController = PIDController(0.05, 0, 0.001)   #  Original
        self._tag_pid_controller: PIDController = PIDController(0.01, 0, 0.01)   #  Original
        self._tag_kF = 0.1  # TODO Tune me
        self._tag_pid_controller.setTolerance(self._tag_tolerance)
        SmartDashboard.putData("Turn TAG PID", self._tag_pid_controller)

    def config_AprilTag_turn_command(self, desired_angle: float) -> None:
        self._turn_setpoint = desired_angle
        self._tag_pid_controller.setSetpoint(self._tag_setpoint)
        self._tag_pid_controller.setTolerance(self._tag_tolerance)
        wpilib.SmartDashboard.putNumber("TAG Turn Setpoint", self._turn_setpoint)

    def calculate_AprilTag_turn_error(self, tag_yaw_offset: float) -> float:
        self.tag_yaw_offset = tag_yaw_offset
        self.heading_error = self._tag_pid_controller.calculate(self.tag_yaw_offset)
        print (">>>>> set point: ", self._tag_pid_controller.getSetpoint()  , "   Input: ", self.tag_yaw_offset)
        wpilib.SmartDashboard.putNumber("TAG Error", self._turn_setpoint)
        return self.heading_error


    def AprilTag_at_turn_setpoint(self) -> bool:
        return self._tag_pid_controller.atSetpoint()

    ################## Drive train Helpers ##########################

    def __deadband(self, input: float, abs_min: float) -> float:
        """
        If the value is between 0 and the abs_min value passed in,
        return 0.

        This eliminates joystick drift on input
        """
        if input < 0 and input > (abs_min * -1):
            input = 0

        if input > 0 and input < abs_min:
            input = 0

        return input

    def __clamp(self, input: float, abs_max: float) -> None:
        """
        Set a max speed for a given input
        """
        if input < 0 and input < (abs_max * -1):
            input = abs_max * -1
        elif input > 0 and input > abs_max:
            input = abs_max

        return input

    def __config_turn_command(self, desired_angle: float) -> None:
        # self._turn_setpoint = desired_angle + self._gyro.getYaw()
        self._turn_setpoint = desired_angle
        self._turn_pid_controller.setSetpoint(self._turn_setpoint)
        self._turn_pid_controller.setTolerance(self._turn_tolerance)
        wpilib.SmartDashboard.putNumber("Turn Setpoint", self._turn_setpoint)

    def __at_turn_setpoint(self) -> bool:
        # curr_angle = self._gyro.getYaw()

        # wpilib.SmartDashboard.putBoolean(
        #     "At Setpoint", self._turn_pid_controller.atSetpoint()
        # )
        # return abs(curr_angle - self._turn_setpoint) < self._turn_tolerance
        return self._turn_pid_controller.atSetpoint()

    def getHeading(self) -> float:
        return self.__get_gyro_heading()

    def __get_gyro_heading(self) -> float:
        angle = math.fmod(-self._gyro.getAngle(), 360)

        if angle < 0:
            return angle if angle >= -180 else angle + 360
        else:
            return angle if angle <= 180 else angle - 360

    def __set_gyro_heading(self) -> None:
        pass

    def get_robot_pose(self) -> Pose2d:
        return self._odometry.getPose()

    def get_wheel_speeds(self) -> ChassisSpeeds:
        diff_speed: DifferentialDriveWheelSpeeds = DifferentialDriveWheelSpeeds(
            self.__rps_to_mps(self._left_leader.get_velocity().value_as_double),
            self.__rps_to_mps(self._right_leader.get_velocity().value_as_double),
        )
        return self._kinematics.toChassisSpeeds(diff_speed)

    def reset_odometry(self, pose: Pose2d) -> None:
        self._odometry.resetPosition(
            self._gyro.getRotation2d(),
            0,
            0,
            pose,
        )

    def reset_encoders(self) -> None:
        self._left_leader.set_position(0)
        self._right_leader.set_position(0)

    def reset_drivetrain(self) -> None:
        self._gyro.setAngleAdjustment(0)
        self._gyro.reset()

        self.reset_encoders()

    def set_alliance_offset(self) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            self._gyro.setAngleAdjustment(180)
        else:
            self._gyro.setAngleAdjustment(0)

    ################### Periodic Updates for the Subsystems ######################

    def periodic(self) -> None:
        # SmartDashboard.putNumber("LeftEncoder", self._left_leader.get_position().value)
        # SmartDashboard.putNumber(
        #     "RightEncoder", self._right_leader.get_position().value
        # )

        pose = self._odometry.update(
            Rotation2d().fromDegrees(self.__get_gyro_heading()),
            self.__rotations_to_meters(
                self._left_leader.get_position().value_as_double
            ),
            self.__rotations_to_meters(
                self._right_leader.get_position().value_as_double
            ),
        )

        self._field.setRobotPose(pose)
        SmartDashboard.putNumber("CCW Angle", self.__get_gyro_heading())
        SmartDashboard.putNumber("LeftVelOut", self._left_volts_out.output)
        SmartDashboard.putNumber("rightVelOut", self._right_volts_out.output)

    def simulationPeriodic(self) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(constants.ROBOT_PERIOD_MS * 2)

        # Start the motor simulation work flow by passing robot battery voltage to sim motors
        self._left_leader.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._right_leader.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._left_follower.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        self._right_follower.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )

        # Apply the motor inputs to the simulation
        self._drivesim.setInputs(
            self._left_leader.sim_state.motor_voltage,
            self._right_leader.sim_state.motor_voltage,
        )

        # advance the simulation model a timing loop
        self._drivesim.update(constants.ROBOT_PERIOD_MS)

        # Update the motor values with the new calculated values from the physics engine
        self._left_leader.sim_state.set_raw_rotor_position(
            self.__feet_to_encoder_rotations(self._drivesim.getLeftPositionFeet())
        )
        self._left_leader.sim_state.set_rotor_velocity(
            self.__velocity_feet_to_rps(self._drivesim.getLeftVelocityFps())
        )
        self._right_leader.sim_state.set_raw_rotor_position(
            self.__feet_to_encoder_rotations(self._drivesim.getRightPositionFeet())
        )
        self._right_leader.sim_state.set_rotor_velocity(
            self.__velocity_feet_to_rps(self._drivesim.getRightVelocityFps())
        )
        self._left_follower.sim_state.set_raw_rotor_position(
            self.__feet_to_encoder_rotations(self._drivesim.getLeftPositionFeet())
        )
        self._left_follower.sim_state.set_rotor_velocity(
            self.__velocity_feet_to_rps(self._drivesim.getLeftVelocityFps())
        )
        self._right_follower.sim_state.set_raw_rotor_position(
            self.__feet_to_encoder_rotations(self._drivesim.getRightPositionFeet())
        )
        self._right_follower.sim_state.set_rotor_velocity(
            self.__velocity_feet_to_rps(self._drivesim.getRightVelocityFps())
        )

        # Update the gyro simulation
        degrees = self._drivesim.getHeading().degrees()

        # The drive train simulator is CCW positive, and the Navx is not.  So when we set
        # the Yaw value here, negate it to represent the real world Navx as well.
        self.navx_yaw.set(-degrees)
        # self.navx_comp.set(degrees)

    ############# Drivetrain Odometry methods ###################

    def __feet_to_encoder_rotations(self, distance_in_feet: float) -> float:
        #                    feet * 12
        # rotations = ---------------------  * gear ratio
        #             2pi * wheel_diameter
        wheel_rotations = (distance_in_feet * 12) / (
            math.pi * constants.DT_WHEEL_DIAMETER
        )
        motor_rotations = wheel_rotations * constants.DT_GEAR_RATIO
        return motor_rotations

    def __velocity_feet_to_rps(self, velocity_in_feet: float) -> float:
        #             velocity * 12
        # rps = -------------------------- * gear ratio
        #          2pi * wheel diameter
        wheel_rotations_per_second = (velocity_in_feet * 12) / (
            math.pi * constants.DT_WHEEL_DIAMETER
        )
        motor_rotations_per_second = (
            wheel_rotations_per_second * constants.DT_GEAR_RATIO
        )

        return motor_rotations_per_second

    def __rotations_to_meters(self, rotations: float) -> float:
        return rotations * constants.DT_WHEEL_CIRCUMFERENCE_METERS

    def should_flip_path(self) -> bool:
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        # return DriverStation.getAlliance() == DriverStation.Alliance.kRed
        return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def __rps_to_mps(self, rotations: float) -> float:
        return rotations * constants.DT_WHEEL_CIRCUMFERENCE_METERS

    ################################# Drivetrain Command Methods ############################
    def configure_turn_pid(self, desired_angle: float) -> Command:
        return cmd.runOnce(lambda: self.__config_turn_command(desired_angle))

    def turn_with_pid(self) -> Command:
        return (
            cmd.run(lambda: self.__turn_with_pid(), self)
            .until(lambda: self.__at_turn_setpoint())
            .withName("TurnWithPID")
        )

class TeleopDriveWithVision(Command):
    def __init__(
        self,
        dt: DriveTrain,
        _yaw_getter: Callable[[], float],
        controller: CommandXboxController,
        flipped_controls=False,
    ):
        self._dt = dt
        self._yaw_getter = _yaw_getter
        self._controller = controller
        self._flipped = flipped_controls
        SmartDashboard.putNumber("VisionKP", 0.012)
        SmartDashboard.putNumber("VisionFF", 0.1)
        self.addRequirements(self._dt)

    def execute(self):
        forward = -self._controller.getLeftY()
        if self._flipped:
            # Invert the translation
            forward *= -1
        yaw: float = self._yaw_getter()
        if 1000 == yaw:
            # We didn't get a result, use the joystick
            if RobotBase.isSimulation:
                yaw = -self._controller.getRawAxis(constants.CONTROLLER_TURN_SIM)
            else:
                yaw = -self._controller.getRightX()
        else:
            yaw = self._calculate_yaw(yaw)
            print ("Yaw: ", yaw)

        SmartDashboard.putNumber("Yaw", yaw)
        self._dt.drive_teleop(forward, yaw)

    def isFinished(self) -> bool:
        # Should only run while button is held, return False
        return False

    def _calculate_yaw(self, yaw: float) -> float:
        yaw = -yaw * VISION_KP

        if yaw < 0:
            yaw = yaw - FEEDFORWARD
        elif yaw > 0:
            yaw += FEEDFORWARD

        return yaw
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
class DriveToTagWithVision(Command):
    
    # Goal of this command:
    # If the robot sees the desired AprilTag, the robot will turn toward the tag and drive slowly at it.
    # A PID controller is used to stear the robot.
    # The robot will stop at a specific distance from the tag.

    # If the robot does not see the tag, the robot will rotate clockwise in place until the tag is seen.

    # The robot will stop after 30 seconds.

    #  SEE IF I CAN USE SIMILAR COMMANDS as follows:  (Review absolute heading vs relative heading)
    #  ALSO SEE IF THIS ONLY CONTROLS HEADING AND SETS FORWARD SPEED TO ZERO.
        # wpilib.SmartDashboard.putData(
        #     "Turn-90",
        #     self._drivetrain.configure_turn_pid(-90).andThen(
        #         self._drivetrain.turn_with_pid().withName("TurnTo -90"),
        #     ),
        # )
        ##  NO_GO  "turn_with_pid" uses the current gyro heading of  the robot
        #
        #  PID controller:   _tag_pid_controller
    
    
    def __init__(self, dt: DriveTrain, _yaw_getter: Callable[[], float], tag_id):  
        self._dt = dt
        self._yaw_getter = _yaw_getter
        SmartDashboard.putNumber("VisionKP", 0.012)
        SmartDashboard.putNumber("VisionFF", 0.1)
        forward = 0
        yaw = 0
        self.addRequirements(self._dt)

    def initialize(self):
        self._dt.config_AprilTag_turn_command(0)   # we want the target offset to be zero
        forward = 0.1  # What is the minimal forward speed

    def execute(self):
        forward = -0.1
        heading_movement_to_correct_yaw_error = 0
        yaw: float = self._yaw_getter()
        print ("VISION YAW: ", yaw)
        if 1000 == yaw:
            # We didn't get a result, slowly turn the robot
            yaw = 0.1
        else:
            yaw = 57.3 * self._calculate_yaw(yaw)     # performed in Vision subsystem (in Radians)
            print ("   INPUT TO CALCULATION: ", yaw)
            heading_movement_to_correct_yaw_error = 0 - self._dt.calculate_AprilTag_turn_error(yaw)   # Calculate turn speed 
            
       # vision target to right of center + yaw
       # Driving + values turn robot to the left

        print ("Forward: ", forward, "   Vision Yaw: ", yaw, "   driving yaw", heading_movement_to_correct_yaw_error)

        SmartDashboard.putNumber("Yaw", yaw)
        self._dt.drive_teleop(forward, heading_movement_to_correct_yaw_error)   # + is to the right

    def isFinished(self) -> bool:
        # Should only run while button is held, return False
        heading_angle_returned_value = self._dt.AprilTag_at_turn_setpoint()
        print ("Have we reached the setpoint: ", heading_angle_returned_value)
        return heading_angle_returned_value

    def _calculate_yaw(self, yaw: float) -> float:
        yaw = -yaw * VISION_KP

        if yaw < 0:
            yaw = yaw - FEEDFORWARD
        elif yaw > 0:
            yaw += FEEDFORWARD

        return yaw

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

class TurnToAnglePID(PIDCommand):
    def __init__(self, dt: DriveTrain, angle: float, timeout=2):
        super().__init__(
            dt._turn_pid_controller,
            dt.getHeading,
            angle,
            lambda output: dt.drive_pid_turn(output),
            dt,
        )

    def isFinished(self) -> bool:
        # End when the controller is at the reference.
        return self.getController().atSetpoint()


# #-----

# class TurnToAngle(PIDCommand):

#     def __init__(self, targetAngleDegrees: float, drive: DriveTrain) -> None:
        
#         super().__init__(
#             wpimath.controller.PIDController(
#                 constants.DriveConstants.kTurnP,
#                 constants.DriveConstants.kTurnI,
#                 constants.DriveConstants.kTurnD,
#             ),
            
#             drive.getHeading,
            
#             targetAngleDegrees,
            
#             lambda output: drive.driveManually(0, output),
            
#             [drive],
#         )

        
#         self.getController().enableContinuousInput(-180, 180)

  
#         self.getController().setTolerance(
#             constants.DriveConstants.kTurnToleranceDeg,
#             constants.DriveConstants.kTurnRateToleranceDegPerS,
#         )

#     def isFinished(self) -> bool:
        
#         return self.getController().atSetpoint()
