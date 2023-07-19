package frc.robot.robotconfigs;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AutonType;
import frc.misc.PID;
import frc.motors.AbstractMotorController;
import frc.piecemanipulation.ManipulationManager;
import frc.sensors.camera.IVision;
import frc.telemetry.imu.AbstractIMU;

public class SwerveComp2023 extends DefaultConfig{

    public SwerveComp2023(){
        ENABLE_SHOOTER = false;
        ENABLE_DRIVE = true;
        ENABLE_MUSIC = false;
        ENABLE_PDP = true;
        ENABLE_APRILTAG = true;
        ENABLE_PIECE_MANAGER = true;
        ENABLE_PNOOMATICS = true;
        ENABLE_LEDS = true;
        ENABLE_SPIKE = true;

        DRIVE_INVERT_LEFT = false;
        DRIVE_INVERT_RIGHT = false;
        MANIPULATION_STYLE = ManipulationManager.ManipulationControlStyles.STANDARD_2023;

        //Wrist
        WRIST_MOTOR_TYPE =  AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
        WRIST_MOTOR_ID = 34;
        WRIST_MOTOR_CANBUS = "rio";
        WRIST_MANUAL = true;
        ENABLE_WRIST = true;
        WRISTPID = new PID(0.08 , 0.0001, 0.045);

        //Misc
        ENABLE_VISION = true;
        ENABLE_IMU = true;
        IMU_NAVX_PORT = I2C.Port.kMXP;
        IMU_ID = 22; //pigeon
        limeLightPid = new PID(0.1, 0.0, 0);
        leveling = new PID(0.0175,0.0,0.01);

        //Elevator
        ENABLE_ELEVATOR = true;
        ELEVATOR_GEARING = 1/9.0;
        ELEVATOR_MOTOR_CANBUS = "rio";
        ELEVATOR_MOTOR_ID = 40;
        ELEVATOR_MOTOR_TYPE = AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
        ELEVATOR_SPROCKET_DIAMETER = 2D;
        ELEVATORPID =  new PID(.3, 0.0, 0.0);
        ARM_ELEVATOR_MANUAL = false;

        //ARM
        ARM_MOTOR_TYPE = AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
        ARM_MOTOR_ID = 39;
        ARM_GEARING = (1/15D) * (28/52D) * (15/61D);
        ARM_SPROCKET_DIAMETER = 1;
        ARM_MOTOR_CANBUS = "rio";
        ENABLE_ARM = true;
        ARM_PID = new PID(.5, 0.0, 0);

        //INTAKE
        INTAKE_MOTOR_TYPE =  AbstractMotorController.SupportedMotors.CAN_SPARK_MAX;
        INTAKE_MOTOR_BOTTOM_TYPE = AbstractMotorController.SupportedMotors.VICTOR;
        INTAKE_MOTOR_BOTTOM_ID = 54;
        INTAKE_MOTOR_LEFT_ID = 44;
        INTAKE_MOTOR_RIGHT_ID = 27;
        INTAKE_MOTOR_CANBUS = "rio";
        INTAKE_MANUAL = true;
        ENABLE_INTAKE = true;

        PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        PCM_ID = 50;
        INTAKE_IN_ID = 10;
        INTAKE_OUT_ID = 9;
        SPIKE_IN_ID = 8;
        SPIKE_OUT_ID = 11;


        // 61:15 52:28 15:1
        //UI Styles
        DRIVE_STYLE = AbstractDriveManager.DriveControlStyles.STANDARD_2023;
        DRIVE_MOTOR_TYPE = AbstractMotorController.SupportedMotors.TALON_FX;
        IMU_TYPE = AbstractIMU.SupportedIMU.PIGEON;
        DRIVE_BASE = AbstractDriveManager.DriveBases.SWIVEL;
        ENABLE_CAMERA = true;
        ENABLE_VISION = true;


        AUTON_TYPE = AutonType.POINT_TO_POINT;

        DRIVEBASE_PID = new PID(0.0000001, 0, 0.0001);
        HEADING_PID = new PID(.8, 0.0, 0.0);
        AUTO_XPID = new PID(1.15, 0.11, 0.001);
        AUTO_YPID = new PID(1.3, 0.11, 0.001);
        TELE_XYPID = new PID(2.7, 0.1, 0.0);
        DRIVEBASE_DISTANCE_BETWEEN_WHEELS = 0.435991;
        MAX_SPEED = 16; //max speed in fps - REAL IS 10(for 4in wheels)
        RUMBLE_TOLERANCE_FPS = 8;
        MAX_ROTATION = 11.2; //max rotational speed// in radians per second - REAL IS 11.2(for 4in wheels)
        WHEEL_DIAMETER = 4; //update: now it's used once
        TURN_SCALE = 0.7;
        DRIVE_SCALE = 1;
        DRIVE_GEARING = 1/6.12;
        ENABLE_COLOR_SENSOR = true;

        CTRE_SENSOR_UNITS_PER_ROTATION = 2048;
        CAMERA_HEIGHT = 0; //Inches
        CAMERA_ANGLE = 0; //Radians
        TARGET_HEIGHT = 0;//2.44; //Meters

        XBOX_CONTROLLER_DEADZONE = 0.07;
        AUTON_TOLERANCE = 0.35;
        AUTO_SPEED = 3;
        AUTO_ROTATION_SPEED = 1;
        XBOX_CONTROLLER_USB_SLOT = 0;

        //PowerDistribution
        ENABLE_PDP = true;
        PDP_ID = 50;
        POWER_DISTRIBUTION_MODULE_TYPE = PowerDistribution.ModuleType.kRev;


        //Drive Motors
        SWERVE_DRIVE_FL = 1;
        SWERVE_TURN_FL = 2;
        SWERVE_DRIVE_FR = 3;
        SWERVE_TURN_FR = 4;
        SWERVE_DRIVE_BR = 5;
        SWERVE_TURN_BR = 6;
        SWERVE_DRIVE_BL = 7;
        SWERVE_TURN_BL = 8;
        DRIVE_MOTOR_CANBUS = "Canivore1";


        // Camera Settings
        ONE_CAMERA = true;
        FOUR_CAMERA = false;

        // limelight
        ENABLE_VISION = true;
        ENABLE_CAMERA = true;
        GOAL_CAM_NAME = "limelight";
        BALL_CAM_NAME = "Camera";
        GOAL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;
        BALL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;
    }

}
