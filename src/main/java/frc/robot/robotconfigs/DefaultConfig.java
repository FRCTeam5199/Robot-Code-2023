package frc.robot.robotconfigs;

import edu.wpi.first.wpilibj.*;
import frc.drive.auton.AutonType;
import frc.misc.PID;
import frc.motors.AbstractMotorController.SupportedMotors;
import frc.piecemanipulation.ManipulationManager.ManipulationControlStyles;
import frc.sensors.camera.IVision;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import static frc.drive.AbstractDriveManager.DriveBases;
import static frc.drive.AbstractDriveManager.DriveControlStyles;
import static frc.misc.PID.EMPTY_PID;
import static frc.telemetry.imu.AbstractIMU.SupportedIMU;

/**
 * Literally dont mind me I am simply vibing I am here because it means you only
 * have to change one value to completely
 * change robot settings (Otherwise, you would have to make 5 changes instead of
 * 1)
 *
 * @author jojo2357
 */
public abstract class DefaultConfig {
    public static final String BOTKEY = loadEnvVariable("bottoken");
    public static final String SLACKBOTKEY = loadEnvVariable("slackbottoken");
    public static final String SLACKSOCKETKEY = loadEnvVariable("slacksockettoken");
    public boolean DEBUG = false;
    public String AUTON_COMPLETE_NOISE = "";
    public boolean autonComplete = false;
    public boolean BRANDONISNOTHERE = false;
    // Subsystems
    public boolean ENABLE_APRILTAG = false;
    public boolean ENABLE_DRIVE = false;
    public boolean ENABLE_PNOOMATICS = false;
    public boolean ENABLE_DRIVE_BALL_TRACKING = false;
    public boolean ENABLE_SHOOTER = false;
    public boolean ENABLE_MUSIC = false;
    public boolean ENABLE_PDP = false;
    public boolean DRIVE_INVERT_LEFT = true;
    public boolean DRIVE_INVERT_RIGHT = false;
    public boolean ENABLE_OVERHEAT_DETECTION = false;
    public boolean ENABLE_CAMERA = false;
    public boolean ENABLE_TOGGLEABLE_RING = false;
    public boolean ENABLE_COLOR_SENSOR = false;

    // Misc
    public boolean ENABLE_VISION = false;
    public boolean ENABLE_IMU = false;
    public PowerDistribution.ModuleType POWER_DISTRIBUTION_MODULE_TYPE = PowerDistribution.ModuleType.kCTRE;
    public PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
    public int PCM_ID = 1;
    public int INTAKE_IN_ID;
    public int INTAKE_OUT_ID;
    public int SPIKE_IN_ID;
    public int SPIKE_OUT_ID;
    public int CLAW_IN_ID;
    public int CLAW_OUT_ID;
    public PID limeLightPid = EMPTY_PID;
    public PID leveling = EMPTY_PID;

    // Camera Settings
    public boolean ONE_CAMERA = true;
    public boolean FOUR_CAMERA = false;

    // SHOOTER
    public IVision.SupportedVision GOAL_CAMERA_TYPE = IVision.SupportedVision.PHOTON;
    public IVision.SupportedVision BALL_CAMERA_TYPE = IVision.SupportedVision.LIMELIGHT;

    // UI Styles
    public DriveControlStyles DRIVE_STYLE = DriveControlStyles.STANDARD_2023;
    public ManipulationControlStyles MANIPULATION_STYLE = ManipulationControlStyles.STANDARD_2023;

    // Motor Types
    public SupportedMotors DRIVE_MOTOR_TYPE = SupportedMotors.TALON_FX;
    public String DRIVE_MOTOR_CANBUS = "rio";

    // Intake
    public boolean CLAW_MANUAL = true;
    public boolean ENABLE_CLAW = true;
    // public boolean ENABLE_SPIKE = false;
    // public boolean ENABLE_PIECE_MANAGER = false;

    // Intake
    public SupportedMotors INTAKE_MOTOR_TYPE = SupportedMotors.CAN_SPARK_MAX;
    public SupportedMotors INTAKE_MOTOR_BOTTOM_TYPE = SupportedMotors.VICTOR;
    public int INTAKE_MOTOR_LEFT_ID = 0;
    public int INTAKE_MOTOR_BOTTOM_ID = 0;
    public int INTAKE_MOTOR_RIGHT_ID = 0;
    public String INTAKE_MOTOR_CANBUS = "rio";
    public boolean INTAKE_MANUAL = true;
    public boolean ENABLE_INTAKE = false;
    public boolean ENABLE_SPIKE = false;
    public boolean ENABLE_COMPOSITE_MANAGER = false;
    public boolean ENABLE_PIECE_MANAGER = false;

    public SupportedMotors WRIST_MOTOR_TYPE = SupportedMotors.CAN_SPARK_MAX;
    public int WRIST_MOTOR_ID = 0;
    public String WRIST_MOTOR_CANBUS = "rio";
    public boolean WRIST_MANUAL = true;
    public PID WRISTPID = EMPTY_PID;

    // Elevator
    public SupportedMotors ELEVATOR_MOTOR_TYPE = SupportedMotors.TALON_FX;
    public int ELEVATOR_MOTOR_ID = 0;
    public double ELEVATOR_GEARING = 1.0 / 9;
    public double ELEVATOR_SPROCKET_DIAMETER = 2D;
    public String ELEVATOR_MOTOR_CANBUS = "rio";
    public boolean ELEVATOR_MANUAL = true;
    public boolean ENABLE_ELEVATOR = true;
    public PID ELEVATORPID = EMPTY_PID;

    public SupportedMotors ARM_MOTOR_TYPE = SupportedMotors.CAN_SPARK_MAX;
    public int ARM_ROTATE_MOTOR_ID = 0;
    public int ARM_EXTEND_MOTOR_ID = 0;
    public double ARM_GEARING = 1.0 / 9;
    public double ARM_SPROCKET_DIAMETER = 2D;
    public String ARM_MOTOR_CANBUS = "rio";
    public boolean ARM_ELEVATOR_MANUAL = true;
    public boolean ARM_MANUAL = true;
    public boolean ENABLE_ARM = true;
    public boolean ENABLE_WRIST = true;
    public PID ARM_ROTATE_PID = EMPTY_PID;
    public PID ARM_EXTEND_PID = EMPTY_PID;
    public boolean ARM_EXTEND;

    public SupportedIMU IMU_TYPE = SupportedIMU.PIGEON;
    public AutonType AUTON_TYPE = AutonType.POINT_TO_POINT;
    public DriveBases DRIVE_BASE = DriveBases.STANDARD;
    // public ModuleConfiguration SWERVE_SDS_DRIVE_BASE;

    public double DRIVEBASE_DISTANCE_BETWEEN_WHEELS = -2; // Distance in meters between wheels
    public double DRIVEBASE_VOLTAGE_MULTIPLIER = 1;
    public double MAX_SPEED = 0; // max speed in fps - REAL IS 10(for 4in wheels)
    public double RUMBLE_TOLERANCE_FPS = 0; // The minimum value in which the controller will begin rumbling
    public double MAX_ROTATION = 0; // max rotational speed in radians per second - REAL IS 11.2(for 4in wheels)
    public double WHEEL_DIAMETER = 6;
    public double TURN_SCALE = 1;
    public double DRIVE_SCALE = 1;
    public double DRIVE_GEARING = 10 / 70.0;
    public int OVERHEAT_THRESHOLD = 80;

    public PID DRIVEBASE_PID = EMPTY_PID;
    public PID SHOOTER_PID = EMPTY_PID;
    public PID HEADING_PID = EMPTY_PID;
    public PID AUTO_XPID = EMPTY_PID;
    public PID AUTO_YPID = EMPTY_PID;
    public PID TELE_XYPID = EMPTY_PID;
    public PID TELEOP_AIMING_PID = EMPTY_PID;
    public PID AUTON_AIMING_PID = EMPTY_PID;
    public PID BACKSPIN_PID = EMPTY_PID;
    public double CTRE_SENSOR_UNITS_PER_ROTATION = 2048;

    public double XBOX_CONTROLLER_DEADZONE = 0.07;
    public double AUTON_TOLERANCE = 0.2;
    public double AUTO_SPEED = 3;
    public double AUTO_ROTATION_SPEED = 1;
    public String GOAL_CAM_NAME = "HD_USB_Camera";
    public String BALL_CAM_NAME = "BallCamera";

    // Drive Motors
    public int DRIVE_LEADER_L_ID; // talon
    public int[] DRIVE_FOLLOWERS_L_IDS; // talon
    public int DRIVE_LEADER_R_ID; // talon
    public int[] DRIVE_FOLLOWERS_R_IDS; // talon
    // Swerve Drive Motors
    public int SWERVE_DRIVE_FR, SWERVE_TURN_FR;
    public int SWERVE_DRIVE_FL, SWERVE_TURN_FL;
    public int SWERVE_DRIVE_BR, SWERVE_TURN_BR;
    public int SWERVE_DRIVE_BL, SWERVE_TURN_BL;
    // Swerve Drive CANCoders
    public int FRcoderID;
    public int FLcoderID;
    public int BRcoderID;
    public int BLcoderID;
    public boolean INVERT_DRIVE_DIRECTION;
    // Swerve Drive offsets
    public double FROFFSET;
    public double FLOFFSET;
    public double BROFFSET;
    public double BLOFFSET;

    public int IMU_ID = 22; // pigeon
    // leds
    public boolean ENABLE_LEDS = true;
    public int LED_STRAND_LENGTH = 34;
    public int LED_STRAND_PORT_ID = 0;
    // pdp
    public int PDP_ID = 0;

    public int XBOX_CONTROLLER_USB_SLOT = 0;
    public int XBOX_CONTROLLER_USB_SLOT_2 = 1;
    public int BUTTON_PANEL_USB_SLOT2 = 2;
    public int BUTTON_PANEL_USB_SLOT3 = 3;
    public int MIDI_CONTROLLER_TOP_ID = 3;
    public int MIDI_CONTROLLER_BOT_ID = 4;

    // Limelight Distance Tracking
    public double CAMERA_HEIGHT = 0; // Inches
    public double CAMERA_ANGLE = 0; // Radians
    public double TARGET_HEIGHT = 0; // Meters

    /**
     * Must be one of the following: {@link I2C.Port} {@link SerialPort.Port}
     * {@link SPI.Port}
     */
    public Object IMU_NAVX_PORT = I2C.Port.kMXP;
    public boolean PERMIT_ROUGE_INPUT_MAPPING = false;

    private static String loadEnvVariable(String filename) {
        try {
            Scanner fis = new Scanner(new File(filename + ".env"));
            return fis.nextLine();
        } catch (IOException e) {
            System.err.println("Could not load " + new File(filename + ".env"));
            return "";
        }
    }

    /**
     * the row is the position of the field -1
     * column one is TargetXHigh
     * column two is TargetXMid
     * column three is TargetXLow
     * column four is TargetY
     * column five is TargetRotation
     * column six is Speed
     */
    public double[][] red2DScoringArray = new double[][] {
            { -7.5, -7.5, -6.9, 16.8, 0, 0.5 },
            { -7.5, -7.5, -6.9, 14.3, 0, 0.5 },
            { -7.5, -7.5, -6.9, 12.3, 0, 0.5 },
            { -7.5, -7.5, -6.9, 10.8, 0, 0.5 },
            { -7.5, -7.5, -6.9, 9.0, 0, 0.5 },
            { -7.5, -7.5, -6.9, 7, 0, 0.5 },
            { -7.5, -7.5, -6.9, 5.55, 0, 0.5 },
            { -7.5, -7.5, -6.9, 3.52, 0, 0.5 },
            { -7.5, -7.5, -6.9, 1.52, 0, 0.5 }
    };

    /**
     * the row is the position of the field -1
     * column one is TargetX
     * column two is TargetY
     * column three is TargetRotation
     * column four is Speed
     * row one is LeftHumanPLayer
     * row two is RightHumanPLayer
     * row three is Spike
     */

    public double[][] red2DPickUpArray = new double[][] {
            { -50, 20, 0, .5 },
            { -49.75, 25, 0, .5 },
            { 0, 0, 0, 0 }
    };
    /**
     * the row is the position of the field -1
     * column one is TargetX
     * column two is TargetY
     * column three is TargetRotation
     * column four is Speed
     */
    public double[][] blue2DScoringArray = new double[][] {
            { -7.5, -7.5, -6.9, 16.8, 180, 0.5 },
            { -7.5, -7.5, -6.9, 14.475, 180, 0.5 },
            { -7.5, -7.5, -6.9, 12.5, 180, 0.5 },
            { -7.5, -7.5, -6.9, 11, 180, 0.5 },
            { -7.5, -7.5, -6.9, 9.0, 180, 0.5 },
            { -7.5, -7.5, -6.9, 7.05, 180, 0.5 },
            { -7.5, -7.5, -6.9, 5, 180, 0.5 },
            { -7.5, -7.5, -6.9, 3.6, 180, 0.5 },
            { -7.5, -7.5, -6.9, 1.5, 180, 0.5 }
    };

    /**
     * the row is the position of the field -1
     * column one is TargetX
     * column two is TargetY
     * column three is TargetRotation
     * column four is Speed
     * row one is LeftHumanPLayer
     * row two is RightHumanPLayer
     * row three is Spike
     */

    public double[][] blue2DPickUpArray = new double[][] {
            { -54 + 49.75, 20, Math.PI, .5 },
            { -54 + 49.75, 25, Math.PI, .5 },
            { 0, 0, 0, 0 }
    };

    /**
     *
     * Prints the enabled toggles for the loaded settings
     */
    public void printToggles() {
        System.out.println("-------------------<RobotSettings>-----------------");
        System.out.println("          Driving " + ENABLE_DRIVE);
        System.out.println("         Shooting " + ENABLE_SHOOTER);
        System.out.println("           Vision " + ENABLE_VISION);
        System.out.println("              IMU " + ENABLE_IMU);
        System.out.println("              IMU " + IMU_TYPE.name());
        System.out.println("     Drive motors " + DRIVE_MOTOR_TYPE.name());
        System.out.println("      Drive style " + DRIVE_STYLE.name());
        System.out.println("   Drivebase Type " + DRIVE_BASE.name());
        System.out.println("  Auton Completed " + autonComplete);
        System.out.println("-------------------</RobotSettings>-----------------");
    }

    /**
     * Prints out "Numbers" which pertain to constants regarding the robot such as
     * gearings, wheel sizes, etc. Not to be
     * confused with {@link #printMappings()} which prints numbers associated witd
     * ID's and software. this is hardware
     */
    public void printNumbers() {
        System.out.println("-------------------<RobotSettings>-----------------");
        System.out.println("Drive PIDF " + DRIVEBASE_PID);
        System.out.println("Max drive speed/rotation " + MAX_SPEED + "/" + MAX_ROTATION);
        System.out.println("Turn + drive scale " + TURN_SCALE + "/" + DRIVE_SCALE);
        // System.out.println("");
        System.out.println("-------------------</RobotSettings>----------------");
    }

    /**
     * Prints out all of the id's for anything that needs an id
     */
    public void printMappings() {
        System.out.println("-------------------<RobotSettingspings>-----------------");
        System.out.println("                    Goal cam name: " + GOAL_CAM_NAME);
        System.out.println("                    Ball cam name: " + BALL_CAM_NAME);
        // System.out.println(" Drive leader left id (followers): " + DRIVE_LEADER_L_ID
        // + " (" + DRIVE_FOLLOWERS_L_IDS[0] + (DRIVE_FOLLOWERS_L_IDS.length > 1 ? ", "
        // + DRIVE_FOLLOWERS_L_IDS[1] : "") + ")");
        // System.out.println("Drive leader right id (followers): " + DRIVE_LEADER_R_ID
        // + " (" + DRIVE_FOLLOWERS_R_IDS[0] + (DRIVE_FOLLOWERS_R_IDS.length > 1 ? ", "
        // + DRIVE_FOLLOWERS_R_IDS[1] : "") + ")");
        System.out.println("                           IMU id: " + IMU_ID);
        System.out.println("-------------------</RobotSettingspings>-----------------");
    }
}