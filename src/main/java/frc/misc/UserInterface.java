package frc.misc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.pointtopoint.AutonRoutines;
import frc.motors.AbstractMotorController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static frc.robot.Robot.robotSettings;

public class UserInterface {
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    public static NetworkTable autoDataTable = instance.getTable("autodata");
    public static NetworkTableEntry autoPath = autoDataTable.getEntry("autoPath");

    private static NetworkTable position = autoDataTable.getSubTable("position");
    public static NetworkTableEntry xPos = position.getEntry("x");
    public static NetworkTableEntry yPos = position.getEntry("y");
    public static NetworkTableEntry enabled = autoDataTable.getEntry("enabled");

    //TABS
    public static final ShuffleboardTab DRIVE_TAB = Shuffleboard.getTab("drive"),
    //PDP_TAB = Shuffleboard.getTab("Lectricity"),
    MUSICK_TAB = Shuffleboard.getTab("musick"),
            ROBOT_TAB = Shuffleboard.getTab("DANGER!"),
            WARNINGS_TAB = Shuffleboard.getTab("Warnings"),
            AUTON_TAB = Shuffleboard.getTab("Auton"),
            CAMERA_TAB = Shuffleboard.getTab("Camera");

    //LAYOUTS
    public static final ShuffleboardLayout DRIVE_PID_LAYOUT = DRIVE_TAB.getLayout("PID", BuiltInLayouts.kList).withProperties(Map.of("Label position", "LEFT")).withSize(2, 3),
            PDP_SETTINGS_LAYOUT = ROBOT_TAB.getLayout("PowerDistribution", BuiltInLayouts.kList).withProperties(Map.of("Label position", "LEFT")).withSize(2, 1);

    //DRIVETRAIN
    public static final SimpleWidget DRIVE_ROT_MULT = DRIVE_TAB.add("Rotation Factor", robotSettings.TURN_SCALE),
            DRIVE_SCALE_MULT = DRIVE_TAB.add("Speed Factor", robotSettings.DRIVE_SCALE),
            DRIVE_P = DRIVE_PID_LAYOUT.add("P", robotSettings.DRIVEBASE_PID.getP()),
            DRIVE_I = DRIVE_PID_LAYOUT.add("I", robotSettings.DRIVEBASE_PID.getI()),
            DRIVE_D = DRIVE_PID_LAYOUT.add("D", robotSettings.DRIVEBASE_PID.getD()),
            DRIVE_F = DRIVE_PID_LAYOUT.add("F", robotSettings.DRIVEBASE_PID.getF()),
            DRIVE_CALIBRATE_PID = DRIVE_PID_LAYOUT.add("Tune PID", false).withWidget(BuiltInWidgets.kToggleSwitch),
            DRIVE_COAST = DRIVE_TAB.add("Coast", false).withWidget(BuiltInWidgets.kToggleSwitch),
            DRIVE_RUMBLE_NEAR_MAX = DRIVE_TAB.add("Rumble Near Max", false).withWidget(BuiltInWidgets.kToggleSwitch),
            DRIVE_SPEED_RPM = DRIVE_TAB.add("Drivebase RPM", 0).withWidget(BuiltInWidgets.kGraph),
            ROBOT_LOCATION = DRIVE_TAB.add("Robot Location", "(0, 0)").withWidget(BuiltInWidgets.kTextView),

    //MUSICK
    MUSIC_DISABLE_SONG_TAB = MUSICK_TAB.add("Stop Song", false).withWidget(BuiltInWidgets.kToggleButton),
            MUSIC_FOUND_SONG = MUSICK_TAB.add("Found it", false),
            DELETE_DEPLOY_DIRECTORY = ROBOT_TAB.add("DELETE DEPLOY DIRECTORY", ""),
            PRINT_ROBOT_TOGGLES = ROBOT_TAB.add("Reprint robot toggles", false).withWidget(BuiltInWidgets.kToggleButton),
            PRINT_ROBOT_MAPPINGS = ROBOT_TAB.add("Reprint robot mappings", false).withWidget(BuiltInWidgets.kToggleButton),
            PRINT_ROBOT_NUMBERS = ROBOT_TAB.add("Reprint robot numbers", false).withWidget(BuiltInWidgets.kToggleButton),
            DRIVE_SPEED = DRIVE_TAB.add("Drivebase Speed", 0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Min", 0, "Max", 20)),
    //PowerDistribution
    PDP_BROWNOUT_MIN_OVERRIDE = PDP_SETTINGS_LAYOUT.add("Settings Override", false).withWidget(BuiltInWidgets.kToggleSwitch),
            PDP_BROWNOUT_MIN_VAL = PDP_SETTINGS_LAYOUT.add("Minimum Brownout Voltage", 9),
    //DANGER PANEL
    GET_RANDOM_FIX = ROBOT_TAB.add("Get random fix", false).withWidget(BuiltInWidgets.kToggleButton);
    public static final HashMap<AbstractMotorController, SimpleWidget> motorTemperatureMonitors = new HashMap<>();

    //STATIC STUFF
    public static SimpleWidget SHOOTER_RPM;
    public static ComplexWidget MUSIC_SELECTOR, PDP_DISPLAY,
            AUTON_STYLE_CHOOSER = AUTON_TAB.add("Auton Styles", AutonRoutines.getSendableChooser()).withWidget(BuiltInWidgets.kComboBoxChooser),
            DRIVE_STYLE_CHOOSER = DRIVE_TAB.add("Drive Styles", AbstractDriveManager.DriveControlStyles.getSendableChooser()).withWidget(BuiltInWidgets.kComboBoxChooser);


    //SmartDashboard
    public static void smartDashboardPutNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void smartDashboardPutBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public static void smartDashboardPutString(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    //MISC
    public static void initRobot() {

        Object m_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

        // Create and push Field2d to SmartDashboard.
        Field2d m_field = new Field2d();
        SmartDashboard.putData(m_field);

        // Push the trajectory to Field2d.
        m_field.getObject("traj").setTrajectory((Trajectory) m_trajectory);
        if (robotSettings.ENABLE_MUSIC) {
            MUSIC_SELECTOR = MUSICK_TAB.add("SongSelector", Chirp.MUSIC_SELECTION).withWidget(BuiltInWidgets.kComboBoxChooser);
        }

        if (robotSettings.ENABLE_CAMERA) {
            UsbCamera camera = CameraServer.startAutomaticCapture(0);
            camera.setResolution(640, 480);
            //UserInterface.SMART_DASHBOARD.add("CameraViewer", camera);
        }
    }
}