package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.drive.DriveManagerSwerve;
import frc.drive.auton.AbstractAutonManager;
import frc.drive.auton.pointtopoint.AutonManager;
import frc.drive.auton.pointtopoint.AutonRoutines;
import frc.misc.Chirp;
import frc.misc.ISubsystem;
import frc.misc.LEDs;
import frc.misc.Pneumatics;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.pdp.PDP;
import frc.piecemanipulation.Arm;
import frc.piecemanipulation.Elevator;
import frc.piecemanipulation.Intake;
import frc.piecemanipulation.ManipulationManager;
import frc.piecemanipulation.Wrist;
import frc.robot.robotconfigs.DefaultConfig;
import frc.robot.robotconfigs.Swerve2022;
import frc.robot.robotconfigs.SwerveComp2023;
import frc.robot.robotconfigs.SwervePrac2023;
import frc.selfdiagnostics.ISimpleIssue;
import frc.sensors.camera.IVision;

import java.io.File;
import java.util.ArrayList;
import java.util.Map;
import java.util.Random;

/**
 * Welcome. Please enjoy your stay here in programmer fun time land. And remember, IntelliJ is king
 */

// Hello!
public class Robot extends TimedRobot {
    /**
     * No son, I refuse to make a new, unseeded random everytime we want a new song. Besides, we have a random at home
     * already so you don't need another one
     */
    public static final Random RANDOM = new Random(System.currentTimeMillis());
    /**
     * If you change this ONE SINGULAR VARIABLE the ENTIRE CONFIG WILL CHANGE. Use this to select which robot you are
     * using from the list under robotconfigs
     */
    public static final ArrayList<ISubsystem> subsystems = new ArrayList<>();
    private static final String DELETE_PASSWORD = "programmer funtime lanD";
    public static DefaultConfig robotSettings;
    public static AbstractDriveManager driver;
    // true = cone, false = cube
    public static Chirp chirp;
    public static PDP pdp;
    public static Wrist wrist;
    public static LEDs leds;
    public static Intake intake;
    public static Elevator elevator;
    public static Pneumatics pneumatics;
    public static ManipulationManager manipulationManager;
    public static Arm arm;
    public static AbstractAutonManager autonManager;
    public static boolean SECOND_TRY;
    public static String lastFoundSong = "";
    private static long lastDisable = 0;

    /**
     * Init everything
     */
    @Override
    public void robotInit() throws IllegalStateException {
        getRestartProximity();
        getSettings();
        robotSettings.printMappings();
        robotSettings.printToggles();
        robotSettings.printNumbers();
        UserInterface.initRobot();
        if (robotSettings.ENABLE_DRIVE) {
            if (robotSettings.DRIVE_BASE == AbstractDriveManager.DriveBases.STANDARD)
                driver = new DriveManagerStandard();
            else if (robotSettings.DRIVE_BASE == AbstractDriveManager.DriveBases.SWIVEL)
                driver = new DriveManagerSwerve();
        }

        if (robotSettings.ENABLE_ELEVATOR)
            elevator = new Elevator();
        if (robotSettings.ENABLE_MUSIC) {
            chirp = new Chirp();
        }
        if(robotSettings.ENABLE_WRIST)
            wrist =  new Wrist();
        if(robotSettings.ENABLE_ARM)
            arm = new Arm();
        if(robotSettings.ENABLE_INTAKE)
            intake = new Intake();
        if(robotSettings.ENABLE_PIECE_MANAGER) {
            manipulationManager = new ManipulationManager();
        }
        if(robotSettings.ENABLE_PNOOMATICS)
            pneumatics =  new Pneumatics();
        if (robotSettings.ENABLE_DRIVE) {
            switch (robotSettings.AUTON_TYPE) {
                case POINT_TO_POINT:
                    autonManager = new AutonManager(AutonRoutines.DO_NOTHING_RED, driver);//Trajectories.TEST_PATH, driver);
                    System.out.println("autoRoutine waypoint");
                    break;
            }
        }
        if (robotSettings.ENABLE_PDP) {
            pdp = new PDP();
        }

        for (AbstractMotorController motor : AbstractMotorController.motorList) {
            if (motor.getMotorTemperature() > 5) {
                UserInterface.motorTemperatureMonitors.put(motor, UserInterface.WARNINGS_TAB.add(motor.getName(), motor.getMotorTemperature()).withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("Min", 30, "Max", 80)));
            }
        }
        if (robotSettings.ENABLE_VISION)
            IVision.manufactureGoalCamera(robotSettings.GOAL_CAMERA_TYPE).setLedMode(IVision.VisionLEDMode.OFF);

    }

    /**
     * Reads from the preferences what the last boot time is. Depending on current system time, sets the {@link
     * #SECOND_TRY} flag to either restart on error or to persist as best as possible. If you leave the robot on for
     * half a century then it might not work right so please refrain from that
     */
    private static void getRestartProximity() {
        long lastBoot = Long.parseLong(Preferences.getString("lastboot", "0"));
        long currentBoot = System.currentTimeMillis();
        Preferences.setString("lastboot", "0" + currentBoot);
        if (lastBoot > currentBoot) {
            SECOND_TRY = false;
        } else if (lastBoot > 1614461266977L) {
            SECOND_TRY = currentBoot - lastBoot < 30000;
        } else if (lastBoot < 1614461266977L && currentBoot < 1614461266977L) {
            SECOND_TRY = currentBoot - lastBoot < 30000;
        } else {
            SECOND_TRY = false;
        }
    }

    /**
     * Loads settings based on the id of the robot.
     *
     * @see DefaultConfig
     */
    private static void getSettings() {
        String hostName = Preferences.getString("hostname", "Default");
        System.out.println("I am " + hostName);
        switch (hostName) {
            case "2022-Swivel":
                robotSettings = new Swerve2022();
                break;
            case "2023-Prac":
                robotSettings = new SwervePrac2023();
                break;
            case "2023-Comp":
                robotSettings = new SwerveComp2023();
                break;
            default:
                //preferences.putString("hostname", "2021-Comp");
                //settingsFile = new CompetitionRobot2021();
                //break;
                robotSettings = new SwerveComp2023();
                //throw new IllegalStateException("You need to ID this robot.");
        }
    }

    @Override
    public void disabledInit() {
        for (ISubsystem system : subsystems) {
            system.initDisabled();
        }
        lastDisable = System.currentTimeMillis();
    }

    @Override
    public void autonomousInit() {
        for (ISubsystem system : subsystems) {
            system.initAuton();
        }
    }

    @Override
    public void teleopInit() {
        for (ISubsystem system : subsystems) {
            system.initTeleop();
        }
    }

    @Override
    public void testInit() {
        for (ISubsystem system : subsystems) {
            system.initTest();
        }
    }

    @Override
    public void robotPeriodic() {
        if (UserInterface.PRINT_ROBOT_TOGGLES.getEntry().getBoolean(false)) {
            robotSettings.printToggles();
            UserInterface.PRINT_ROBOT_TOGGLES.getEntry().setBoolean(false);
        }
        if (UserInterface.PRINT_ROBOT_MAPPINGS.getEntry().getBoolean(false)) {
            robotSettings.printMappings();
            UserInterface.PRINT_ROBOT_MAPPINGS.getEntry().setBoolean(false);
        }
        if (UserInterface.PRINT_ROBOT_NUMBERS.getEntry().getBoolean(false)) {
            robotSettings.printNumbers();
            UserInterface.PRINT_ROBOT_NUMBERS.getEntry().setBoolean(false);
        }
        if (UserInterface.MUSIC_DISABLE_SONG_TAB.getEntry().getBoolean(false)) {
            chirp.stop();
            UserInterface.MUSIC_DISABLE_SONG_TAB.getEntry().setBoolean(false);
        }
        if (UserInterface.DELETE_DEPLOY_DIRECTORY.getEntry().getString("").equals(DELETE_PASSWORD)) {
            UserInterface.DELETE_DEPLOY_DIRECTORY.getEntry().setString("Correct password");
            deleteFolder(Filesystem.getDeployDirectory());
            throw new RuntimeException("Deleted deploy dir contents");
        }

        UserInterface.smartDashboardPutString("alliance", DriverStation.getAlliance().toString());

        driver.updateGeneric();
    /*
        for (AbstractMotorController motor : AbstractMotorController.motorList) {
            if (motor.getMotorTemperature() > 5) {
                UserInterface.motorTemperatureMonitors.get(motor).getEntry().setDouble(motor.getMotorTemperature());
            }
        }*/

        ISimpleIssue.robotPeriodic();
    }

    @Override
    public void disabledPeriodic() {
        if (robotSettings.ENABLE_DRIVE && System.currentTimeMillis() > lastDisable + 5000)
            driver.setBrake(false);
    }

    @Override
    public void autonomousPeriodic() {
        for (ISubsystem system : subsystems) {
            system.updateAuton();
        }
    }

    @Override
    public void teleopPeriodic() {
        for (ISubsystem system : subsystems) {
            system.updateTeleop();
        }
    }

    @Override
    public void testPeriodic() {
        for (ISubsystem system : subsystems) {
            system.updateTest();
        }
    }

    /**
     * Uses {@link #deleteFolder(File) deadly recursion} in order to maybe delete ghost files
     *
     * @param parentFolder The deploy folder/subfolders within deploy folder
     */
    private void deleteFolder(File parentFolder) {
        for (File file : parentFolder.listFiles()) {
            if (file.isDirectory()) {
                deleteFolder(file);
            }
            file.delete();
            System.out.println("REMOVED FILE " + file.getName());
        }
    }
}