package frc.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.InitializationFailureException;
import frc.misc.PID;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.motors.followers.AbstractFollowerMotorController;
import frc.selfdiagnostics.MotorDisconnectedIssue;
import frc.sensors.camera.IVision;
import frc.telemetry.RobotTelemetryStandard;

import java.util.Objects;

import static frc.robot.Robot.robotSettings;

/**
 * Everything that has to do with driving is in here. There are a lot of auxilairy helpers and {@link
 * frc.robot.Robot#robotSettings} that feed in here.
 *
 * @see RobotTelemetryStandard
 */
public class DriveManagerStandard extends AbstractDriveManager {
    private static final boolean DEBUG = false;
    public final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(robotSettings.DRIVEBASE_DISTANCE_BETWEEN_WHEELS);
    private final GenericEntry
            P = UserInterface.DRIVE_P.getEntry(),
            I = UserInterface.DRIVE_I.getEntry(),
            D = UserInterface.DRIVE_D.getEntry(),
            F = UserInterface.DRIVE_F.getEntry(),
            calibratePid = UserInterface.DRIVE_CALIBRATE_PID.getEntry(),
            coast = UserInterface.DRIVE_COAST.getEntry(),
            rumbleController = UserInterface.DRIVE_RUMBLE_NEAR_MAX.getEntry(),
            driveRPM = UserInterface.DRIVE_SPEED_RPM.getEntry();
    public AbstractMotorController leaderL, leaderR;
    public AbstractFollowerMotorController followerL, followerR;
    public IVision visionCamera, ballCamera;
    private BaseController controller;
    private PID lastPID = PID.EMPTY_PID;
    private PIDController TELEOP_AIMING_PID;

    public DriveManagerStandard() throws UnsupportedOperationException, InitializationFailureException {
        super();
    }

    /**
     * Initializes the driver
     *
     * @throws UnsupportedOperationException  When a setting does not have a valid configuration defined
     * @throws InitializationFailureException When something fails to init properly
     */
    @Override
    public void init() throws UnsupportedOperationException, InitializationFailureException {
        createDriveMotors();
        initPID();
        initMisc();
        createTelem();
        if (robotSettings.ENABLE_VISION) {
            visionCamera = IVision.manufactureGoalCamera(robotSettings.GOAL_CAMERA_TYPE);
            if (robotSettings.ENABLE_DRIVE_BALL_TRACKING)
                ballCamera = IVision.manufactureBallCamera(robotSettings.BALL_CAMERA_TYPE);
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return !leaderL.isFailed() && !leaderR.isFailed() && !followerL.failureFlag() && !followerR.failureFlag() ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    /**
     * Put any experimental stuff to do with the drivetrain here
     */
    @Override
    public void updateTest() {
        driveRPM.setDouble(leaderL.getSpeed() + leaderR.getSpeed() / 2);
    }

    /**
     * This is where driving happens. Call this every tick to drive and set {@link frc.robot.robotconfigs.DefaultConfig#DRIVE_STYLE}
     * to change the drive stype
     *
     * @throws IllegalArgumentException if {@link frc.robot.robotconfigs.DefaultConfig#DRIVE_STYLE} is not implemented
     *                                  here or if you missed a break statement
     */
    @Override
    public void updateTeleop() throws IllegalArgumentException {
        updateGeneric();
        driveRPM.setDouble(leaderL.getSpeed() + leaderR.getSpeed() / 2);
        double avgSpeedInFPS = Math.abs((leaderL.getSpeed() + leaderR.getSpeed()) / 2);
        UserInterface.DRIVE_SPEED.getEntry().setDouble(avgSpeedInFPS);
        if (Objects.requireNonNull(robotSettings.DRIVE_STYLE) == DriveControlStyles.STANDARD_2023) {
            double invertedDrive = robotSettings.DRIVE_INVERT_LEFT ? -1 : 1;
            double dynamic_gear_R = controller.get(DefaultControllerEnums.XBoxButtons.RIGHT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN ? 0.25 : 1;
            double dynamic_gear_L = controller.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN ? 0.25 : 1;
            if (robotSettings.DEBUG && DEBUG) {
                // System.out.println("Forward: " + (invertedDrive * dynamic_gear_L * controller.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_Y)) + " Turn: " + (dynamic_gear_R * -controller.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_X)));
//                System.out.println("Forward: " + (invertedDrive * dynamic_gear_L * controller.get(XboxAxes.LEFT_JOY_Y)) + " Turn: " + (dynamic_gear_R * -controller.get(XboxAxes.RIGHT_JOY_X)));
            }
            if (rumbleController.getBoolean(false)) {
                controller.rumble(Math.max(0, Math.min(1, (avgSpeedInFPS - robotSettings.RUMBLE_TOLERANCE_FPS) / (robotSettings.MAX_SPEED - robotSettings.RUMBLE_TOLERANCE_FPS))));
            }
            driveCringe(invertedDrive * dynamic_gear_L * controller.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_Y), dynamic_gear_R * -controller.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_X));
        } else {
            throw new IllegalStateException("Invalid drive type");
        }

    }

    @Override
    public void updateAuton() {
        updateGeneric();
        driveRPM.setDouble(leaderL.getSpeed() + leaderR.getSpeed() / 2);
    }

    @Override
    public void initTest() {
        initGeneric();
        setBrake(false);
        resetDriveEncoders();
    }

    @Override
    public void initTeleop() {
        initGeneric();
    }

    @Override
    public void initAuton() {
        initGeneric();
        setBrake(false);
        resetDriveEncoders();
    }

    @Override
    public void initDisabled() {
        setBrake(true);
    }

    @Override
    public void initGeneric() {
        setBrake(true);
        if (robotSettings.ENABLE_VISION && robotSettings.ENABLE_DRIVE_BALL_TRACKING) {
            Alliance a = DriverStation.getAlliance();
            if (a == Alliance.Blue) {
                ballCamera.setPipeline(2);
            } else if (a == Alliance.Red) {
                ballCamera.setPipeline(1);
            } else {
                ballCamera.setPipeline(0);
                DriverStation.reportError("No alliance detected, big problem..", false);
            }
        }
    }

    @Override
    public void resetDriveEncoders() {
        leaderL.resetEncoder();
        leaderR.resetEncoder();
    }

    public void setBrake(boolean braking) {
        coast.setBoolean(!braking);
        leaderL.setBrake(braking);
        leaderR.setBrake(braking);
        followerL.setBrake(braking);
        followerR.setBrake(braking);
    }

    @Override
    public void driveMPS(double xMeters, double yMeters, double rotation) {
        drivePure(Units.metersToFeet(xMeters), rotation);
    }

    @Override
    public void driveWithChassisSpeeds(ChassisSpeeds speeds) { //speeds in mps
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveMPS(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    /**
     * updates telemetry and if calibrating pid, does that
     */
    @Override
    public void updateGeneric() {
        super.updateGeneric();
        if (robotSettings.ENABLE_IMU) {
            guidance.updateGeneric();
        }
        MotorDisconnectedIssue.handleIssue(this, leaderL, leaderR);
        MotorDisconnectedIssue.handleIssue(this, followerL, followerR);
        setBrake(!coast.getBoolean(false));
        if (calibratePid.getBoolean(false)) {
            PID readPid = new PID(P.getDouble(robotSettings.DRIVEBASE_PID.getP()), I.getDouble(robotSettings.DRIVEBASE_PID.getI()), D.getDouble(robotSettings.DRIVEBASE_PID.getD()), F.getDouble(robotSettings.DRIVEBASE_PID.getF()));
            if (!lastPID.equals(readPid)) {
                lastPID = readPid;
                setPID(lastPID);
                if (robotSettings.DEBUG && DEBUG) {
                    // System.out.println("Set drive pid to " + lastPID);
                }
            }
        }
    }

    @Override
    protected void onControlChange() {
        //recreate controllers
        initMisc();
    }

    @Override
    public void lockWheels() {

    }

    @Override
    public boolean leveling() {
        return true;
    }
    public boolean driveForwardWithAngle() {
        return true;
    }

    /**
     * Drives the bot based on the requested left and right speed
     *
     * @param leftFPS  Left drivetrain speed in feet per second
     * @param rightFPS Right drivetrain speed in feet per second
     */
    public void driveFPS(double leftFPS, double rightFPS) {
        //todo get rid of this
        if (robotSettings.DEBUG && DEBUG) {
            UserInterface.smartDashboardPutNumber("Left Wheel RPM", leaderL.getSpeed());
            UserInterface.smartDashboardPutNumber("Left Wheel Voltage", leaderL.getVoltage());
        }
        UserInterface.smartDashboardPutNumber("leftFPS feedy BOI", leftFPS);
        UserInterface.smartDashboardPutNumber("leftFPS magic number", leftFPS);
        leaderL.moveAtVelocity(leftFPS);
        leaderR.moveAtVelocity(rightFPS);
    }

    /**
     * drives the robot based on -1 / 1 inputs (ie 100% forward and 100% turning)
     *
     * @param forward  the percentage of max forward to do
     * @param rotation the percentage of max turn speed to do
     */
    public void drive(double forward, double rotation) {
        drivePure(adjustedDrive(forward * (robotSettings.INVERT_DRIVE_DIRECTION ? -1 : 0)), adjustedRotation(rotation));
    }

    public void drivePercent(double leftPercent, double rightPercent) {
        leaderL.moveAtPercent(leftPercent);
        leaderR.moveAtPercent(rightPercent);
    }

    /**
     * This takes a speed in feet per second, a requested turn speed in radians/sec
     *
     * @param FPS   Speed in Feet per Second
     * @param omega Rotation in Radians per Second
     */
    public void drivePure(double FPS, double omega) {
        driveWithChassisSpeeds(new ChassisSpeeds(Units.feetToMeters(FPS), 0, omega));
    }

    @Override
    public void drivePure(double forward, double sideways, double omega) {
        drivePure(forward, omega);
    }

    /**
     * Drives the bot based on the requested left and right speed
     *
     * @param leftMPS  Left drivetrain speed in meters per second
     * @param rightMPS Right drivetrain speed in meters per second
     */
    public void driveMPS(double leftMPS, double rightMPS) {
        driveFPS(Units.metersToFeet(leftMPS), Units.metersToFeet(rightMPS));
    }

    /**
     * Creates the drive motors
     *
     * @throws InitializationFailureException When follower drive motors fail to link to leaders or when leader
     *                                        drivetrain motors fail to invert
     */
    private void createDriveMotors() throws InitializationFailureException {
        leaderL = robotSettings.DRIVE_MOTOR_TYPE.createMotorOfType(robotSettings.DRIVE_MOTOR_CANBUS, robotSettings.DRIVE_LEADER_L_ID);
        leaderR = robotSettings.DRIVE_MOTOR_TYPE.createMotorOfType(robotSettings.DRIVE_MOTOR_CANBUS, robotSettings.DRIVE_LEADER_R_ID);

        //rpm <=> rps <=> gearing <=> wheel circumference
        leaderL.setRealFactorFromMotorRPM(robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI), 1 / 60D);
        leaderR.setRealFactorFromMotorRPM(robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI), 1 / 60D);

        followerL = robotSettings.DRIVE_MOTOR_TYPE.createFollowerMotorsOfType(robotSettings.DRIVE_MOTOR_CANBUS, robotSettings.DRIVE_FOLLOWERS_L_IDS);
        followerR = robotSettings.DRIVE_MOTOR_TYPE.createFollowerMotorsOfType(robotSettings.DRIVE_MOTOR_CANBUS, robotSettings.DRIVE_FOLLOWERS_R_IDS);

        followerL.follow(leaderL);
        followerR.follow(leaderR);

        leaderL.setInverted(robotSettings.DRIVE_INVERT_LEFT).resetEncoder();
        leaderR.setInverted(robotSettings.DRIVE_INVERT_RIGHT).resetEncoder();

        setAllMotorCurrentLimits(35);

        followerL.invert(robotSettings.DRIVE_INVERT_LEFT);
        followerR.invert(robotSettings.DRIVE_INVERT_RIGHT);
    }

    /**
     * Initialize the PID for the motor controllers.
     */
    private void initPID() {
        setPID(robotSettings.DRIVEBASE_PID);
        TELEOP_AIMING_PID = new PIDController(robotSettings.TELEOP_AIMING_PID.getP(), robotSettings.TELEOP_AIMING_PID.getI(), robotSettings.TELEOP_AIMING_PID.getD());//new PIDController(0.005, 0.00001, 0.0005);
    }

    /**
     * Creates xbox controller n stuff
     *
     * @throws UnsupportedOperationException when there is no configuration for {@link frc.robot.robotconfigs.DefaultConfig#DRIVE_STYLE}
     */
    private void initMisc() throws UnsupportedOperationException {
        // System.out.println("THE XBOX CONTROLLER IS ON " + robotSettings.XBOX_CONTROLLER_USB_SLOT);
        if (Objects.requireNonNull(robotSettings.DRIVE_STYLE) == DriveControlStyles.STANDARD_2023) {
            controller = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        } else {
            throw new UnsupportedOperationException("There is no UI configuration for " + robotSettings.DRIVE_STYLE.name() + " to control the drivetrain. Please implement me");
        }
        if (robotSettings.DEBUG && DEBUG)
            System.out.println("Created a " + controller.toString());
    }

    /**
     * Set all motor current limits
     *
     * @param limit Current limit in amps
     */
    public void setAllMotorCurrentLimits(int limit) {
        leaderL.setCurrentLimit(limit);
        leaderR.setCurrentLimit(limit);
        followerL.setCurrentLimit(limit);
        followerR.setCurrentLimit(limit);
    }

    /**
     * Sets the pid for all the motors that need pid setting
     *
     * @param pid the {@link PID} object that contains pertinent pidf data
     */
    private void setPID(PID pid) {
        leaderL.setPid(pid);
        leaderR.setPid(pid);
    }

    public boolean aimAtTargetPitch() {
        visionCamera.setLedMode(IVision.VisionLEDMode.ON);
        if (visionCamera.hasValidTarget()) {
            driveCringe(0, -adjustedRotation(TELEOP_AIMING_PID.calculate(visionCamera.getPitch())));
            boolean isAligned = Math.abs(visionCamera.getPitch()) <= robotSettings.AUTON_TOLERANCE * 30;
            //System.out.println("Am I aligned? " + (isAligned ? "yes" : "no"));
            if (isAligned) TELEOP_AIMING_PID.reset();
            return isAligned;
        } else {
            return false;
        }
    }

    public void driveCringe(double forward, double rotation) {
        double FPS = adjustedDrive(forward * (robotSettings.INVERT_DRIVE_DIRECTION ? -1 : 1));
        double omega = adjustedRotation(rotation);

        ChassisSpeeds cringChassis = new ChassisSpeeds(Units.feetToMeters(FPS), 0, omega);
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(cringChassis);

        double leftFPS = Units.metersToFeet(wheelSpeeds.leftMetersPerSecond);
        double rightFPS = Units.metersToFeet(wheelSpeeds.rightMetersPerSecond);

        //todo verify this doesnt suck
        leaderL.moveAtVelocity(leftFPS);
        leaderR.moveAtVelocity(rightFPS);
    }
}