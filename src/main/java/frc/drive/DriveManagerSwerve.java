package frc.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.slack.api.model.User;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.*;
import frc.motors.SwerveMotorController;
import frc.selfdiagnostics.MotorDisconnectedIssue;
import frc.sensors.camera.IVision;

import java.sql.Driver;
import java.util.Objects;

import static frc.robot.Robot.robotSettings;

/*
notes n stuff

14wide x 22long between wheels

max speed 3.6 m/s

 */
public class DriveManagerSwerve extends AbstractDriveManager {
    private static final boolean DEBUG = false;
    private final Translation2d driftOffset = new Translation2d(-0.6, 0);
    private final double trackWidth = 21;
    private final double trackLength = 24.5;
    public SwerveModuleState[] moduleStates;
    public SwerveMotorController driverFR, driverBR, driverBL, driverFL;
    public IVision visionCamera;
    boolean useLocalOrientation = false;
    double startHeading = 0;
    double rotation;
    double forwards;
    double orgDeg = 0;
    double leftwards;
    double motorRot = 0;
    double currentMotorRot = 0;
    int limelightcounter = 0;
    private Translation2d frontLeftLocation = new Translation2d(-trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701);
    private Translation2d frontRightLocation = new Translation2d(-trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701);
    private Translation2d backLeftLocation = new Translation2d(trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701);
    private Translation2d backRightLocation = new Translation2d(trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701);
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
    private PIDController FRpid, BRpid, BLpid, FLpid;
    private PIDController limeLightPid, leveling;
    private BaseController xbox, midiTop, midiBot, panel1, panel2;
    private CANCoder FRcoder, BRcoder, BLcoder, FLcoder;
    private double leveling_auto_roll = 0, Leveling_auto_Pitch = 0;
    boolean brokeHigh = false, fallLow = false;
    private PIDController X_PID;
    private PIDController Y_PID;
    private double TAPX = 0, TAPY = 0, TAPRotation= 0, TAPSpeed = 0;

    public DriveManagerSwerve() {
        super();
    }

    @Override
    public void init() {
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        createPIDControllers(new PID(0.0111, 0.0, 0.0));
        createDriveMotors();
        setDrivingPIDS(new PID(0.0002, 0, 0.0001, 0.03));
        setCANCoder();
        setupSteeringEncoders();
        setKin();
        limeLightPid = new PIDController(robotSettings.limeLightPid.P, robotSettings.limeLightPid.I, robotSettings.limeLightPid.D);
        leveling =  new PIDController(robotSettings.leveling.P, robotSettings.leveling.I, robotSettings.leveling.D);
        X_PID = new PIDController(robotSettings.TELE_XYPID.P , robotSettings.TELE_XYPID.I, robotSettings.TELE_XYPID.D);
        Y_PID = new PIDController(robotSettings.TELE_XYPID.P , robotSettings.TELE_XYPID.I, robotSettings.TELE_XYPID.D);


        if (robotSettings.ENABLE_VISION) {
            visionCamera = IVision.manufactureGoalCamera(robotSettings.GOAL_CAMERA_TYPE);
        }
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        if (driverFR.driver.isFailed() || driverBR.driver.isFailed() || driverFL.driver.isFailed() || driverBL.driver.isFailed() || driverFR.steering.isFailed() || driverBR.steering.isFailed() || driverFL.steering.isFailed() || driverBL.steering.isFailed())
            return SubsystemStatus.FAILED;
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public void updateTest() {
        updateGeneric();
        //if (robotSettings.DEBUG && DEBUG) {
            System.out.println(FRcoder.getAbsolutePosition() + " FR " /*+ driverFR.steering.getRotations()*/);
            System.out.println(FLcoder.getAbsolutePosition() + " FL " /* + driverFL.steering.getRotations()*/);
            System.out.println(BRcoder.getAbsolutePosition() + " BR " /* + driverBR.steering.getRotations()*/);
            System.out.println(BLcoder.getAbsolutePosition() + " BL " /* + driverBL.steering.getRotations()*/);
            //System.out.println();
            //System.out.println(guidance.imu.relativeYaw());
        //}
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
        TAPUpdate();
        if(xbox.get(DefaultControllerEnums.XBoxButtons.GUIDE) == DefaultControllerEnums.ButtonStatus.DOWN){
            TAP(TAPX,TAPY,TAPRotation,TAPSpeed);
        }else {
            driveSwerve();
        }

        if (xbox.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) {
            guidance.setSwerveOdometryCurrent(guidance.fieldX(), guidance.fieldY());
            guidance.resetOdometry();
        }
        if(xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                startHeading = 180;
            }else{
                startHeading = 0;
            }
        }
    }

    @Override
    public void updateAuton() {


    }

    @Override
    public void initTest() {
        resetSteeringEncoders();
        setupSteeringEncoders();
    }

    @Override
    public void initTeleop() {
        setupSteeringEncoders();
        resetSteeringEncoders();
        setBrake(true);
        //setSteeringContinuous(FLcoder.getAbsolutePosition(), FRcoder.getAbsolutePosition(),BLcoder.getAbsolutePosition(),BRcoder.getAbsolutePosition());
        useLocalOrientation = false;
        guidance.imu.resetOdometry();
    }

    @Override
    public void initAuton() {
        setBrake(true);
        setKin();
       //setSteeringContinuous(FLcoder.getAbsolutePosition(), FRcoder.getAbsolutePosition(),BLcoder.getAbsolutePosition(),BRcoder.getAbsolutePosition());
    }

    @Override
    public void initDisabled() {
    }


    @Override
    public void initGeneric() {

    }

    /**
     * reset steering motor encoders
     */
    private void resetSteeringEncoders() {
        driverFR.steering.resetEncoder();
        driverBR.steering.resetEncoder();
        driverFL.steering.resetEncoder();
        driverBL.steering.resetEncoder();
    }

    private void driveSwerve() {
        forwards = xbox.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_Y) * (1);
        if (robotSettings.ENABLE_VISION && xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN && !robotSettings.BRANDONISNOTHERE) {
            visionCamera.setLedMode(IVision.VisionLEDMode.ON);
            if(visionCamera.hasValidTarget()) {
                //System.out.println("AIMING");
                leftwards = limeLightPid.calculate(visionCamera.getAngle());
            }
        } else {
            visionCamera.setLedMode(IVision.VisionLEDMode.OFF);
            leftwards = xbox.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_X) * (-1);
        }
            //visionCamera.setLedMode(IVision.VisionLEDMode.OFF);
        if (Math.abs(xbox.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_X)) >= .2) {
            rotation = xbox.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_X) * (-1.6);
            startHeading = guidance.imu.relativeYaw();

        } else if(Math.abs(xbox.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_X)) <= 0.15 && Math.abs(xbox.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_X)) <= 0.15 && Math.abs(xbox.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_Y)) <= 0.15) {
            rotation = 0;
            //startHeading = guidance.imu.relativeYaw();
       // }else if(xbox.get(DefaultControllerEnums.XBoxButtons.MENU) == DefaultControllerEnums.ButtonStatus.DOWN){
        //    rotation = xbox.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_X)*(-1.6);
        }else{
            rotation = (guidance.imu.relativeYaw() - startHeading) * -.05;
        }

        if(xbox.get(DefaultControllerEnums.XboxAxes.RIGHT_TRIGGER) >= 0.5){
            forwards *= .25;
            leftwards *= .25;
            rotation *= .25;
        }

        if(xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN && !robotSettings.BRANDONISNOTHERE){
            leveling();
        }else {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                driveMPS(adjustedDrive(-forwards), adjustedDrive(-leftwards), adjustedRotation(rotation));
            }else {
                driveMPS(adjustedDrive(forwards), adjustedDrive(leftwards), adjustedRotation(rotation));
            }
        }

    }

    private boolean useLocalOrientation() {
        return (xbox.get(DefaultControllerEnums.XboxAxes.LEFT_TRIGGER) > 0.1 || useLocalOrientation);
    }

    private boolean dorifto() {
        return false;
    }

    /**
     * Sets the drive steering
     *
     * @param FL Front left translation requested. units?
     * @param FR Front right translation requested. units?
     * @param BL Back left translation requested. units?
     * @param BR Back right translation requested. units?
     */
    private void setSteeringContinuous(double FL, double FR, double BL, double BR) {
        // try removing off set
        // try forcing Fl,FR,BL,BR 0
        UserInterface.smartDashboardPutNumber("front left turn gaol", Math.toRadians(FL));
        UserInterface.smartDashboardPutNumber("front Right turn gaol", Math.toRadians(FR));
        UserInterface.smartDashboardPutNumber("Back left turn gaol", Math.toRadians(BL));
        UserInterface.smartDashboardPutNumber("Back right turn gaol", Math.toRadians(BR));
        FLpid.setSetpoint(-FL);
        FRpid.setSetpoint(-FR);
        BRpid.setSetpoint(-BR);
        BLpid.setSetpoint(-BL);
        //System.out.println(driverFL.steering.getRotations());
        //System.out.println("setpoint no offset: " + FR);
        //System.out.println("Absolute Position/F current positiion FL: " + FLcoder.getAbsolutePosition());
        //System.out.println("Absolute Position/ current positiion FR: " + FRcoder.getAbsolutePosition());
        //System.out.println("Absolute Position/ current positiion BL: " + BLcoder.getAbsolutePosition());
        //System.out.println("Absolute Position/ current positiion BR: " + BRcoder.getAbsolutePosition());
        // System.out.println("turning speed/pid should be: " + FLpid.calculate(FLcoder.getAbsolutePosition()));
        driverFL.steering.moveAtPercent(-FLpid.calculate(FLcoder.getAbsolutePosition()));
        driverFR.steering.moveAtPercent(-FRpid.calculate(FRcoder.getAbsolutePosition()));
        driverBL.steering.moveAtPercent(-BLpid.calculate(BLcoder.getAbsolutePosition()));
        driverBR.steering.moveAtPercent(-BRpid.calculate(BRcoder.getAbsolutePosition()));

        // driverFL.getState(), driverFR.getState(), driverBL.getState(), driverBR.getState()

    }

    /**
     * Drives the bot in percent control mode based on inputs
     *
     * @param FL {@link #driverFL} requested drive (-3.5, 3.5)
     * @param FR {@link #driverFR} requested drive (-3.5, 3.5)
     * @param BL {@link #driverBL} requested drive (-3.5, 3.5)
     * @param BR {@link #driverBR} requested drive (-3.5, 3.5)
     */
    private void setDrive(double FL, double FR, double BL, double BR) {
        double FPS_FL = Units.metersToFeet(FL);
        double FPS_FR = Units.metersToFeet(FR);
        double FPS_BL = Units.metersToFeet(BL);
        double FPS_BR = Units.metersToFeet(BR);
        double num = 19;
        if (robotSettings.DEBUG && DEBUG) {
            System.out.println("FL: " + FL);
            System.out.println("FR: " + FR);
            System.out.println("BL: " + BL);
            System.out.println("BR: " + BR);
        }

        /*driverFR.driver.moveAtVelocity(FPS_FR);
        driverBR.driver.moveAtVelocity(FPS_BR);
        driverFL.driver.moveAtVelocity(FPS_FL);
        driverBL.driver.moveAtVelocity(FPS_BL);*/


        double gearRatio = 1;//robotSettings.SWERVE_SDS_DRIVE_BASE.getDriveReduction() * robotSettings.SWERVE_SDS_DRIVE_BASE.getWheelDiameter();
        double voltageMult = 95 / 371.0; // 127.4/371.0 is full speed
        //System.out.println(adjustedDriveVoltage((FPS_FR) * gearRatio * robotSettings.DRIVE_SCALE, voltageMult));
        driverFR.driver.moveAtVoltage(adjustedDriveVoltage((FPS_FR) * gearRatio * robotSettings.DRIVE_SCALE, voltageMult));
        driverFL.driver.moveAtVoltage(adjustedDriveVoltage((FPS_FL) * gearRatio * robotSettings.DRIVE_SCALE, voltageMult));
        driverBR.driver.moveAtVoltage(adjustedDriveVoltage((FPS_BR) * gearRatio * robotSettings.DRIVE_SCALE, voltageMult));
        driverBL.driver.moveAtVoltage(adjustedDriveVoltage((FPS_BL) * gearRatio * robotSettings.DRIVE_SCALE, voltageMult));
    }

    /**
     * set steering motors to return their encoder position in degrees
     */

    private void setupSteeringEncoders() {
        //12.8:1
        driverFR.steering.setRealFactorFromMotorRPM((1 / 12.8) * 360, 1);
        driverBR.steering.setRealFactorFromMotorRPM((1 / 12.8) * 360, 1);
        driverFL.steering.setRealFactorFromMotorRPM((1 / 12.8) * 360, 1);
        driverBL.steering.setRealFactorFromMotorRPM((1 / 12.8) * 360, 1);
    }

    //TODO implement this in regard to telem
    @Override
    public void resetDriveEncoders() {
        driverFR.driver.resetEncoder();
        driverFL.driver.resetEncoder();
        driverBR.driver.resetEncoder();
        driverBL.driver.resetEncoder();
    }

    @Override
    public void setBrake(boolean brake) {
        driverFR.driver.setBrake(brake);
        driverFL.driver.setBrake(brake);
        driverBL.driver.setBrake(brake);
        driverBR.driver.setBrake(brake);
    }

    public void setBrakeTurngin(boolean brake) {
        driverFR.steering.setBrake(brake);
        driverFL.steering.setBrake(brake);
        driverBL.steering.setBrake(brake);
        driverBR.steering.setBrake(brake);
    }

    @Override
    public void driveMPS(double xMeters, double yMeters, double rotation) { // after here
        UserInterface.smartDashboardPutNumber("xMeters ",xMeters);
        UserInterface.smartDashboardPutNumber("yMeters ",yMeters);
        UserInterface.smartDashboardPutNumber("rotation ",rotation);
        UserInterface.smartDashboardPutNumber("tot magitude climb", Math.sqrt((Math.pow(guidance.imu.relativeRoll(),2) + Math.pow(guidance.imu.relativePitch(),2))));
        ChassisSpeeds speeds;
        //System.out.println("xmeters: + " + xMeters + "ymeters: " + yMeters + "rotation + " + rotation);
        //x+ m/s forwards, y+ m/s left, omega+ rad/sec ccw
        if (useLocalOrientation()) {
            speeds = new ChassisSpeeds(xMeters, yMeters, rotation);
        } else if (dorifto()) {
            speeds = new ChassisSpeeds(xMeters, 0, rotation);
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xMeters, yMeters, rotation, Rotation2d.fromDegrees(-guidance.swerveRobotPose.getEstimatedPosition().getRotation().getDegrees()));
        }
        driveWithChassisSpeeds(speeds);
    }
    @Override
    public void drivePure(double forward, double omega) {
        drivePure(forward, 0, omega);
    }

    @Override
    public void drivePure(double forward, double sideways, double omega) {
        driveMPS(forward, sideways, adjustedRotation(omega));
    }

    @Override

    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        moduleStates = kinematics.toSwerveModuleStates(speeds);

        //System.out.println("notice me");
        if (xbox.get(DefaultControllerEnums.XBoxButtons.RIGHT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) { // ignore for now
            moduleStates = kinematics.toSwerveModuleStates(speeds, frontRightLocation);
        } else if (dorifto()) {
            double driftOffset = 3;
            double offset = trackLength / 2 / 39.3701;
            offset -= speeds.vxMetersPerSecond / driftOffset;
            //System.out.println("forwards: " + speeds.vxMetersPerSecond);
            moduleStates = kinematics.toSwerveModuleStates(speeds, new Translation2d(offset, 0));
        }



        // Front left module state
        SwerveModuleState frontLeft = moduleStates[0], frontRight = moduleStates[1], backLeft = moduleStates[2], backRight = moduleStates[3];
        //try continuous here

        //System.out.println("FieldX: " + -guidance.fieldX());
        //System.out.println("FieldY: " + -guidance.fieldY());

//        if(xbox.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_X) == 0 && xbox.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_X) == 0 && xbox.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_Y) == 0) {
//            setDrive(0, 0, 0, 0); // before here
//        }else {
            setSteeringContinuous(frontLeft.angle.getDegrees(), frontRight.angle.getDegrees(), backLeft.angle.getDegrees(), backRight.angle.getDegrees()); // <-- maybe here

            setDrive(frontLeft.speedMetersPerSecond, frontRight.speedMetersPerSecond, backLeft.speedMetersPerSecond, backRight.speedMetersPerSecond);
        //}
        if (DEBUG) {
            //System.out.printf("%4f %4f %4f %4f \n", frontLeft.speedMetersPerSecond, frontRight.speedMetersPerSecond, backLeft.speedMetersPerSecond, backRight.speedMetersPerSecond);
        }
    }

    @Override
    public void updateGeneric() {
        UserInterface.smartDashboardPutNumber("DriverFL Absoluto positon", driverFL.driver.getRotations());
        UserInterface.smartDashboardPutNumber("DriverFR Absoluto positon", driverFR.driver.getRotations());
        UserInterface.smartDashboardPutNumber("DriverBL Absoluto positon", driverBL.driver.getRotations());
        UserInterface.smartDashboardPutNumber("DriverBR Absoluto positon", driverBR.driver.getRotations());
        UserInterface.smartDashboardPutNumber("DriverFL position", Math.toRadians(FLcoder.getAbsolutePosition()));
        UserInterface.smartDashboardPutNumber("DriverFR position", Math.toRadians(FRcoder.getAbsolutePosition()));
        UserInterface.smartDashboardPutNumber("DriverBL position", Math.toRadians(BLcoder.getAbsolutePosition()));
        UserInterface.smartDashboardPutNumber("DriverBR position", Math.toRadians(BRcoder.getAbsolutePosition()));
        UserInterface.smartDashboardPutNumber("Field X", guidance.fieldX());
        UserInterface.smartDashboardPutNumber("Field Y", guidance.fieldY());
        UserInterface.smartDashboardPutNumber("DriverFL current", driverFL.driver.getCurrent());
        UserInterface.smartDashboardPutNumber("DriverFR current", driverFR.driver.getCurrent());
        UserInterface.smartDashboardPutNumber("DriverBL current", driverBL.driver.getCurrent());
        UserInterface.smartDashboardPutNumber("DriverBR current", driverBR.driver.getCurrent());
        UserInterface.smartDashboardPutNumber("SteeringFL current", driverFL.steering.getCurrent());
        UserInterface.smartDashboardPutNumber("SteeringFR current", driverFR.steering.getCurrent());
        UserInterface.smartDashboardPutNumber("SteeringBL current", driverBL.steering.getCurrent());
        UserInterface.smartDashboardPutNumber("SteeringBR current", driverBR.steering.getCurrent());
        MotorDisconnectedIssue.handleIssue(this, driverFL.driver);
        MotorDisconnectedIssue.handleIssue(this, driverFL.steering);
        MotorDisconnectedIssue.handleIssue(this, driverBL.driver);
        MotorDisconnectedIssue.handleIssue(this, driverBL.steering);
        MotorDisconnectedIssue.handleIssue(this, driverFR.driver);
        MotorDisconnectedIssue.handleIssue(this, driverFR.steering);
        MotorDisconnectedIssue.handleIssue(this, driverBR.driver);
        MotorDisconnectedIssue.handleIssue(this, driverBR.steering);
    }

    @Override
    protected void onControlChange() {
        //pass
    }

    /**
     * Sets the pid for all steering motors
     *
     * @param pid the pid for the swerve steering motors
     * @deprecated (For now, dont use this since the PID in the motors arent continuous)
     */
    @Deprecated
    private void setSteeringPIDS(PID pid) {
        driverFR.steering.setPid(pid);
        driverBR.steering.setPid(pid);
        driverFL.steering.setPid(pid);
        driverBL.steering.setPid(pid);
    }

    /**
     * Sets the pid for all drive motors
     *
     * @param pid the pid for the swerve drive motors
     */
    private void setDrivingPIDS(PID pid) {
        driverFR.driver.setPid(pid);
        driverBR.driver.setPid(pid);
        driverFL.driver.setPid(pid);
        driverBL.driver.setPid(pid);
    }

    private void setKin(){
        Translation2d FLPos = Objects.requireNonNullElseGet(frontLeftLocation, () -> frontLeftLocation = new Translation2d(-trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701));
        Translation2d FRPos = Objects.requireNonNullElseGet(frontRightLocation, () -> frontRightLocation = new Translation2d(-trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701));
        Translation2d BLPos = Objects.requireNonNullElseGet(backLeftLocation, () -> backLeftLocation = new Translation2d(trackLength / 2 / 39.3701, trackWidth / 2 / 39.3701));
        Translation2d BRPos = Objects.requireNonNullElseGet(backRightLocation, () -> backRightLocation = new Translation2d(trackLength / 2 / 39.3701, -trackWidth / 2 / 39.3701));
        kinematics = Objects.requireNonNullElseGet(kinematics, () -> kinematics = new SwerveDriveKinematics(FLPos, FRPos, BLPos, BRPos));
    }

    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{
                driverFL.getState(), driverFR.getState(), driverBL.getState(), driverBR.getState()
        };
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[]{
                new SwerveModulePosition(driverFL.driver.getRotations(), Rotation2d.fromDegrees(FLcoder.getAbsolutePosition())),
                new SwerveModulePosition(driverFR.driver.getRotations(),  Rotation2d.fromDegrees(FLcoder.getAbsolutePosition())),
                new SwerveModulePosition(driverBL.driver.getRotations(),  Rotation2d.fromDegrees(FLcoder.getAbsolutePosition())),
                new SwerveModulePosition(driverBR.driver.getRotations(),  Rotation2d.fromDegrees(FLcoder.getAbsolutePosition()))
        };
    }


    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    private void createDriveMotors() throws InitializationFailureException {
        driverFR = new SwerveMotorController(robotSettings.SWERVE_DRIVE_FR, robotSettings.DRIVE_MOTOR_TYPE, robotSettings.SWERVE_TURN_FR, robotSettings.DRIVE_MOTOR_TYPE);
        driverBR = new SwerveMotorController(robotSettings.SWERVE_DRIVE_BR, robotSettings.DRIVE_MOTOR_TYPE, robotSettings.SWERVE_TURN_BR, robotSettings.DRIVE_MOTOR_TYPE);
        driverBL = new SwerveMotorController(robotSettings.SWERVE_DRIVE_BL, robotSettings.DRIVE_MOTOR_TYPE, robotSettings.SWERVE_TURN_BL, robotSettings.DRIVE_MOTOR_TYPE);
        driverFL = new SwerveMotorController(robotSettings.SWERVE_DRIVE_FL, robotSettings.DRIVE_MOTOR_TYPE, robotSettings.SWERVE_TURN_FL, robotSettings.DRIVE_MOTOR_TYPE);

        //rpm <=> rps <=> gearing <=> wheel circumference
        driverFR.driver.setRealFactorFromMotorRPM(robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI / 12), 1 / 60D);
        driverFL.driver.setRealFactorFromMotorRPM(robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI / 12), 1 / 60D);
        driverBR.driver.setRealFactorFromMotorRPM(robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI / 12), 1 / 60D);
        driverBL.driver.setRealFactorFromMotorRPM(robotSettings.DRIVE_GEARING * (robotSettings.WHEEL_DIAMETER * Math.PI / 12), 1 / 60D);

        driverFR.driver.setInverted(false).setBrake(true);
        driverFL.driver.setInverted(true).setBrake(true);
        driverBR.driver.setBrake(true);
        driverBL.driver.setInverted(true).setBrake(true);

        driverFR.steering.setInverted(true);
        driverBR.steering.setInverted(true);
        driverBL.steering.setInverted(true);
        driverFL.steering.setInverted(true);
    }

    public void setCANCoder() {
        FLcoder = new CANCoder(11, robotSettings.DRIVE_MOTOR_CANBUS);
        FRcoder = new CANCoder(12, robotSettings.DRIVE_MOTOR_CANBUS);
        BRcoder = new CANCoder(13, robotSettings.DRIVE_MOTOR_CANBUS);
        BLcoder = new CANCoder(14, robotSettings.DRIVE_MOTOR_CANBUS);
        FLcoder.configMagnetOffset(-16.5234375 - Math.toDegrees(0.07));
        FRcoder.configMagnetOffset(-25.048828125 - Math.toDegrees(0.17));
        BLcoder.configMagnetOffset(-169.716796875 - Math.toDegrees(0.02));
        BRcoder.configMagnetOffset(-56.337890625 - Math.toDegrees(0.2));
    }

    public void createPIDControllers(PID steeringPID) {
        FLpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        FRpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        BLpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        BRpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        BRpid = new PIDController(steeringPID.P, steeringPID.I, steeringPID.D);
        FLpid.enableContinuousInput(-180, 180);
        FRpid.enableContinuousInput(-180, 180);
        BLpid.enableContinuousInput(-180, 180);
        BRpid.enableContinuousInput(-180, 180);
    }

    public boolean driveWheetRot(double move) {
        useLocalOrientation = true;
        if (motorRot == 0) {
            motorRot = Math.abs(move);
            currentMotorRot = driverFR.driver.getRotations();
        }

        //System.out.println("original motor rotation: " + currentMotorRot + "motorRot = " + motorRot + "how far motor has moved: " + driverFR.driver.getRotations());
        if ((driverFR.driver.getRotations() > currentMotorRot - motorRot) && move > 0) {
            driveMPS(adjustedDrive(-.3), adjustedDrive(0), adjustedRotation(0));
            return false;
        } else if ((driverFR.driver.getRotations() < currentMotorRot + motorRot) && move < 0) {
            driveMPS(adjustedDrive(.3), adjustedDrive(0), adjustedRotation(0));
            return false;
        } else {
            motorRot = 0;
            lockWheels();
            //System.out.println("we are all done in here");
            return true;
        }
    }

    public boolean driveWheetRot(double move, double percent) {
        useLocalOrientation = true;
        if (motorRot == 0) {
            motorRot = Math.abs(move);
            currentMotorRot = driverFR.driver.getRotations();
        }

        //System.out.println("original motor rotation: " + currentMotorRot + "motorRot = " + motorRot + "how far motor has moved: " + driverFR.driver.getRotations());
        if ((driverFR.driver.getRotations() > currentMotorRot - motorRot) && move > 0) {
            driveMPS(adjustedDrive(-percent), adjustedDrive(0), adjustedRotation(0));
            return false;
        } else if ((driverFR.driver.getRotations() < currentMotorRot + motorRot) && move < 0) {
            driveMPS(adjustedDrive(percent), adjustedDrive(0), adjustedRotation(0));
            return false;
        } else {
            motorRot = 0;
            lockWheels();
           // System.out.println("we are all done in here");
            return true;
        }
    }

    public boolean driveWheetRot(double move, double percent, double movedeg) {
        useLocalOrientation = false;
        if (motorRot == 0) {
            motorRot = Math.abs(move);
            currentMotorRot = driverFR.driver.getRotations();
        }

        if ((driverFR.driver.getRotations() > currentMotorRot - motorRot) && move > 0) {
            driveMPS(adjustedDrive(-percent), adjustedDrive(0), adjustedRotation(0));
            return false;
        } else if ((driverFR.driver.getRotations() < currentMotorRot + motorRot) && move < 0) {
            driveMPS(adjustedDrive(percent), adjustedDrive(0), adjustedRotation(0));
            return false;
        } else {
            motorRot = 0;
            lockWheels();
            //System.out.println("we are all done in here");
            return true;
        }
    }

    public boolean turnDegree(int degrees) {
        if (orgDeg == 0)
            orgDeg = guidance.imu.relativeYaw();
        if (orgDeg + degrees >= guidance.imu.relativeYaw() && degrees > 0) {
            driveMPS(adjustedDrive(0), adjustedDrive(0), adjustedRotation(.55));
            //System.out.println("The original degree was: " + orgDeg + "The Current Degree was: " + guidance.imu.relativeYaw());
            return false;


        } else if (orgDeg + degrees <= guidance.imu.relativeYaw() && degrees < 0) {
            driveMPS(adjustedDrive(0), adjustedDrive(0), adjustedRotation(-.55));
            //System.out.println("The original degree was: " + orgDeg + "The Current Degree was: " + guidance.imu.relativeYaw());
            return false;
        } else {
            lockWheels();
            orgDeg = 0;
            System.out.println("finished turning");
            return true;
        }
    }

    public boolean aimYaw() {
        visionCamera.setLedMode(IVision.VisionLEDMode.ON);
        double neededRot = -(visionCamera.getAngle() / 15);
        driveMPS(adjustedDrive(0), adjustedDrive(0), adjustedRotation(neededRot));
        if (!visionCamera.hasValidTarget()) {
            //System.out.println("not target found");
            if (limelightcounter == 1) {
                lockWheels();
                return true;
            }
            limelightcounter++;
        }
        if (Math.abs(visionCamera.getAngle()) <= 4) {
            //System.out.println("inrange of limelight");
            lockWheels();
            return true;
        }
        System.out.println("turning");
        return false;
    }
    @Override
    public void lockWheels() {
        driveMPS(0.0, 0, 0.02);
    }
    @Override
    public boolean leveling(){
        if(leveling_auto_roll == 0 && Leveling_auto_Pitch == 0){
            leveling_auto_roll = guidance.imu.relativeRoll();
            Leveling_auto_Pitch = guidance.imu.relativeRoll();
        }

        double totalMagnetude = Math.sqrt((Math.pow(guidance.imu.relativeRoll(),2) + Math.pow(guidance.imu.relativePitch(),2)));
        forwards = -leveling.calculate(totalMagnetude);

        if(guidance.imu.relativeRoll() < 0)
            forwards *= -1;

        if(totalMagnetude > 12 ){
            brokeHigh = true;
        }
        if (brokeHigh && totalMagnetude < 10){
            fallLow = true;
            lockWheels();
        }else {
            if(forwards < -1.89D/12)
                forwards = -1.89D/12;
            if(forwards > 1.89D/12)
                forwards = 1.89D/12;

        }

        if(Math.abs(guidance.imu.relativeRoll()) <= 6){
            fallLow = false;
            brokeHigh = false;
            forwards = 0;
            return true;
        }
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
            drivePure(-adjustedDrive(forwards), adjustedDrive(0), adjustedRotation(0));
        }else {
            drivePure(adjustedDrive(forwards), adjustedDrive(0), adjustedRotation(0));
        }
        return false;
    }
    /**
        TAP Teleop Attack Point
        named this to piss off everyone :P
    */
    public void TAP(double targetX, double targetY, double targetRotaion, double speed){

        double x = targetX - guidance.fieldX();
        double y = targetY - guidance.fieldY();

        double calcX = X_PID.calculate(x);
        double calcY = Y_PID.calculate(y);

        double rotation = 0;
        double signNeeded = 1;
        double at = guidance.swerveRobotPose.getEstimatedPosition().getRotation().getDegrees();
        double goal = targetRotaion;

        rotation = UtilFunctions.mathematicalMod((goal - at) + 180, 360) - 180;
        drivePure(-robotSettings.AUTO_SPEED * speed * calcX, robotSettings.AUTO_SPEED * speed * calcY, rotation * signNeeded * .08);
    }

    public void TAPUpdate(){
        int scoreColumnInt = -1;
        int scoreRowInt = -1;
        double[][] scoringArray;
        double[][] pickUpArray;
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
            scoringArray = robotSettings.blue2DScoringArray;
            pickUpArray = robotSettings.blue2DPickUpArray;
        }else{
            scoringArray = robotSettings.red2DScoringArray;
            pickUpArray = robotSettings.red2DPickUpArray;
        }
        switch (robotSettings.DRIVE_STYLE){
            case STANDARD_2023:{
                //SPIKE
        if(panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTShute) == DefaultControllerEnums.ButtonStatus.DOWN){
            TAPX= pickUpArray[2][0];
            TAPY= pickUpArray[2][1];
            TAPRotation= pickUpArray[2][2];
            TAPSpeed= pickUpArray[2][3];
        }
        //gts1
        if(panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == DefaultControllerEnums.ButtonStatus.DOWN){
            TAPX= pickUpArray[0][0];
            TAPY= pickUpArray[0][1];
            TAPRotation= pickUpArray[0][2];
            TAPSpeed= pickUpArray[0][3];
        }
        //gts2
        if(panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation2) == DefaultControllerEnums.ButtonStatus.DOWN){
            TAPX= pickUpArray[1][0];
            TAPY= pickUpArray[1][1];
            TAPRotation= pickUpArray[1][2];
            TAPSpeed= pickUpArray[1][3];
        }

        ControllerEnums.ButtonPanelButtonsPlacement2023[] columnButtons = new ControllerEnums.ButtonPanelButtonsPlacement2023[]{
                ControllerEnums.ButtonPanelButtonsPlacement2023.T1,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T2,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T3,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T4,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T5,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T6,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T7,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T8,
                ControllerEnums.ButtonPanelButtonsPlacement2023.T9,
        };

        for (int i = 0; i < columnButtons.length; i++) {
            if(panel2.get(columnButtons[i]) == DefaultControllerEnums.ButtonStatus.DOWN)
                scoreColumnInt = i;
        }
                break;
            }
            case MIDI:{
                //SPIKE
        if(midiBot.get(ControllerEnums.MidiController.R2C4) == DefaultControllerEnums.ButtonStatus.DOWN){
            TAPX= pickUpArray[2][0];
            TAPY= pickUpArray[2][1];
            TAPRotation= pickUpArray[2][2];
            TAPSpeed= pickUpArray[2][3];
        }
        //gts1
        if(midiBot.get(ControllerEnums.MidiController.R1C3) == DefaultControllerEnums.ButtonStatus.DOWN){
            TAPX= pickUpArray[0][0];
            TAPY= pickUpArray[0][1];
            TAPRotation= pickUpArray[0][2];
            TAPSpeed= pickUpArray[0][3];
        }
        //gts2
        if(midiBot.get(ControllerEnums.MidiController.R1C4) == DefaultControllerEnums.ButtonStatus.DOWN){
            TAPX= pickUpArray[1][0];
            TAPY= pickUpArray[1][1];
            TAPRotation= pickUpArray[1][2];
            TAPSpeed= pickUpArray[1][3];
        }

        ControllerEnums.MidiController[] columnButtons = new ControllerEnums.MidiController[]{
                ControllerEnums.MidiController.R2C6,
                ControllerEnums.MidiController.R2C7,
                ControllerEnums.MidiController.R2C8,
                ControllerEnums.MidiController.R3C6,
                ControllerEnums.MidiController.R3C7,
                ControllerEnums.MidiController.R3C8,
                ControllerEnums.MidiController.R4C6,
                ControllerEnums.MidiController.R4C7,
                ControllerEnums.MidiController.R4C8,
        };
        for (int i = 0; i < columnButtons.length; i++) {
            if(midiBot.get(columnButtons[i]) == DefaultControllerEnums.ButtonStatus.DOWN)
                scoreColumnInt = i;
        }

        ControllerEnums.MidiController[] rowButtons = new ControllerEnums.MidiController[]{
                ControllerEnums.MidiController.R1C1,
                ControllerEnums.MidiController.R2C1,
                ControllerEnums.MidiController.R3C1,
        };
        for (int i = 0; i < rowButtons.length; i++) {
            if(midiTop.get(rowButtons[i]) == DefaultControllerEnums.ButtonStatus.DOWN)
                scoreRowInt = i;
        }
                break;
            }
        }

        if(scoreRowInt != -1 && scoreColumnInt != -1) {
            TAPX = scoringArray[scoreColumnInt][scoreRowInt];
            TAPY = scoringArray[scoreColumnInt][3];
            TAPRotation = scoringArray[scoreColumnInt][4];
            TAPSpeed = scoringArray[scoreColumnInt][5];
        }

        UserInterface.smartDashboardPutNumber("TAP X" , TAPX);
        UserInterface.smartDashboardPutNumber("TAP Y" , TAPY);
        UserInterface.smartDashboardPutNumber("TAP ROTATION" , TAPRotation);
        UserInterface.smartDashboardPutNumber("TAP SPEED" , TAPSpeed);

    }
}


