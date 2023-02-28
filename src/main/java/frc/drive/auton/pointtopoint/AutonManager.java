package frc.drive.auton.pointtopoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AbstractAutonManager;
import frc.drive.auton.Point;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.misc.UtilFunctions;
import frc.robot.Robot;

import static frc.robot.Robot.autonManager;
import static frc.robot.Robot.robotSettings;

public class AutonManager extends AbstractAutonManager {
    public final Timer timer = new Timer();
    private final PIDController ROT_PID;
    private final PIDController X_PID;
    private final PIDController Y_PID;
    public AbstractDriveManager drivingChild;
    public boolean firstTimerRun = false;
    public AutonRoutines autonPath;
    public boolean specialActionComplete = false;
    public boolean specialActionComplete2 = false;
    public double yawBeforeTurn = 0, rotationOffset = 0.01;
    private boolean isInTolerance = false;
    private boolean isAiming = false;

    public AutonManager(AutonRoutines routine, AbstractDriveManager driveManager) {
        super(driveManager);
        addToMetaList();
        drivingChild = driveManager;
        autonPath = routine;
        ROT_PID = new PIDController(robotSettings.HEADING_PID.P, robotSettings.HEADING_PID.I, robotSettings.HEADING_PID.D);
        X_PID = new PIDController(robotSettings.AUTO_XPID.P , robotSettings.AUTO_XPID.I, robotSettings.AUTO_XPID.D);
        Y_PID = new PIDController(robotSettings.AUTO_YPID.P , robotSettings.AUTO_YPID.I, robotSettings.AUTO_YPID.D);
        init();
    }

    @Override
    public void init() {
        super.initAuton();
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {
    }

    @Override
    public void updateGeneric() {
        UserInterface.smartDashboardPutNumber("Auton Stage", autonPath.currentWaypoint);
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    //getMeters - get wheel meters traveled

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return drivingChild.getSubsystemStatus() == SubsystemStatus.NOMINAL && drivingChild.guidance.getSubsystemStatus() == SubsystemStatus.NOMINAL ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    @Override
    public void updateAuton() {
        if (autonPath.currentWaypoint >= autonPath.WAYPOINTS.size())
            return;
        updateGeneric();
        System.out.println("Home is: " + autonPath.WAYPOINTS.get(0).LOCATION + " and im going to " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION));
        Point point = (autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION);
        if (attackPoint(point, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPEED, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION == AutonSpecialActions.DRIVE_TO && autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG != 0) || isInTolerance) {
            isInTolerance = true;
            DriverStation.reportWarning("IN TOLERANCE", false);
            System.out.println("Special Action: " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION.toString());
            switch (autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION) {
                case WAIT500:
                    if(!firstTimerRun){
                        timer.reset();
                        firstTimerRun = true;
                    }
                    specialActionComplete = timer.advanceIfElapsed(.5);
                    if(specialActionComplete)
                        firstTimerRun = false;
                    break;
                case WAIT1000:
                    if(!firstTimerRun){
                        timer.reset();
                        firstTimerRun = true;
                    }
                    specialActionComplete = timer.advanceIfElapsed(1);
                    if(specialActionComplete)
                        firstTimerRun = false;
                    break;
                case WAIT100:
                    if(!firstTimerRun){
                        timer.reset();
                        firstTimerRun = true;
                    }
                    specialActionComplete = timer.advanceIfElapsed(.1);
                    if(specialActionComplete)
                        firstTimerRun = false;
                    break;
                case AUTO_LEVEL:
                    System.out.println("TRYINginging");
                    specialActionComplete = drivingChild.leveling();
                    break;
                case LOCK_WHEELS:
                    drivingChild.lockWheels();
                    specialActionComplete = false;
                    break;
                case INTAKE_PISTON_IN:
                    Robot.intake.intakeIn();
                    specialActionComplete = true;
                    break;
                case INTAKE_PISTON_OUT:
                    Robot.intake.intakeOut();
                    specialActionComplete = true;
                    break;
                case INTAKE_WHEEL_IN:
                    Robot.intake.intakeRight.moveAtVoltage(6);
                    Robot.intake.intakeLeft.moveAtVoltage(-6);
                    specialActionComplete = true;
                    break;
                case INTAKE_WHEEL_OUT:
                    Robot.intake.intakeRight.moveAtVoltage(-12);
                    Robot.intake.intakeLeft.moveAtVoltage(12);
                    specialActionComplete = true;
                    break;
                case INTAKE_WHEEL_OFF:
                    Robot.intake.intakeLeft.moveAtVoltage(0);
                    Robot.intake.intakeRight.moveAtVoltage(-0);
                    specialActionComplete = true;
                    break;
                case DRIVE_TO:
                case NONE:
                    //litterally do nothing
                    specialActionComplete = true;
                    break;
                default:
                    throw new UnsupportedOperationException("Cringe. You're unable to use the Special Action " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION.name() + " in your auton.");
            }

            switch (autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION_2) {
                case WAIT500:
                    specialActionComplete2 = timer.advanceIfElapsed(.5);
                    break;
                case ARM_ELEVATOR_UP:
                    Robot.elevator.elevate.moveAtPosition(2.2);
                    specialActionComplete2 = timer.advanceIfElapsed(.1);
                    System.out.println("tring to move Elevator");
                    break;
                case ARM_ELEVATOR_GO_TO:
                    specialActionComplete2 = Robot.manipulationManager.goTo(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG2,autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG3);
                    break;
                case ARM_ELEVATOR_DOWN:
                    specialActionComplete2 = Robot.manipulationManager.goTo(0,-50);
                    System.out.println("tring to move Elevator");
                    break;
                case CUBE_TOP:
                    specialActionComplete2 = Robot.manipulationManager.goTo(-1,-226);
                    System.out.println("tring to move Elevator");
                    break;
                case ARM_ELEVATOR_RESET:
                    specialActionComplete2 = Robot.manipulationManager.goTo(0,0);
                    System.out.println("tring to move Elevator");
                    break;
                case INTAKE_PISTON_IN:
                    Robot.intake.intakeIn();
                    specialActionComplete2 = true;
                    break;
                case INTAKE_PISTON_OUT:
                    Robot.intake.intakeOut();
                    specialActionComplete2 = true;
                    break;
                case INTAKE_WHEEL_IN:
                    Robot.intake.intakeRight.moveAtVoltage(6);
                    Robot.intake.intakeLeft.moveAtVoltage(-6);
                    specialActionComplete2 = true;
                    break;
                case INTAKE_WHEEL_OUT:
                    Robot.intake.intakeRight.moveAtVoltage(-12);
                    Robot.intake.intakeLeft.moveAtVoltage(12);
                    specialActionComplete2 = true;
                    break;
                case INTAKE_WHEEL_OFF:
                    Robot.intake.intakeLeft.moveAtVoltage(0);
                    Robot.intake.intakeRight.moveAtVoltage(-0);
                    specialActionComplete2 = true;
                    break;
                case NONE:
                    //litterally do nothing
                    specialActionComplete2 = true;
                    System.out.println("No special action2");
                    break;
                default:
                    throw new UnsupportedOperationException("Cringe. You're unable to use the Special Action 2" + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION_2.name() + " in your auton.");
            }

            if (specialActionComplete && specialActionComplete2) {
                if (++autonPath.currentWaypoint < autonPath.WAYPOINTS.size()) {
                    isInTolerance = false;
                    //throw new IllegalStateException("Holy crap theres no way it worked. This is illegal");
                    Point b = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION);
                    setupNextPosition(b);
                    //attackPoint(b, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPEED);
                    specialActionComplete = false;
                    specialActionComplete2 = false;
                    ROT_PID.reset();
                } else {
                    onFinish();
                }
            }
        }
    }

    /**
     * "Attack" (drive towards) a point on the field.
     *
     * @param point           the {@link Point point on the field} to attack
     * @param speed           the speed at which to do it
     * @param permitSwiveling
     * @return true if point has been attacked
     */
    public boolean attackPoint(Point point, double speed, boolean permitSwiveling) {
        UserInterface.smartDashboardPutString("Location", point.toString());
        Point here = new Point(drivingChild.guidance.fieldX(), drivingChild.guidance.fieldY());
        boolean angleTolerance = Math.abs(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG - drivingChild.guidance.swerveRobotPose.getEstimatedPosition().getRotation().getDegrees()) <= (robotSettings.AUTON_TOLERANCE *5.0);
        boolean inTolerance = here.isWithin(robotSettings.AUTON_TOLERANCE * 2.5, point);
        if (Math.abs(drivingChild.guidance.imu.absoluteRoll()) >= 1) {
            inTolerance = here.isWithin(robotSettings.AUTON_TOLERANCE * 4.5, point);
        }
        UserInterface.smartDashboardPutNumber("how far from correct the angle is", Math.abs(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG - drivingChild.guidance.swerveRobotPose.getEstimatedPosition().getRotation().getDegrees()));
        UserInterface.smartDashboardPutBoolean("angle tolerance", angleTolerance);
        UserInterface.smartDashboardPutBoolean("inTolerance", inTolerance);
        if (point.X <= -9000 && point.Y <= -9000) {
            inTolerance = true;
            angleTolerance = true;
        }
        UserInterface.smartDashboardPutNumber("rotOffset", -rotationOffset);
        UserInterface.smartDashboardPutString("Current Position", here.toString());
        if (!(inTolerance && angleTolerance)) {
            if (permitSwiveling) {
                double x = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.X - drivingChild.guidance.fieldX();
                double y = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.Y - drivingChild.guidance.fieldY();
                double pythag = Math.sqrt((x * x) + (y * y));
                double targetHeading = speed < 0 ? drivingChild.guidance.realRetrogradeHeadingError(x, y) : drivingChild.guidance.realHeadingError(x, y);
                System.out.println("demanded X " + x + " Demanded Y " + y);
                System.out.println("Pidgeon Angle: " + drivingChild.guidance.imu.relativeYaw());
                double calcX = X_PID.calculate(x);
                double calcY = Y_PID.calculate(y);
                UserInterface.smartDashboardPutNumber("trying to turn to", autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG);
                UserInterface.smartDashboardPutNumber("change in angle", autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG - drivingChild.guidance.swerveRobotPose.getEstimatedPosition().getRotation().getDegrees());
                UserInterface.smartDashboardPutNumber("roation stick input", ROT_PID.calculate(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG - drivingChild.guidance.imu.relativeYaw()));
                UserInterface.smartDashboardPutNumber("X direction stick input", robotSettings.AUTO_SPEED * speed * calcX);
                UserInterface.smartDashboardPutNumber("Y direction stick input", robotSettings.AUTO_SPEED * speed * calcY);
                double rotation = 0;
                double signNeeded = 1;
                double at = drivingChild.guidance.swerveRobotPose.getEstimatedPosition().getRotation().getDegrees();
                double goal = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG;

                rotation = UtilFunctions.mathematicalMod((goal - at) + 180, 360) - 180;

                if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
                    drivingChild.drivePure(-robotSettings.AUTO_SPEED * speed * calcX, robotSettings.AUTO_SPEED * speed * calcY, rotation * signNeeded * .0675);
                }else {
                    drivingChild.drivePure(-robotSettings.AUTO_SPEED * speed * calcX, robotSettings.AUTO_SPEED * speed * calcY, rotation * signNeeded * .0675/*-ROT_PID.calculate(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG - drivingChild.guidance.imu.relativeYaw())*/);
                }
            } else {
                double x = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).X;
                double y = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).Y;
                double targetHeading = speed < 0 ? drivingChild.guidance.realRetrogradeHeadingError(x, y) : drivingChild.guidance.realHeadingError(x, y);
                System.out.println("demanded X " + x + " Demanded Y " + y);
                //drivingChild.drivePure(robotSettings.AUTO_SPEED * speed /* (robotSettings.INVERT_DRIVE_DIRECTION ? -1 : 0)*/, ROT_PID.calculate(targetHeading) * -robotSettings.AUTO_ROTATION_SPEED);
            }
        } else {
            if (autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION != AutonSpecialActions.AUTO_LEVEL) {
                if (permitSwiveling) {
                    drivingChild.drivePure(0, 0, 0);
                } else {

                    drivingChild.drivePure(0, 0);

                }
            }
            System.out.println("Robot Shouldnt be moving");
            if (robotSettings.DEBUG)
                System.out.println("In tolerance.");
            //System.out.println("Driving FPS " + 0);
        }
        if (autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION_2 == AutonSpecialActions.ARM_ELEVATOR_GO_TO){
            Robot.manipulationManager.goTo(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG2,autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG3);
        }
        return inTolerance && angleTolerance;
    }

    /**
     * Saves a value with the current position and how far the robot has to turn to reach the next point
     *
     * @param point the {@link Point} (x, y) to go to
     */
    public void setupNextPosition(Point point) {
        yawBeforeTurn = drivingChild.guidance.imu.relativeYaw();
        rotationOffset = drivingChild.guidance.angleFromHere(point.X, point.Y);
    }

    /**
     * When the path finishes, we have flags to set, brakes to prime, and music to jam to
     */
    public void onFinish() {
        robotSettings.autonComplete = true;
        if (robotSettings.ENABLE_MUSIC && !robotSettings.AUTON_COMPLETE_NOISE.equals("")) {
            drivingChild.setBrake(true);
            Robot.chirp.loadMusic(robotSettings.AUTON_COMPLETE_NOISE);
            Robot.chirp.play();
        }
    }

    @Override
    public void initAuton() {
        robotSettings.autonComplete = false;
        drivingChild.setBrake(true);
        if (robotSettings.ENABLE_IMU) {
            drivingChild.guidance.resetOdometry();
            drivingChild.guidance.imu.resetOdometry();
        }
        timer.stop();
        timer.reset();
        timer.start();
        autonPath.currentWaypoint = 0;
        try {
            autonPath = AutonRoutines.myChooser.getSelected();
        }catch (Exception ignored){

        }
        drivingChild.guidance.setSwerveOdometryCurrent(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.X, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.Y, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG);
    }

    public void printAutonPositions(){
        UserInterface.smartDashboardPutNumber("rotOffset", -rotationOffset);
        UserInterface.smartDashboardPutString("Current Position", autonManager.toString());
    }

    @Override
    public String getSubsystemName() {
        return "PointToPoint Auton Manager";
    }
}
