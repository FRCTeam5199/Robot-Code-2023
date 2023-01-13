package frc.drive.auton.pointtopoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.drive.AbstractDriveManager;
import frc.drive.auton.AbstractAutonManager;
import frc.drive.auton.Point;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.robot.Robot;

import static frc.robot.Robot.robotSettings;

public class AutonManager extends AbstractAutonManager {
    public final Timer timer = new Timer();
    private final PIDController ROT_PID;
    public AbstractDriveManager drivingChild;
    public AutonRoutines autonPath;
    public boolean specialActionComplete = false;
    public double yawBeforeTurn = 0, rotationOffset = 0.01;
    private boolean isInTolerance = false;
    private boolean isAiming = false;

    public AutonManager(AutonRoutines routine, AbstractDriveManager driveManager) {
        super(driveManager);
        addToMetaList();
        drivingChild = driveManager;
        autonPath = routine;
        ROT_PID = new PIDController(robotSettings.HEADING_PID.P, robotSettings.HEADING_PID.I, robotSettings.HEADING_PID.D);
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
        Point point = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION);
        if (attackPoint(point, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPEED, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION == AutonSpecialActions.DRIVE_TO && autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG != 0) || isInTolerance) {
            isInTolerance = true;
            DriverStation.reportWarning("IN TOLERANCE", false);
            System.out.println("Special Action: " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION.toString());
            switch (autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION) {
                case NONE:
                    //litterally do nothing
                    specialActionComplete = true;
                    break;
                default:
                    throw new UnsupportedOperationException("Cringe. You're unable to use the Special Action " + autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPECIAL_ACTION.name() + " in your auton.");
            }
            if (specialActionComplete) {
                if (++autonPath.currentWaypoint < autonPath.WAYPOINTS.size()) {
                    isInTolerance = false;
                    //throw new IllegalStateException("Holy crap theres no way it worked. This is illegal");
                    Point b = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION);
                    setupNextPosition(b);
                    //attackPoint(b, autonPath.WAYPOINTS.get(autonPath.currentWaypoint).SPEED);
                    specialActionComplete = false;
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
        Point here = new Point(drivingChild.guidance.fieldX(), -drivingChild.guidance.fieldY());

        boolean inTolerance = here.isWithin(robotSettings.AUTON_TOLERANCE * 3, point);
        if (point.X <= -9000 && point.Y <= -9000)
            inTolerance = true;
        UserInterface.smartDashboardPutNumber("rotOffset", -rotationOffset);
        UserInterface.smartDashboardPutString("Current Position", here.toString());
        if (!inTolerance) {
            if (permitSwiveling) {
                double x = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).X;
                double y = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).Y;
                double targetHeading = speed < 0 ? drivingChild.guidance.realRetrogradeHeadingError(x, y) : drivingChild.guidance.realHeadingError(x, y);

                drivingChild.drivePure(robotSettings.AUTO_SPEED * speed * Math.cos(Math.toRadians(targetHeading)), robotSettings.AUTO_SPEED * speed * Math.sin(Math.toRadians(targetHeading)), ROT_PID.calculate(autonPath.WAYPOINTS.get(autonPath.currentWaypoint).INTARG - drivingChild.guidance.imu.relativeYaw()));
            } else {
                double x = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).X;
                double y = autonPath.WAYPOINTS.get(autonPath.currentWaypoint).LOCATION.subtract(autonPath.WAYPOINTS.get(0).LOCATION).Y;
                double targetHeading = speed < 0 ? drivingChild.guidance.realRetrogradeHeadingError(x, y) : drivingChild.guidance.realHeadingError(x, y);

                drivingChild.drivePure(robotSettings.AUTO_SPEED * speed /* (robotSettings.INVERT_DRIVE_DIRECTION ? -1 : 0)*/, ROT_PID.calculate(targetHeading) * -robotSettings.AUTO_ROTATION_SPEED);
            }
        } else {
            drivingChild.drivePure(0, 0);
            if (robotSettings.DEBUG)
                System.out.println("In tolerance.");
            //System.out.println("Driving FPS " + 0);
        }
        return inTolerance;
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
    }

    @Override
    public String getSubsystemName() {
        return "PointToPoint Auton Manager";
    }
}
