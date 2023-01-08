package frc.sensors.camera;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.robot.Robot;

import static frc.robot.Robot.robotSettings;

/**
 * This is for the limelight looking at the goal that the shooter is shooting at
 */
public class GoalLimelight implements IVision {
    public static final GoalLimelight GOAL_LIME_LIGHT = new GoalLimelight();
    private NetworkTable limelight;
    private NetworkTableEntry yaw, size, hasTarget, pitch, pose;
    private LinearFilter filter;

    private GoalLimelight() {
        addToMetaList();
        init();
    }

    /**
     * creates all of the network table stuff
     */
    @Override
    public void init() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        filter = LinearFilter.movingAverage(5);
        yaw = limelight.getEntry("tx");
        size = limelight.getEntry("ta");
        hasTarget = limelight.getEntry("tv");
        pitch = limelight.getEntry("ty");
        pose = limelight.getEntry("camtran");
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.FAILED;
    }

    @Override
    public void updateTest() {
        updateGeneric();
    }

    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initAuton() {

    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    @Override
    public String getSubsystemName() {
        return "Goal Camera";
    }

    @Override
    public void setPipeline(int pipeline) {
        if (pipeline >= 0 && pipeline <= 9) {
            limelight.getEntry("pipeline").setNumber(pipeline);
        }
    }

    @Override
    public double getAngle(int channelIgnored) {
        if (hasValidTarget()) {
            return yaw.getDouble(0);
        } else {
            return 0;
        }
    }

    @Override
    public double getPitch(int channelIgnored) {
        if (hasValidTarget()) {
            return pitch.getDouble(0);
        } else {
            return 0;
        }
    }

    @Override
    public double getAngleSmoothed(int channelIgnored) {
        if (hasValidTarget()) {
            return filter.calculate(yaw.getDouble(0));
        } else {
            return 0;
        }
    }

    @Override
    public double getSize(int channelIgnored) {
        if (hasValidTarget()) {
            return size.getDouble(0);
        } else {
            return 0;
        }
    }

    @Override
    public void setLedMode(VisionLEDMode ledMode) {
        int setTo;
        switch (ledMode) {
            case ON:
                setTo = 3;
                break;
            case OFF:
                setTo = 1;
                break;
            case BLINK:
                setTo = 2;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + ledMode);
        }
        limelight.getEntry("ledMode").setNumber(setTo);

        if (robotSettings.ENABLE_TOGGLEABLE_RING && robotSettings.ENABLE_PDP) {
           Robot.pdp.setToggleable(setTo == 3 || setTo == 2);
        }
    }

    @Override
    public boolean hasValidTarget() {
        return (hasTarget.getDouble(0) == 1); //0 = not visible, 1 = visible
    }
}