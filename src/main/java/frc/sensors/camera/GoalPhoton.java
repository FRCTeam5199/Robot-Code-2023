package frc.sensors.camera;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.misc.SubsystemStatus;
import org.photonvision.PhotonCamera;

import static frc.robot.Robot.robotSettings;

public class GoalPhoton implements IVision {
    public static final GoalPhoton GOAL_PHOTON = new GoalPhoton();
    public static PhotonCamera photonCamera;
    private NetworkTableEntry yaw;
    private NetworkTableEntry size;
    private NetworkTableEntry hasTarget;
    private NetworkTableEntry pitch;
    private NetworkTableEntry pose;
    private LinearFilter filter;
    private NetworkTable photon;

    /**
     * inits GoalPhoton
     */
    public GoalPhoton() {
        addToMetaList();
        init();
    }

    /**
     * stores values in simpler variable names
     */
    @Override
    public void init() {
        filter = LinearFilter.movingAverage(5);
        photon = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable(robotSettings.GOAL_CAM_NAME);
        yaw = photon.getEntry("targetYaw");
        size = photon.getEntry("targetArea");
        hasTarget = photon.getEntry("hasTarget");
        pitch = photon.getEntry("targetPitch");
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.FAILED;
    }

    /**
     * calls updateGeneric see GoalPhoton.updateGeneric
     */
    @Override
    public void updateTest() {
        updateGeneric();
    }

    /**
     * calls updateGeneric see GoalPhoton.updateGeneric
     */
    @Override
    public void updateTeleop() {
        updateGeneric();
    }

    /**
     *
     */
    @Override
    public void updateAuton() {
    }

    /**
     * updates generic things for GoalPhoton
     */
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

    /**
     * Get angle between crosshair and goal left/right with filter calculation.
     *
     * @return angle between crosshair and goal, left negative, 29.8 degrees in both directions.
     */
    @Override
    public double getAngleSmoothed(int channelIgnored) {
        double angle = yaw.getDouble(0);
        if (hasValidTarget()) {
            return filter.calculate(angle);
        }
        return 0;
    }

    @Override
    public void setPipeline(int pipeline) {
        if (pipeline >= 0 && pipeline <= 9) {
            photon.getEntry("pipeline").setNumber(pipeline);
        }
    }

    /**
     * Check for a valid target in the camera's view.
     *
     * @return whether or not there is a valid target in view.
     */
    @Override
    public boolean hasValidTarget() {
        return hasTarget.getBoolean(false);
    }

    /**
     * Get angle between crosshair and goal left/right.
     *
     * @return angle between crosshair and goal, left negative, 29.8 degrees in both directions.
     */
    @Override
    public double getAngle(int channelIgnored) {
        double angle = yaw.getDouble(0);
        if (hasValidTarget()) {
            return angle;
        }
        return 0;
    }

    /**
     * Get angle between crosshair and goal up/down.
     *
     * @return angle between crosshair and goal, down negative, 22 degrees in both directions.
     */
    @Override
    public double getPitch(int channelIgnored) {
        double angle = pitch.getDouble(0);
        if (hasValidTarget()) {
            return angle;
        }
        return 0;
    }

    /**
     * Get the size of the goal onscreen.
     *
     * @return size of the goal in % of the screen, 0-100.
     */
    @Override
    public double getSize(int channelIgnored) {
        double goalSize = size.getDouble(0);
        if (hasValidTarget()) {
            return goalSize;
        }
        return 0;
    }

    @Override
    public void setLedMode(VisionLEDMode ledMode) {
        //throw new UnsupportedOperationException("Cannot set LED mode " + ledMode.name() + " on " + getSubsystemName());
    }
}