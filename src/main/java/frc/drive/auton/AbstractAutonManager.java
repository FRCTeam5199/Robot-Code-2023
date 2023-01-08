package frc.drive.auton;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.robot.Robot;

import static frc.robot.Robot.robotSettings;

/**
 * If you have a custom auton that needs to be implemented, extend this class. Since every Auton Manager needs to have a
 * {@link DriveManagerStandard drivetrain} and a {@link Timer timer}, they are here along with {@link ISubsystem}.
 *
 * @see ISubsystem
 */
public abstract class AbstractAutonManager implements ISubsystem {
    protected final Timer timer = new Timer();
    protected final AbstractDriveManager DRIVING_CHILD;
    protected final RamseteController controller = new RamseteController();
    protected Trajectory trajectory;

    /**
     * Initializes the auton manager and stores the reference to the drivetrain object
     *
     * @param driveManager the drivetrain object created for the robot
     */
    protected AbstractAutonManager(AbstractDriveManager driveManager) {
        addToMetaList();
        DRIVING_CHILD = driveManager;
        init();
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return DRIVING_CHILD.getSubsystemStatus() == SubsystemStatus.NOMINAL && DRIVING_CHILD.guidance.getSubsystemStatus() == SubsystemStatus.NOMINAL ? SubsystemStatus.NOMINAL : SubsystemStatus.FAILED;
    }

    /**
     * Runs the auton path. When complete, sets a flag in {@link frc.robot.robotconfigs.DefaultConfig#autonComplete} and
     * runs {@link #onFinish()}
     */
    @Override
    public void updateAuton() {
        if (!robotSettings.autonComplete) {
            Trajectory.State goal = trajectory.sample(timer.get());
            if (robotSettings.ENABLE_IMU) {
                System.out.println("I am currently at (" + DRIVING_CHILD.guidance.fieldX() + "," + DRIVING_CHILD.guidance.fieldY() + ")\nI am going to (" + goal.poseMeters.getX() + "," + goal.poseMeters.getY() + ")");
                if (robotSettings.DRIVE_BASE == AbstractDriveManager.DriveBases.STANDARD)
                    DRIVING_CHILD.driveWithChassisSpeeds(controller.calculate(DRIVING_CHILD.guidance.robotPose, goal));
                else
                    DRIVING_CHILD.driveWithChassisSpeeds(controller.calculate(DRIVING_CHILD.guidance.robotPose, goal)); //might need to be different
            }
            if (timer.get() > trajectory.getTotalTimeSeconds()) {
                onFinish();
            }
        }
    }

    /**
     * When the path finishes, we have flags to set, brakes to prime, and music to jam to
     */
    protected void onFinish() {
        robotSettings.autonComplete = true;
        if (robotSettings.ENABLE_MUSIC && !robotSettings.AUTON_COMPLETE_NOISE.equals("")) {
            DRIVING_CHILD.setBrake(true);
            Robot.chirp.loadMusic(robotSettings.AUTON_COMPLETE_NOISE);
            Robot.chirp.play();
        }
    }

    /**
     * On enable, unset finished flag, and prime the path and reset the timer
     */
    @Override
    public void initAuton() {
        robotSettings.autonComplete = false;
        if (robotSettings.ENABLE_IMU) {
            DRIVING_CHILD.guidance.resetOdometry();
            if (trajectory != null) {
                Transform2d transform = DRIVING_CHILD.guidance.robotPose.minus(trajectory.getInitialPose());
                trajectory = trajectory.transformBy(transform);
            }
        }
        timer.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public String getSubsystemName() {
        return "Auton manager";
    }
}