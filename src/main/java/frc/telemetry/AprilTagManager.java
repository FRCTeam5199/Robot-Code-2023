package frc.telemetry;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.apriltag.AprilTagPoseEstimate.*;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerStandard;
import frc.drive.DriveManagerSwerve;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.robot.Robot;
import org.photonvision.*;
import edu.wpi.first.math.*;
import frc.telemetry.RobotTelemetrySwivel;

import org.photonvision.RobotPoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.sensors.camera.GoalPhoton;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static java.util.Timer.*;


public class AprilTagManager implements ISubsystem {
    Field2d fieldsim = new Field2d();
    Translation2d t2d = new Translation2d();
    RobotTelemetrySwivel swiveltelem;
    Rotation2d initrotate = new Rotation2d(0);
    AbstractDriveManager driver;
    PhotonCamera photonCamera1;
    PhotonCamera photonCamera2;
    PhotonCamera photonCamera3;
    PhotonCamera photonCamera4;

    ArrayList<SwerveModulePosition> swervearray;

    Translation2d wheels = new Translation2d(10.5, 12.5);


    static final Transform3d campos1 = new Transform3d(new Translation3d(18*(0.0254), -2*(0.0254), 0), new Rotation3d(0, 0, 0));
    static final Transform3d campos2 = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    static final Transform3d campos3 = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    static final Transform3d campos4 = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));



    // Everything to do with the april tag and Fields is in inches.
    double fieldlength = 651.25;
    double fieldwidth = 315.5;

    Pose2d initPose = new Pose2d(0, 0, initrotate);
    Pose2d currentpose;

    ArrayList<AprilTag> ApriList;

    AprilTagFieldLayout fieldLayout;
    ArrayList<Pair<PhotonCamera, Transform3d>> cams;
    ArrayList<PhotonCamera> cameras;
    RobotPoseEstimator.PoseStrategy poseStrategy;
    RobotPoseEstimator robotPoseEstimator;
    SwerveDriveKinematics swervekin;


    public AprilTagManager(AbstractDriveManager driver) {
        addToMetaList();
        if (driver instanceof DriveManagerSwerve)
            this.driver = driver;
        init();
    }

    @Override
    public void init() {
        final AprilTag tag1 = new AprilTag(1, new Pose3d(610.77*(0.0254), 42.19*0.0254, 18.228*(0.0254), new Rotation3d(0, 0, 180)));
        final AprilTag tag2 = new AprilTag(2, new Pose3d(610.77*(0.0254), 108.19*(0.0254), 18.22*(0.0254), new Rotation3d(0, 0, 180)));
        final AprilTag tag3 = new AprilTag(3, new Pose3d(610.77*(0.0254), 147.19*(0.0254), 18.22*(0.0254), new Rotation3d(0, 0, 180)));
        final AprilTag tag4 = new AprilTag(4, new Pose3d(636.96*(0.0254), 265.74*(0.0254), 27.38*(0.0254), new Rotation3d(0, 0, 180)));
        final AprilTag tag5 = new AprilTag(5, new Pose3d(14.25*(0.0254), 265.74*(0.0254), 27.38*(0.0254), new Rotation3d(0, 0, 0)));
        final AprilTag tag6 = new AprilTag(6, new Pose3d(40.45*(0.0254), 147.19*(0.0254), 18.22*(0.0254), new Rotation3d(0, 0, 0)));
        final AprilTag tag7 = new AprilTag(7, new Pose3d(40.45*(0.0254), 108.19*(0.0254), 18.22*(0.0254), new Rotation3d(0, 0, 0)));
        final AprilTag tag8 = new AprilTag(8, new Pose3d(40.45*(0.0254), 42.19*(0.0254), 18.22*(0.0254), new Rotation3d(0, 0, 0)));
        ApriList = new ArrayList<AprilTag>();
        ApriList.add(tag1);
        ApriList.add(tag2);
        ApriList.add(tag3);
        ApriList.add(tag4);
        ApriList.add(tag5);
        ApriList.add(tag6);
        ApriList.add(tag7);
        ApriList.add(tag8);

        photonCamera1 = new PhotonCamera("Global_Shutter_Camera");
        if(Robot.robotSettings.FOUR_CAMERA) {
            photonCamera2 = new PhotonCamera("back");
            photonCamera3 = new PhotonCamera("left");
            photonCamera4 = new PhotonCamera("right");
        }
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        }catch(IOException e) {
            fieldLayout = new AprilTagFieldLayout(ApriList, fieldlength, fieldwidth);
        }

        cams = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        cams.add(new Pair<>(photonCamera1, campos1));
        if(Robot.robotSettings.FOUR_CAMERA) {
            cams.add(new Pair<>(photonCamera2, campos2));
            cams.add(new Pair<>(photonCamera3, campos3));
            cams.add(new Pair<>(photonCamera4, campos4));
        }
        SwerveModulePosition swervemodpos1 = new SwerveModulePosition(16.43, initrotate);
        SwerveModulePosition swervemodpos2 = new SwerveModulePosition(16.43, initrotate);
        SwerveModulePosition swervemodpos3 = new SwerveModulePosition(16.43, initrotate);
        SwerveModulePosition swervemodpos4 = new SwerveModulePosition(16.43, initrotate);

        swervearray = new ArrayList<SwerveModulePosition>();
        swervearray.add(swervemodpos1);
        swervearray.add(swervemodpos2);
        swervearray.add(swervemodpos3);
        swervearray.add(swervemodpos4);


        swervekin = ((DriveManagerSwerve)driver).getKinematics();
        poseStrategy = RobotPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
        robotPoseEstimator = new RobotPoseEstimator(fieldLayout, poseStrategy, cams);

    }


    @Override
    public SubsystemStatus getSubsystemStatus() {
        return SubsystemStatus.NOMINAL;
    }

    @Override
    public void updateTest() {

    }


    @Override
    public void updateTeleop() {
        updateGeneric();

    }

    @Override
    public void updateAuton() {
        updateGeneric();
    }

    @Override
    public void updateGeneric() {
        UserInterface.smartDashboardPutNumber("Apriltag Estimate X Pose: ",getEstimatedGlobalPose().getFirst().getX());
        UserInterface.smartDashboardPutNumber("Apriltag Estimate Y Pose: ",getEstimatedGlobalPose().getFirst().getY());


    }
    public Pair<Pose2d, Double> getEstimatedGlobalPose() {
        robotPoseEstimator.setReferencePose(new Pose2d());

        double currentTime = getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();

        if (result.isEmpty()) {
            return new Pair<Pose2d, Double>(new Pose2d(-2,-2,new Rotation2d(0)), 0.0);
        } else {
            //return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());


            return new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(), 0.0);
        }
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
        return "April tag manager";
    }
}
