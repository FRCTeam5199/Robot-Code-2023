package frc.telemetry;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.drive.AbstractDriveManager;
import frc.drive.DriveManagerSwerve;
import frc.misc.UserInterface;

import static frc.robot.Robot.robotSettings;

/**
 * Telemetry specially designed to be compatible with {@link DriveManagerSwerve a swivel drivetrain} by using {@link
 * frc.motors.SwerveMotorController swivel controllers}
 */
public class RobotTelemetrySwivel extends AbstractRobotTelemetry {
    /**
     * The reason there cant be a universal RobotTelem. needs {@link edu.wpi.first.math.kinematics.SwerveModuleState
     * module states} in order to maintin track of current pos
     */
    private SwerveDriveOdometry odometer;
    private double aprilAvg;
    protected RobotTelemetrySwivel(AbstractDriveManager driver) {
        super(driver);
        if (!(driver instanceof DriveManagerSwerve))
            throw new IllegalArgumentException("Nope");
    }

    @Override
    public void init() {
        aprilAvg = 0;
        super.init();
        if (imu != null) {
            resetSwerveOdometryandCreate();
        }

    }

    @Override
    public void updateGeneric() {
        UserInterface.smartDashboardPutNumber("swerve robot pose yaw", swerveRobotPose.getEstimatedPosition().getRotation().getDegrees());
        if(robotSettings.ENABLE_APRILTAG){
            Pair<Pose2d, Double> apriltagpos = tagManager.getEstimatedGlobalPose();
            if(apriltagpos.getFirst().getX() == -2 && apriltagpos.getFirst().getY() == -2) {
              //nothing
            }else {
                try {

                    double AprilX = -54 + (apriltagpos.getFirst().getX() * 3.28);
                    double AprilY = apriltagpos.getFirst().getY() * 3.28;
                    UserInterface.smartDashboardPutNumber("Always April field Y", AprilY);
                   /*  if(AprilX <= -16 && AprilX  >= -32 ){
                        AprilY = swerveRobotPose.getEstimatedPosition().getY();
                    } */
                    if(aprilAvg == 0){
                        aprilAvg = AprilY;
                    }else{
                        aprilAvg = aprilAvg *.9 + AprilY*(1.0/10);
                    }
                    Translation2d translation2dft = new Translation2d(AprilX, aprilAvg);
                    Pose2d poseinft = new Pose2d(translation2dft, apriltagpos.getFirst().getRotation());
                    UserInterface.smartDashboardPutNumber("April Field X", translation2dft.getX());
                    UserInterface.smartDashboardPutNumber("Sometimes April Field Y", translation2dft.getY());
                    UserInterface.smartDashboardPutNumber("apriltag get second", apriltagpos.getSecond());
                    double timernow = Timer.getFPGATimestamp() - apriltagpos.getSecond();
                    UserInterface.smartDashboardPutNumber("timer now", timernow);
                    swerveRobotPose.addVisionMeasurement(poseinft, timernow);
                }catch (Exception e){
                    System.out.println(e);
                }
            }
        }
        if (robotSettings.ENABLE_IMU) {
            if (driver instanceof DriveManagerSwerve)
                swerveRobotPose.update(new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition());

            //robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), ((DriveManagerSwerve) driver).getModulePosition());

            super.updateGeneric();

        }
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
        updateGeneric();
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {

    }

    @Override
    public void initAuton() {
        resetSwerveOdometryandCreate();
    }

    @Override
    public void initDisabled() {

    }

    @Override
    public void initGeneric() {

    }

    public void resetSwerveOdometryandCreate(){
        if(driver instanceof DriveManagerSwerve) {
            //robotPose = odometer.update(new Rotation2d(Units.degreesToRadians(imu.absoluteYaw())), ((DriveManagerSwerve) driver).getModulePosition());
            if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                odometer = new SwerveDriveOdometry(((DriveManagerSwerve) driver).getKinematics(), Rotation2d.fromDegrees(imu.relativeYaw()), ((DriveManagerSwerve) driver).getModulePosition());
                swerveRobotPose = new SwerveDrivePoseEstimator(((DriveManagerSwerve) driver).getKinematics(), new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d());
                odometer.resetPosition(new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(0, 0, new Rotation2d(0)));
                swerveRobotPose.resetPosition(new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(0, 0, new Rotation2d(0)));
            }
            if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
                odometer = new SwerveDriveOdometry(((DriveManagerSwerve) driver).getKinematics(), Rotation2d.fromDegrees(imu.relativeYaw()), ((DriveManagerSwerve) driver).getModulePosition());
                swerveRobotPose = new SwerveDrivePoseEstimator(((DriveManagerSwerve) driver).getKinematics(), new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(0, 0, new Rotation2d(Math.PI)));
                odometer.resetPosition(new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(0, 0, new Rotation2d(Math.PI)));
                swerveRobotPose.resetPosition(new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(0, 0, new Rotation2d(Math.PI)));
            }
        }
    }

    @Override
    public void setSwerveOdometryCurrent(double currentX, double currentY){
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            this.setSwerveOdometryCurrent(currentX, currentY, Math.PI);
        }else{
            this.setSwerveOdometryCurrent(currentX, currentY, 0);
        }
    }

    @Override
    public void setSwerveOdometryCurrent(double currentX, double currentY, double angle){
        if(DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            odometer = new SwerveDriveOdometry(((DriveManagerSwerve) driver).getKinematics(), Rotation2d.fromDegrees(imu.relativeYaw()), ((DriveManagerSwerve) driver).getModulePosition());
            swerveRobotPose = new SwerveDrivePoseEstimator(((DriveManagerSwerve) driver).getKinematics(), new Rotation2d(Units.degreesToRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(currentX, currentY, new Rotation2d(0)));
            odometer.resetPosition(new Rotation2d(Math.toRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(currentX, currentY, new Rotation2d(0)));
            swerveRobotPose.resetPosition(new Rotation2d(Math.toRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(currentX, currentY, new Rotation2d(0)));
        }
        if(DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            odometer = new SwerveDriveOdometry(((DriveManagerSwerve) driver).getKinematics(), Rotation2d.fromDegrees(imu.relativeYaw()), ((DriveManagerSwerve) driver).getModulePosition());
            swerveRobotPose = new SwerveDrivePoseEstimator(((DriveManagerSwerve) driver).getKinematics(), new Rotation2d(imu.relativeYaw()), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(currentX, currentY, new Rotation2d(Math.PI)));
            odometer.resetPosition(new Rotation2d((Math.toRadians(imu.relativeYaw()))), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(currentX, currentY, new Rotation2d(Math.PI)));
            swerveRobotPose.resetPosition(new Rotation2d(Math.toRadians(imu.relativeYaw())), ((DriveManagerSwerve) driver).getModulePosition(), new Pose2d(currentX, currentY, new Rotation2d(Math.PI)));
        }

    }

}
