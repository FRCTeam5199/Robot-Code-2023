package frc.piecemanipulation;

import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.PID;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;

import static frc.robot.Robot.robotSettings;

public class Arm implements ISubsystem {
    public AbstractMotorController arm;
    public BaseController xbox;

    public Arm(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        createMotors();
        createMotorPid(robotSettings.ARM_PID);
        arm.setBrake(true);
    }

    @Override
    public SubsystemStatus getSubsystemStatus() {
        return null;
    }

    @Override
    public void updateTest() {

    }

    @Override
    public void updateTeleop() {
        if(robotSettings.ARM_MANUAL)
            manuelDrive();
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
        return null;
    }

    public void createControllers(){
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
    }

    public void createMotors(){
        if(robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            arm = new TalonMotorController(robotSettings.ARM_MOTOR_ID, robotSettings.ARM_MOTOR_CANBUS);
        if(robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            arm = new SparkMotorController(robotSettings.ARM_MOTOR_ID);
        arm.setRealFactorFromMotorRPM(robotSettings.ARM_GEARING * (robotSettings.ARM_SPROCKET_DIAMETER * Math.PI / 12), 1/60D );
    }
    public void createMotorPid(PID pid){
        arm.setPid(pid);
    }

    public void manuelDrive(){
        if(xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN){
            //System.out.println("X is being pressed");
            arm.moveAtVelocity(.05);
        }else if(xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            //System.out.println("Y is being pressed");
            arm.moveAtVelocity(-.05);
        }else{
            arm.moveAtVelocity(0);
        }
        //System.out.println(arm.getRotations());
    }

}
