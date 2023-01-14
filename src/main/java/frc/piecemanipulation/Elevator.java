package frc.piecemanipulation;

import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;

import static frc.robot.Robot.robotSettings;

public class Elevator implements ISubsystem {
    public AbstractMotorController elevate;
    public BaseController xbox;

    public Elevator(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        createMotors();
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

    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {
        if (robotSettings.ELEVATOR_MANUAL)
        manuelDrive();
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
        if(robotSettings.ELEVATOR_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            elevate = new TalonMotorController(robotSettings.ELEVATOR_MOTOR_ID, robotSettings.ELEVATOR_MOTOR_CANBUS);
        if(robotSettings.ELEVATOR_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            elevate = new SparkMotorController(robotSettings.ELEVATOR_MOTOR_ID);
        elevate.setRealFactorFromMotorRPM(robotSettings.ELEVATOR_GEARING * (robotSettings.ELEVATOR_SPROCKET_DIAMETER * Math.PI / 12), 1/60D );
    }

    public void manuelDrive(){
        if(xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN){
            elevate.moveAtVelocity(1);
        }else if(xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            elevate.moveAtVelocity(-1);
        }else{
            elevate.moveAtVelocity(0);
        }
        System.out.println(elevate.getRotations());
    }
}
