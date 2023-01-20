package frc.piecemanipulation;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;
import frc.motors.VictorMotorController;
import frc.robot.robotconfigs.*;
import frc.telemetry.imu.AbstractIMU;
import frc.misc.Pneumatics;

import static frc.robot.Robot.robotSettings;


public class Intake implements ISubsystem {
    public AbstractMotorController intakeLeft, intakeRight;
    private BaseController xbox;

    public Intake() {
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
        updateGeneric();
    }

    @Override
    public void updateAuton() {

    }

    @Override
    public void updateGeneric() {
        if (robotSettings.INTAKE_MANUAL)
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

    public void createMotors(){
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX) {
            intakeLeft = new TalonMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID, robotSettings.INTAKE_MOTOR_CANBUS);
            intakeRight = new TalonMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID, robotSettings.INTAKE_MOTOR_CANBUS);
        }
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX) {
            intakeLeft = new SparkMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID);
            intakeRight = new SparkMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID);
        }
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.VICTOR) {
            intakeLeft = new VictorMotorController(robotSettings.INTAKE_MOTOR_LEFT_ID);
            intakeRight = new VictorMotorController(robotSettings.INTAKE_MOTOR_RIGHT_ID);
        }
    }

    public void createControllers(){
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
    }

    public void manuelDrive(){
        if(xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN){
            //System.out.println("X is being pressed");
            intakeRight.moveAtVoltage(-3);
            intakeLeft.moveAtVoltage(3);
        }else if(xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            //System.out.println("Y is being pressed");
            intakeRight.moveAtVoltage(3);
            intakeLeft.moveAtVoltage(-3);
        }else{
            intakeRight.moveAtVoltage(0);
            intakeLeft.moveAtVoltage(0);
        }
        //System.out.println(elevate.getRotations());
        /*
        if(xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN){
            pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
        }
        if(xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN){
            pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
        }

        if(xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
        }
        if(xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
        } */
    }
}