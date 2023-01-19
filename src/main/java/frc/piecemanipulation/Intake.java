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
    public AbstractMotorController intake;
    public BaseController xbox;
    Pneumatics pneumatics;
    DoubleSolenoid solenoidIntake;
    DoubleSolenoid subSolenoidIntake;

    public void Intake() {
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

    public void createPneumatics(){
        solenoidIntake = pneumatics.solenoidIntake;
        subSolenoidIntake = pneumatics.subsolenoidIntake;

    }
    public void createMotors(){
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            intake = new TalonMotorController(robotSettings.INTAKE_MOTOR_ID, robotSettings.INTAKE_MOTOR_CANBUS);
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            intake = new SparkMotorController(robotSettings.INTAKE_MOTOR_ID);
        if(robotSettings.INTAKE_MOTOR_TYPE == AbstractMotorController.SupportedMotors.VICTOR)
            intake = new VictorMotorController(robotSettings. INTAKE_MOTOR_ID);
    }

    public void createControllers(){
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
    }

    public void manuelDrive(){
        if(xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN){
            //System.out.println("X is being pressed");
            intake.moveAtVoltage(2);
        }else if(xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            //System.out.println("Y is being pressed");
            intake.moveAtVoltage(-2);
        }else{
            intake.moveAtVelocity(0);
        }
        //System.out.println(elevate.getRotations());

        if(xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN){
            solenoidIntake.set(DoubleSolenoid.Value.kForward);
        }
        if(xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN){
            solenoidIntake.set(DoubleSolenoid.Value.kReverse);
        }

        if(xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            solenoidIntake.set(DoubleSolenoid.Value.kForward);
        }
        if(xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            solenoidIntake.set(DoubleSolenoid.Value.kReverse);
        }
    }
}