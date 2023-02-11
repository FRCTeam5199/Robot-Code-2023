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
import frc.robot.Robot;

import static frc.robot.Robot.*;


public class Intake implements ISubsystem {
    public AbstractMotorController intakeLeft, intakeRight;
    private BaseController xbox, panel;

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
        intakeLeft.setCurrentLimit(20);
        intakeRight.setCurrentLimit(20);
        intakeRight.setBrake(false);
        intakeLeft.setBrake(false);
    }

    public void createControllers(){
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void manuelDrive(){
        if(!manipulationManager.cubeConeMode) {
            if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                //System.out.println("X is being pressed");
                intakeRight.moveAtVoltage(12);
                intakeLeft.moveAtVoltage(-12);
            } else if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
                //System.out.println("Y is being pressed");
                intakeRight.moveAtVoltage(-12);
                intakeLeft.moveAtVoltage(12);
            } else {
                intakeRight.moveAtVoltage(0);
                intakeLeft.moveAtVoltage(0);
            }
        }
        if(manipulationManager.cubeConeMode){
            //System.out.println(elevate.getRotations());
            intakeRight.moveAtVoltage(0);
            intakeLeft.moveAtVoltage(0);
            if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
            }
            if (xbox.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
            }
        }
        /*
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.FENDER_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN){
            Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
        }
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.LOW_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN) {
            Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
        }
        */
    }
    public void intakeIn(){
        Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kReverse);
    }
    public void intakeOut(){
        Robot.pneumatics.intakePiston.set(DoubleSolenoid.Value.kForward);
    }
}