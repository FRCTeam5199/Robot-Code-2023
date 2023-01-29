package frc.piecemanipulation;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;

import static frc.robot.Robot.robotSettings;

public class Wrist implements ISubsystem {
    public AbstractMotorController wrist;
    public BaseController panel, xbox2;

    public Wrist(){
        addToMetaList();
        init();
    }
    @Override
    public void init() {
        createControllers();
        createMotors();
        wrist.setPid(robotSettings.WRISTPID);
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
        if(robotSettings.ARM_ELEVATOR_MANUAL){
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_3) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtVoltage(-2);
            }
            else if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtVoltage(2);
            }
            else {
                wrist.moveAtVoltage(0);
            }
        }else {
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_3) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtPosition(0);
            }
            else if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                wrist.moveAtPosition(5);
            }
        }
        System.out.println(wrist.getRotations());
        if (xbox2.get(DefaultControllerEnums.XBoxButtons.RIGHT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN)
            wrist.resetEncoder();
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

    public void createMotors(){
        if(robotSettings.WRIST_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            wrist = new TalonMotorController(robotSettings.WRIST_MOTOR_ID, robotSettings.WRIST_MOTOR_CANBUS);
        if(robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            wrist = new SparkMotorController(robotSettings.WRIST_MOTOR_ID);
        //arm.setOutPutRange(-.8,.8);
        wrist.setCurrentLimit(40);
        wrist.setBrake(true);
    }

    public void createControllers(){
        panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.DefaultControllers.BUTTON_PANEL);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
    }
}
