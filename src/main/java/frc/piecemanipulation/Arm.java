package frc.piecemanipulation;

import frc.controllers.ButtonPanelController;
import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.PID;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;

import static edu.wpi.first.wpilibj.RobotBase.suppressExitWarning;
import static frc.robot.Robot.robotSettings;

public class Arm implements ISubsystem {
    public AbstractMotorController arm;
    public BaseController xbox, xbox2, panel1, panel2, midiTop, midiBot;

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
        if(robotSettings.ARM_ELEVATOR_MANUAL) {
            manuelDrive();
        }else{
            if (!robotSettings.ENABLE_PIECE_MANAGER) {
                PositionDrive();
            }
        }
        if(xbox2.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN){
            resetArmEncoder();
        }
        System.out.println("Arm Position: " + arm.getRotations());
        System.out.println("Motor Power " + arm.getVoltage());
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
        arm.resetEncoder();
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
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void createMotors(){
        if(robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            arm = new TalonMotorController(robotSettings.ARM_MOTOR_ID, robotSettings.ARM_MOTOR_CANBUS);
        if(robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            arm = new SparkMotorController(robotSettings.ARM_MOTOR_ID);
        arm.setRealFactorFromMotorRPM(1, 1 );
        //arm.setOutPutRange(-.8,.8);
        arm.setCurrentLimit(40);

    }
    public void createMotorPid(PID pid){
        arm.setPid(pid);
    }


    public void resetArmEncoder(){
       // arm.resetEncoder();
    }

    public void manuelDrive(){
        if(Math.abs(xbox2.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_Y)) >= .1){
            arm.moveAtVoltage(xbox2.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_Y) * -12);
        }else {
            arm.moveAtVoltage(0);
        }
        //System.out.println("Manual Enabled");

    }
    public void PositionDrive(){
       /* if(panel.get(ControllerEnums.ButtonPanelButtons2022.FENDER_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN){
            arm.moveAtPosition(-1);
            //System.out.println("trying to go to zero");
        }
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.TARMAC_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN){
            arm.moveAtPosition(-43);
            //System.out.println("trying to go verticle");
        }
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.LOW_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN){
            arm.moveAtPosition(-70);
           // System.out.println("back horizontal");
        }
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.INTAKE_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN){
            arm.moveAtPosition(-90);
            //System.out.println("trying to go groudn");
        }
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.INTAKE_UP) == DefaultControllerEnums.ButtonStatus.DOWN){
            arm.moveAtPosition(-15);
            //System.out.println("forward horizonal");
        }
        System.out.println("Motor Power " + arm.getVoltage());
        //System.out.println("Arm Position: " + arm.getRotations()); */
    }

    public void moveArm(double position){
        arm.moveAtPosition(position);
        UserInterface.smartDashboardPutNumber("Arm Goal Position" , position);
    }


}
