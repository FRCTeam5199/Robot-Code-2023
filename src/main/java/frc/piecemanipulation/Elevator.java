package frc.piecemanipulation;

// import edu.wpi.first.math.controller.PIDController;
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

import static frc.robot.Robot.robotSettings;

public class Elevator implements ISubsystem {
    public AbstractMotorController elevate;
    public BaseController xbox, xbox2, panel1, panel2, midiTop, midiBot;

    public Elevator(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        createMotors();
        createMotorPid(robotSettings.ELEVATORPID);
        elevate.setBrake(true);
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
       if (robotSettings.ARM_ELEVATOR_MANUAL){
            manuelDrive();
       }else{
           if (!robotSettings.ENABLE_PIECE_MANAGER)
               positionDrive();
        }
        if(xbox2.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN){
            //resetElevateEncoder();
        }
    }

    @Override
    public void initTest() {

    }

    @Override
    public void initTeleop() {
        //elevate.resetEncoder();
    }

    @Override
    public void initAuton() {
        elevate.resetEncoder();
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
        if(robotSettings.ELEVATOR_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            elevate = new TalonMotorController(robotSettings.ELEVATOR_MOTOR_ID, robotSettings.ELEVATOR_MOTOR_CANBUS);
        if(robotSettings.ELEVATOR_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            elevate = new SparkMotorController(robotSettings.ELEVATOR_MOTOR_ID);
        //elevate.setRealFactorFromMotorRPM(robotSettings.ELEVATOR_GEARING * (robotSettings.ELEVATOR_SPROCKET_DIAMETER * Math.PI / 12), 1/60D );
        elevate.setRealFactorFromMotorRPM(1, 1 );
        elevate.setCurrentLimit(40);
    }

    public void resetElevateEncoder(){
        elevate.resetEncoder();
    }

    public void createMotorPid(PID pid){
        elevate.setPid(pid);
    }

    public void manuelDrive(){
        if(Math.abs(xbox2.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_Y)) >= .1){
            elevate.moveAtVoltage(xbox2.get(DefaultControllerEnums.XboxAxes.LEFT_JOY_Y) * -6);
        }else {
            elevate.moveAtVoltage(0);
        }
        System.out.println("Elevator Position: " + elevate.getRotations());
    }

    public void positionDrive(){
       /* if(panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == DefaultControllerEnums.ButtonStatus.DOWN){
            elevate.moveAtPosition(-1);
            System.out.println("top");
        }
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN){
            elevate.moveAtPosition(-26);
            System.out.println("MID");
        }
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_5) == DefaultControllerEnums.ButtonStatus.DOWN){
            elevate.moveAtPosition(-52);
            System.out.println("bottom");
        }
        System.out.println("Elevator Position: " + elevate.getRotations());
        System.out.println("Elevator Voltage: " + elevate.getVoltage()); */
    }

    public void moveElevator(double position){
        elevate.moveAtPosition(position);
        UserInterface.smartDashboardPutNumber("Elevator goal position:", position);
    }
}
