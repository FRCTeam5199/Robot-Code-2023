package frc.piecemanipulation;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.robot.Robot;

import static frc.robot.Robot.arm;
import static frc.robot.Robot.robotSettings;


public class ManipulationManager implements ISubsystem {
    public BaseController panel, xbox2, midiTop, midiBot;
    public double armGoal = 0;
    public double elevateGoal = 0.6;
    public boolean cubeConeMode = true; // true =  Cone, false  = Cube

    public ManipulationManager(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        enableControllers();
        cubeConeMode = true;
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
        if(midiTop.get(ControllerEnums.MidiController.R1C5) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(true);
        if (midiTop.get(ControllerEnums.MidiController.R1C6) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(false);


        if(xbox2.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            robotSettings.ARM_ELEVATOR_MANUAL = true;
        }
        if(xbox2.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            robotSettings.ARM_ELEVATOR_MANUAL = false;
        }


        if(!robotSettings.ARM_ELEVATOR_MANUAL) {
            //spike pickup
            if (midiTop.get(ControllerEnums.MidiController.R1C3) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -3;
                armGoal = -50;
            }
            if (midiTop.get(ControllerEnums.MidiController.R1C2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -45;
                armGoal = -145;
            }
//            if (TCannon.get(ControllerEnums.TCannonExtraButtons.TILT_LOW) == DefaultControllerEnums.ButtonStatus.DOWN){
//                elevateGoal = -6.5;
//                armGoal = -18.5;
//            }

            if(!cubeConeMode) {

                if (midiTop.get(ControllerEnums.MidiController.R1C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -2;
                    armGoal = -232;
                }
                if (midiTop.get(ControllerEnums.MidiController.R2C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -3;
                    armGoal = -244;
                }
                if (midiTop.get(ControllerEnums.MidiController.R3C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = 0;
                    armGoal = -280;
                }
            }
            if(cubeConeMode) {
                if (midiTop.get(ControllerEnums.MidiController.R1C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -2;
                    armGoal = -218;
                }
                if (midiTop.get(ControllerEnums.MidiController.R2C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -44;
                    armGoal = -201;
                }
                if (midiTop.get(ControllerEnums.MidiController.R3C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -0;
                    armGoal = -280;
                }

            }
            Robot.arm.moveArm(armGoal);

            if (checkArmCollision()) {
            } else if (checkArmPassover()) {
                Robot.elevator.moveElevator(-44);
            } else {
                Robot.elevator.moveElevator(elevateGoal);
            }
            if(robotSettings.ENABLE_WRIST){
                //-75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200
                if(arm.arm.getRotations() >= -115){
                    Robot.wrist.wrist.moveAtPosition(-2);
                }else {
                    Robot.wrist.wrist.moveAtPosition(-42.5);
                }
            }
        }
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

    public void enableControllers() {
        panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.DefaultControllers.BUTTON_PANEL);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public boolean checkArmPassover(){
        if (-95 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -190){
            return true;
        }
        return false;
    }

    public boolean checkArmCollision(){
        if(Robot.arm.arm.getRotations() >= 0 || Robot.arm.arm.getRotations() <= -300){
           return true;
        }
        return false;
    }

    public boolean goTo(double elevator, double arm){

        if(Math.abs(elevator - Robot.elevator.elevate.getRotations()) <= .2 && Math.abs(arm - Robot.arm.arm.getRotations()) <= .2) {
            return true;
        }
        Robot.arm.moveArm(arm);

        if (checkArmCollision()) {
        } else if (checkArmPassover()) {
            Robot.elevator.moveElevator(-50);
        } else {
            Robot.elevator.moveElevator(elevator);
            System.out.println("tring to move to: " + elevator);
        }
        System.out.println("current elevator position: " + Robot.elevator.elevate.getRotations());

        if(robotSettings.ENABLE_WRIST){
            //-75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200
            if(Robot.arm.arm.getRotations() >= -115){
                Robot.wrist.wrist.moveAtPosition(-2);
            }else {
                Robot.wrist.wrist.moveAtPosition(-42.5);
            }
        }

        return false;
    }

    public void changeCubeCone(boolean cubeCone){
        cubeConeMode = cubeCone;
    }
}
