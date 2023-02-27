package frc.piecemanipulation;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.controllers.basecontrollers.DefaultControllerEnums.ButtonStatus;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.robot.Robot;

import static frc.robot.Robot.arm;
import static frc.robot.Robot.robotSettings;


public class ManipulationManager implements ISubsystem {
    public BaseController panel1, panel2, xbox2, midiTop, midiBot;
    public double armGoal = 0;
    public double elevateGoal = 2.2;
    public boolean cubeConeMode = true; // true =  Cone, false  = Cube
    public boolean spikeUp = false;

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
        /*if(midiTop.get(ControllerEnums.MidiController.R1C5) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(true);
        if (midiTop.get(ControllerEnums.MidiController.R1C6) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(false);
        */
        if(panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Cone) == ButtonStatus.DOWN){
            changeCubeCone(true);
        }
        if(panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Cube) == ButtonStatus.DOWN){
            changeCubeCone(false);
        }
        //will allow later this is just for quick fix
        /*if(xbox2.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            robotSettings.ARM_ELEVATOR_MANUAL = true;
        }
        if(xbox2.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            robotSettings.ARM_ELEVATOR_MANUAL = false;
            Robot.intake.intakeIn();
        }*/


        if(!robotSettings.ARM_ELEVATOR_MANUAL) {
            if (midiTop.get(ControllerEnums.MidiController.R1C3) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -11;
                armGoal = -62;
            }
            
            if (midiTop.get(ControllerEnums.MidiController.R1C2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -44;
                armGoal = -133;
            }
            if (midiTop.get(ControllerEnums.MidiController.R4C4) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.elevator.moveElevator(2.2);
                armGoal = 0;
            }
            //spike
            if (midiTop.get(ControllerEnums.MidiController.R2C3) == DefaultControllerEnums.ButtonStatus.DOWN){
                elevateGoal = -8.42;
                armGoal = -15.8;
                spikeUp = true;
            }

            if(!cubeConeMode) {

                if (midiTop.get(ControllerEnums.MidiController.R1C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -1;
                    armGoal = -240;
                }
                if (midiTop.get(ControllerEnums.MidiController.R2C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -2;
                    armGoal = -254;
                }
                if (midiTop.get(ControllerEnums.MidiController.R3C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = 0;
                    armGoal = -290;
                }
            }
            if(cubeConeMode) {
                if (midiTop.get(ControllerEnums.MidiController.R1C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -3;
                    armGoal = -226;
                }
                if (midiTop.get(ControllerEnums.MidiController.R2C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -44;
                    armGoal = -206;
                }
                if (midiTop.get(ControllerEnums.MidiController.R3C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -0;
                    armGoal = -290;
                }

            }
            Robot.arm.moveArm(armGoal);

            if (checkArmCollision()) {
            } else if (checkArmPassover()) {
                Robot.elevator.moveElevator(-44);
            } else {
                if ((spikeUp)){
                    Robot.elevator.moveElevator(0);
                    if (Math.abs(-15.8 - Robot.arm.arm.getRotations()) <= .2) {
                        Robot.elevator.moveElevator(elevateGoal);
                        if (Math.abs(-8.42 - Robot.elevator.elevate.getRotations()) <= .2) {
                            Robot.intake.intakeIn();
                            spikeUp = false;
                        }
                    }
                }else {
                    Robot.elevator.moveElevator(elevateGoal);
                }
            }

            if(robotSettings.ENABLE_WRIST){
                //-75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200
                if(Robot.arm.arm.getRotations() >= -123){
                    if(Robot.wrist.wrist.getRotations() >= 0 && Robot.wrist.wrist.getRotations() <= 4000){
                        Robot.wrist.wrist.moveAtVoltage(0);
                    }else{
                        Robot.wrist.wrist.moveAtVoltage(-6);
                    }
                }else {
                    if(Robot.wrist.wrist.getRotations() <= 4011 && Robot.wrist.wrist.getRotations() >= 10){
                        Robot.wrist.wrist.moveAtVoltage(0);
                    }else{
                        Robot.wrist.wrist.moveAtVoltage(6);
                    }
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
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public boolean checkArmPassover(){
        if (-67.5 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -185){
            return true;
        }
        return false;
    }

    public boolean checkArmCollision(){
        if(Robot.arm.arm.getRotations() >= -10 || Robot.arm.arm.getRotations() <= -300){
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
            Robot.elevator.moveElevator(-44);
        } else {
            Robot.elevator.moveElevator(elevator);
            System.out.println("tring to move to: " + elevator);
        }
        System.out.println("current elevator position: " + Robot.elevator.elevate.getRotations());

        if(robotSettings.ENABLE_WRIST){
            //-75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200
            if(Robot.arm.arm.getRotations() >= -123){
                if(Robot.wrist.wrist.getRotations() >= 0 && Robot.wrist.wrist.getRotations() <= 4000){
                    Robot.wrist.wrist.moveAtVoltage(0);
                }else{
                    Robot.wrist.wrist.moveAtVoltage(-6);
                }
            }else {
                if(Robot.wrist.wrist.getRotations() <= 4011 && Robot.wrist.wrist.getRotations() >= 10){
                    Robot.wrist.wrist.moveAtVoltage(0);
                }else{
                    Robot.wrist.wrist.moveAtVoltage(6);
                }
            }
        }

        return false;
    }

    public void changeCubeCone(boolean cubeCone){
        cubeConeMode = cubeCone;
    }
}
