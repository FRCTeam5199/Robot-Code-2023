package frc.piecemanipulation;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.LEDs;
import frc.misc.SubsystemStatus;
import frc.misc.LEDs.LEDEnums;
import frc.robot.Robot;

import static frc.robot.Robot.arm;
import static frc.robot.Robot.robotSettings;

import java.io.IOError;
import java.io.IOException;


public class ManipulationManager implements ISubsystem {
    public static LEDs leds;
    public BaseController panel, xbox2;
    public double armGoal = 0;
    public double elevateGoal = 0;
    public boolean cubeConeMode = true; // true =  Cone, false  = Cube
    int[] rgby = {255, 255, 0};
    int[] rgbp = {138, 43, 226};

    public ManipulationManager(){
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        enableControllers();
        cubeConeMode = true;
        if(robotSettings.ENABLE_LEDS){
            leds = new LEDs();
            leds.init();
        }
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
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.FENDER_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(true);
            //ledenum(rgby);
            try{
            leds.yellow();
            }catch(IOError e){
                System.out.println("Yellow LED not working: most likely bc ENABLE_LEDS is false");
            }
        if (panel.get(ControllerEnums.ButtonPanelButtons2022.LOW_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(false);
            //ledenum(rgbp);
            try{
            leds.purple();
            }catch(IOError e){
                System.out.println("Purple LED not working: most likely bc ENABLE_LEDS is false");
            }


        if(xbox2.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            robotSettings.ARM_ELEVATOR_MANUAL = true;
        }
        if(xbox2.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            robotSettings.ARM_ELEVATOR_MANUAL = false;
        }


        if(!robotSettings.ARM_ELEVATOR_MANUAL) {
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN) {
                goTo(-2, -5);
                elevateGoal = -2;
                armGoal = -5;
            }
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_5) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -8.5;
                armGoal = -62.5;
            }
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -50;
                armGoal = -63;
            }

            if(!cubeConeMode) {
                
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -5;
                    armGoal = -235;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -5;
                    armGoal = -248;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -5;
                    armGoal = -275;
                }
            }
            if(cubeConeMode) {
                
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -5;
                    armGoal = -225;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -50;
                    armGoal = -205;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -5;
                    armGoal = -275;
                }

                Robot.arm.moveArm(armGoal);

                if (checkArmCollision()) {
                } else if (checkArmPassover()) {
                    Robot.elevator.moveElevator(-50);
                } else {
                    Robot.elevator.moveElevator(elevateGoal);
                }

            }
            if(robotSettings.ENABLE_WRIST){
                //-75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200
                if(arm.arm.getRotations() >= -137){
                    Robot.wrist.wrist.moveAtPosition(0);
                }else {
                    Robot.wrist.wrist.moveAtPosition(5);
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

    public void enableControllers(){
        panel = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT, BaseController.DefaultControllers.BUTTON_PANEL);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
    }

    public boolean checkArmPassover(){
        if (-75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -210){
            return true;
        }
        return false;
    }

    public boolean checkArmCollision(){
        if(Robot.arm.arm.getRotations() >= -10 || Robot.arm.arm.getRotations() <= -261){
           return true;
        }
        return false;
    }

    public void goTo(double elevator, double arm){

        Robot.arm.moveArm(arm);

        if (checkArmCollision()) {
        } else if (checkArmPassover()) {
            Robot.elevator.moveElevator(-50);
        } else {
            Robot.elevator.moveElevator(elevator);
        }
    }

    public void changeCubeCone(boolean cubeCone){
        cubeConeMode = cubeCone;
    }

    public static void ledenum(int rgbl[]){
        LEDEnums rgbEnums = LEDEnums.SOLID_COLOR_RGB;
    }
}
