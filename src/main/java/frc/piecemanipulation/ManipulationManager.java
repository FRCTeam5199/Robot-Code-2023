package frc.piecemanipulation;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.robot.Robot;

import static frc.robot.Robot.robotSettings;


public class ManipulationManager implements ISubsystem {
    public BaseController panel, xbox2;
    public double armGoal = 0;
    public double elevateGoal = 0;
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
        if(panel.get(ControllerEnums.ButtonPanelButtons2022.FENDER_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(true);
        if (panel.get(ControllerEnums.ButtonPanelButtons2022.LOW_SHOT) == DefaultControllerEnums.ButtonStatus.DOWN)
            changeCubeCone(false);


        if(xbox2.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN){
            robotSettings.ARM_ELEVATOR_MANUAL = true;
        }
        if(xbox2.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            robotSettings.ARM_ELEVATOR_MANUAL = false;
        }


        if(!robotSettings.ARM_ELEVATOR_MANUAL) {
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.arm.moveArm(-5);
                elevateGoal = -2;
            }
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_5) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.arm.moveArm(-62.5);
                elevateGoal = -8.5;
            }
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.arm.moveArm(-63);
                elevateGoal = -50;
            }

            if(!cubeConeMode) {
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.arm.moveArm(-235);
                    elevateGoal = -5;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.arm.moveArm(-248);
                    elevateGoal = -5;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.arm.moveArm(-275);
                    elevateGoal = -5;
                }
            }
            if(cubeConeMode) {
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.arm.moveArm(-225);
                    elevateGoal = -5;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.arm.moveArm(-205);
                    elevateGoal = -50;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.arm.moveArm(-275);
                    elevateGoal = -5;
                }
            }
            if (checkArmCollision()) {
            } else if (checkArmPassover()) {
                Robot.elevator.moveElevator(-50);
            } else {
                    Robot.elevator.moveElevator(elevateGoal);
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
        if (-75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200){
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

    public void changeCubeCone(boolean cubeCone){
        cubeConeMode = cubeCone;
    }
}
