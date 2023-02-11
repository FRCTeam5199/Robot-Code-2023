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
    public BaseController panel, xbox2, TCannon;
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
            //spike pickup
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -1;
                armGoal = -13;
            }
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_5) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -2;
                armGoal = -53;
            }
            if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_DOWN) == DefaultControllerEnums.ButtonStatus.DOWN) {
                elevateGoal = -50;
                armGoal = -63;
            }
//            if (TCannon.get(ControllerEnums.TCannonExtraButtons.TILT_LOW) == DefaultControllerEnums.ButtonStatus.DOWN){
//                elevateGoal = -6.5;
//                armGoal = -18.5;
//            }

            if(!cubeConeMode) {

                if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -1;
                    armGoal = -226;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -0;
                    armGoal = -243;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -1;
                    armGoal = -275;
                }
            }
            if(cubeConeMode) {
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -1;
                    armGoal = -219;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.PIVOT_PISTON_UP) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -0;
                    armGoal = -236;
                }
                if (panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    elevateGoal = -1;
                    armGoal = -275;
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
        //TCannon = BaseController.createOrGet(5, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public boolean checkArmPassover(){
        if (-66 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -190){
            return true;
        }
        return false;
    }

    public boolean checkArmCollision(){
        if(Robot.arm.arm.getRotations() >= 5 || Robot.arm.arm.getRotations() <= -276){
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
            if(Robot.arm.arm.getRotations() >= -70){
                Robot.wrist.wrist.moveAtPosition(0);
            }else {
                Robot.wrist.wrist.moveAtPosition(5);
            }
        }

        return false;
    }

    public void changeCubeCone(boolean cubeCone){
        cubeConeMode = cubeCone;
    }
}
