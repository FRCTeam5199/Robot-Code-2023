package frc.piecemanipulation;

import static frc.robot.Robot.robotSettings;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.controllers.basecontrollers.BaseController.DefaultControllers;
import frc.misc.ISubsystem;
import frc.misc.PID;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.motors.SparkMotorController;
import frc.motors.TalonMotorController;

public class Arm implements ISubsystem {
    public AbstractMotorController armRotationController, armExtendingController;
    public PIDController armExtendingPIDController = new PIDController(0.03, 0.001, 0);
    public PIDController armRotationPIDController = new PIDController(0.01, 0, 0);
    public BaseController xbox, xbox2, panel1, panel2, midiTop, midiBot;

    public Arm() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        createMotors();
        createExtendMotorPid(robotSettings.ARM_EXTEND_PID);
        createRotateMotorPid(robotSettings.ARM_ROTATE_PID);
        armExtendingController.setBrake(false);
        armRotationController.setBrake(true);
        armRotationPIDController.setTolerance(5, 10);
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
        if (robotSettings.ARM_ELEVATOR_MANUAL) {
            manuelDrive();
        } else {
            if (!robotSettings.ENABLE_PIECE_MANAGER) {
                PositionDrive();
            }
        }

        if (xbox2.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) {
            resetArmEncoder();
        }

        double exposition = armExtendingController.getRotations();
        extendMove();
        rotateMove();
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
        armExtendingController.resetEncoder();
        armRotationController.resetEncoder();
    }

    @Override
    public void initAuton() {
        armRotationController.resetEncoder();
        armRotationController.resetEncoder();
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

    public void createControllers() {
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, DefaultControllers.XBOX_CONTROLLER);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2,
                DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1, DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, DefaultControllers.BUTTON_PANEL);
    }

    public void createMotors() {
        if (robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            armRotationController = new TalonMotorController(robotSettings.ARM_ROTATE_MOTOR_ID,
                    robotSettings.ARM_MOTOR_CANBUS);
        if (robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            armRotationController = new SparkMotorController(robotSettings.ARM_ROTATE_MOTOR_ID, MotorType.kBrushless);

        // arm.setOutPutRange(-.8,.8);
        armRotationController.setRealFactorFromMotorRPM(1, 1);
        armRotationController.setCurrentLimit(40);

        if (robotSettings.ARM_EXTEND) {
            if (robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
                armExtendingController = new TalonMotorController(robotSettings.ARM_EXTEND_MOTOR_ID,
                        robotSettings.ARM_MOTOR_CANBUS);
            if (robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
                armExtendingController = new SparkMotorController(robotSettings.ARM_EXTEND_MOTOR_ID);
        }

        armExtendingController.setRealFactorFromMotorRPM(1, 1);
        armExtendingController.setCurrentLimit(40);
    }

    public void createExtendMotorPid(PID pid) {
        armExtendingController.setPid(pid);
    }

    public void createRotateMotorPid(PID pid) {
        armRotationController.setPid(pid);
    }

    public void resetArmEncoder() {
        // arm.resetEncoder();
    }

    public void manuelDrive() {
        /*
         * if(Math.abs(xbox2.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_Y)) >= .1){
         * armr.moveAtVoltage(xbox2.get(DefaultControllerEnums.XboxAxes.RIGHT_JOY_Y) *
         * -12);
         * } else {
         * armr.moveAtVoltage(0);
         * }
         */
    }

    public void PositionDrive() {
        /*
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.FENDER_SHOT) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * arm.moveAtPosition(-1);
         * //System.out.println("trying to go to zero");
         * }
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.TARMAC_SHOT) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * arm.moveAtPosition(-43);
         * //System.out.println("trying to go verticle");
         * }
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.LOW_SHOT) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * arm.moveAtPosition(-70);
         * // System.out.println("back horizontal");
         * }
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.INTAKE_DOWN) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * arm.moveAtPosition(-90);
         * //System.out.println("trying to go groudn");
         * }
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.INTAKE_UP) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * arm.moveAtPosition(-15);
         * //System.out.println("forward horizonal");
         * }
         */
    }

    public void moveArm(double position) {
        armRotationController.moveAtPosition(position);
        UserInterface.smartDashboardPutNumber("Arm Goal Position", position);
    }

    /**
     * Runs the arm extend motor
     **/
    public void extendMove() {
        // WORK ON MOVE LOGIC \/
        // System.out.println("Arm Extend: " + ((SparkMotorController)
        // armExtendingController).getAbsoluteRotations());
        /*
         * armExtendingController.moveAtPercent(
         * armExtendingPIDController
         * .calculate(((SparkMotorController)
         * armExtendingController).getAbsoluteRotations(), 20));
         */
        if (xbox.get(DefaultControllerEnums.XBoxPOVButtons.RIGHT) == DefaultControllerEnums.ButtonStatus.DOWN) {
            System.out.println("Right Extend: " + armExtendingController.getRotations());
            // armExtendingController.setInverted(false);
            // System.out.println("PID TARGET RIGHT EXTEND PERCENT: "
            // + armExtendingPIDController.calculate(armExtendingController.getRotations(),
            // 15));
            armExtendingController
                    .moveAtPercent(armExtendingPIDController.calculate(armExtendingController.getRotations(), 15));
        } else if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.LEFT) == DefaultControllerEnums.ButtonStatus.DOWN)) {
            System.out.println("Left Extend: " + armExtendingController.getRotations());
            // System.out.println("PID TARGET LEFT EXTEND PERCENT: "
            // + armExtendingPIDController.calculate(armExtendingController.getRotations(),
            // 0));
            armExtendingController
                    .moveAtPercent(armExtendingPIDController.calculate(armExtendingController.getRotations(), 0));
            // If arm is at the elevator, extend the arm so it doesn't hit the elevator.
        }
        /*
         * else if ((armRotationController.getRotations() > 13 &&
         * armRotationController.getRotations() < 20)
         * && (armExtendingController.getRotations() < 10)) {
         * armExtendingController.moveAtPosition(armPIDController
         * .calculate(((SparkMotorController)
         * armExtendingController).getAbsoluteRotations(), 7));
         * }
         */
        else {
            armExtendingController.moveAtPercent(0);
        }

    }

    public void rotateMove() {
        if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.UP) == DefaultControllerEnums.ButtonStatus.DOWN)) {
            System.out.println("Up Rotate: " + armRotationController.getRotations());
            armRotationController.setInverted(false);
            // System.out.println("PID TARGET UP ROTATE PERCENT: " +
            // armRotationPIDController.calculate(armRotationController.getRotations(),
            // 30));
            armRotationController.moveAtPercent(armRotationPIDController.calculate(armRotationController.getRotations(),
                    30));
        } else if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.DOWN) == DefaultControllerEnums.ButtonStatus.DOWN)) {
            System.out.println("Down Rotate: " + armRotationController.getRotations());
            armRotationController.setInverted(true);
            // System.out.println("PID TARGET DOWN ROTATE PERCENT: " +
            // armRotationPIDController.calculate(armRotationController.getRotations(),
            // 70));
            armRotationController
                    .moveAtPercent(armRotationPIDController.calculate(armRotationController.getRotations(), 70));
        } else {
            armRotationController.moveAtPercent(0);
        }

        // System.out.println("Arm Rotate: " + ((SparkMotorController)
        // armRotationController).getAbsoluteRotations());
    }
}