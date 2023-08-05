package frc.piecemanipulation;

import static frc.robot.Robot.robotSettings;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import frc.controllers.ControllerEnums;
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

        armRotationController.setBrake(true);
        armExtendingController.setBrake(true);

        armRotationPIDController.setTolerance(5, 10);

        armExtendingController.resetEncoder();
        armRotationController.resetEncoder();
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

        if (xbox.get(DefaultControllerEnums.XBoxButtons.RIGHT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) {
            resetArmEncoder();
        }

        if (xbox.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) {
            System.out.println("Turning off brake mode");
            armExtendingController.setBrake(false);
            armRotationController.setBrake(false);
        }

        double exposition = armExtendingController.getRotations();
        extendMove();
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
        // armExtendingController.resetEncoder();
        // armRotationController.resetEncoder();
    }

    @Override
    public void initAuton() {
        // armRotationController.resetEncoder();
        // armRotationController.resetEncoder();
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
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3, DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, DefaultControllers.BUTTON_PANEL);
    }

    public void createMotors() {
        if (robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            armRotationController = new TalonMotorController(robotSettings.ARM_ROTATE_MOTOR_ID, robotSettings.ARM_MOTOR_CANBUS);
        if (robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            armRotationController = new SparkMotorController(robotSettings.ARM_ROTATE_MOTOR_ID, MotorType.kBrushless);

        // arm.setOutPutRange(-.8,.8);
        armRotationController.setRealFactorFromMotorRPM(1, 1);
        armRotationController.setCurrentLimit(40);

        if (robotSettings.ARM_EXTEND) {
            if (robotSettings.ARM_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
                armExtendingController = new TalonMotorController(robotSettings.ARM_EXTEND_MOTOR_ID, robotSettings.ARM_MOTOR_CANBUS);
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
        if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.RIGHT) == DefaultControllerEnums.ButtonStatus.DOWN)) {
            armExtendingController.setInverted(false);
            armExtendingController.moveAtPercent(2);
        } else if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.LEFT) == DefaultControllerEnums.ButtonStatus.DOWN)) {
            armExtendingController.setInverted(true);
            armExtendingController.moveAtPercent(2);
        } else {
            armExtendingController.moveAtPercent(0);
        }

        if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.UP) == DefaultControllerEnums.ButtonStatus.DOWN)) {
            armRotationController.setInverted(false);
            armRotationController.moveAtPercent(5);
        } else if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.DOWN) == DefaultControllerEnums.ButtonStatus.DOWN)) {
            armRotationController.setInverted(true);
            armRotationController.moveAtPercent(5);
        } else {
            armRotationController.moveAtPercent(0);
        }
    }

    public void PositionDrive() {
        // // Human Player
        // if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1)/*panel2.get(ControllerEnums.ButtonPanelButtonsPlacement2023.)*/ == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     armRotationController.setInverted(false);
        //     armRotationPIDController.setSetpoint(27);

        // // Stable
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Floor) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     armRotationController.setInverted(true);
        //     armRotationPIDController.setSetpoint(5);

        // // High Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.High) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     armRotationController.setInverted(true);
        //     armRotationPIDController.setSetpoint(63);

        // // Mid Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Mid) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     armRotationController.setInverted(true);
        //     armRotationPIDController.setSetpoint(80);

        // // Low Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Low) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     armRotationController.setInverted(true);
        //     armRotationPIDController.setSetpoint(55);
        // }

        armRotationController.moveAtPercent(armRotationPIDController.calculate(armRotationController.getRotations()));
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
        if (armExtendingController.getRotations() > 1000) {
            System.out.println("Adjusting...");
            armExtendingController.moveAtPercent(-5);
        }

        // if (xbox.get(DefaultControllerEnums.XBoxPOVButtons.RIGHT) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     // armExtendingController.setInverted(false);
        //     // System.out.println("Right Arm Extend: " + armExtendingController.getRotations());
        //     // System.out.println("PID TARGET RIGHT EXTEND PERCENT: " + armExtendingPIDController.calculate(armExtendingController.getRotations(), 15));
        //     armExtendingPIDController.setSetpoint(15);
        // } else if ((xbox.get(DefaultControllerEnums.XBoxPOVButtons.LEFT) == DefaultControllerEnums.ButtonStatus.DOWN)) {
        //     // System.out.println("Left Arm Extend: " + armExtendingController.getRotations());
        //     // System.out.println("PID TARGET LEFT EXTEND PERCENT: " + armExtendingPIDController.calculate(armExtendingController.getRotations(), 0));
        //     armExtendingPIDController.setSetpoint(0.5);
        // }

        // System.out.println("Extend: " + armExtendingController.getRotations());
        // System.out.println("PID TARGET EXTEND PERCENT: " + armExtendingPIDController.calculate(armExtendingController.getRotations()));

        armExtendingController.moveAtPercent(-armExtendingPIDController.calculate(armExtendingController.getRotations()));
    }
}