package frc.piecemanipulation;

import static frc.robot.Robot.robotSettings;
import static frc.robot.Robot.arm;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

public class Elevator implements ISubsystem {
    public AbstractMotorController elevatorController;
    public PIDController elevatorPIDController = new PIDController(0.01, 0, 0);
    public BaseController xbox, xbox2, panel1, panel2, midiTop, midiBot;

    public Elevator() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        createMotors();
        createMotorPid(robotSettings.ELEVATORPID);
        elevatorController.setBrake(true);
        elevatorPIDController.setTolerance(5, 10);
        
        elevatorController.resetEncoder();
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
        if (robotSettings.ARM_ELEVATOR_MANUAL) {
            manuelDrive();
        } else {
            if (!robotSettings.ENABLE_PIECE_MANAGER)
                positionDrive();
        }

        if (xbox.get(DefaultControllerEnums.XBoxButtons.RIGHT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) {
            resetElevateEncoder();
        }

        // if (xbox.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     elevatorController.setBrake(false);
        // }

        // elevator();
    }

    @Override
    public void initTest() {
    }

    @Override
    public void initTeleop() {
        // System.out.println(">>>>>>>>>>>>>>>>>> RESET ELEVATOR ENCODER");
        // elevatorController.resetEncoder();
    }

    @Override
    public void initAuton() {
        // System.out.println(">>>>>>>>>>>>>>>>>> RESET ELEVATOR ENCODER");
        // elevatorController.resetEncoder();
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
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT, BaseController.DefaultControllers.XBOX_CONTROLLER);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2, BaseController.DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2, BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3, BaseController.DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID, BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID, BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public void createMotors() {
        if (robotSettings.ELEVATOR_MOTOR_TYPE == AbstractMotorController.SupportedMotors.TALON_FX)
            elevatorController = new TalonMotorController(robotSettings.ELEVATOR_MOTOR_ID,
                    robotSettings.ELEVATOR_MOTOR_CANBUS);
        if (robotSettings.ELEVATOR_MOTOR_TYPE == AbstractMotorController.SupportedMotors.CAN_SPARK_MAX)
            elevatorController = new SparkMotorController(robotSettings.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        // elevate.setRealFactorFromMotorRPM(robotSettings.ELEVATOR_GEARING *
        // (robotSettings.ELEVATOR_SPROCKET_DIAMETER * Math.PI / 12), 1/60D );
        elevatorController.setRealFactorFromMotorRPM(1, 1);
        elevatorController.setCurrentLimit(40);

        System.out.println("Elevator Motor Type: " + robotSettings.ELEVATOR_MOTOR_TYPE);
    }

    public void resetElevateEncoder() {
        elevatorController.resetEncoder();
    }

    public void createMotorPid(PID pid) {
        elevatorController.setPid(pid);
    }

    public void manuelDrive() {
        if (xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            // System.out.println("Elevator Up: " + elevatorController.getRotations());
            elevatorController.setInverted(false);
            elevatorController.moveAtPercent(5);
        } else if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            // System.out.println("Elevator Down: " + elevatorController.getRotations());
            elevatorController.setInverted(true);
            elevatorController.moveAtPercent(5);
        } else {
            elevatorController.moveAtPercent(0);
        }

    }

    public void positionDrive() {
        // // Human Player
        // if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     // System.out.println("Elevator Up: " + elevatorController.getRotations());
        //     System.out.println("From Elevator - Cone Button Pressed");
            
        //     elevatorPIDController.setSetpoint(38);

        // // Stable
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Floor) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     // System.out.println("Elevator Down: " + elevatorController.getRotations());
        //     elevatorPIDController.setSetpoint(0);

        // // High Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.High) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     // System.out.println("Elevator Down: " + elevatorController.getRotations());
        //     if (arm.armRotationController.getRotations() > 40) {
        //         elevatorPIDController.setSetpoint(38);
        //     }

        // // Mid Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Mid) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     System.out.println("Elevator Down: " + elevatorController.getRotations());
        //     elevatorPIDController.setSetpoint(38);

        // // Low Cone Goal
        // } else if (panel1.get(ControllerEnums.ButtonPanelButtonsElse2023.Low) == DefaultControllerEnums.ButtonStatus.DOWN) {
        //     System.out.println("Elevator Down: " + elevatorController.getRotations());
        //     elevatorPIDController.setSetpoint(0);
        // }

        elevatorController.moveAtPercent(elevatorPIDController.calculate(elevatorController.getRotations()));
    }

    public void moveElevator(double position) {
        elevatorController.moveAtPosition(position);
        UserInterface.smartDashboardPutNumber("Elevator goal position:", position);
        System.out.println("Moving Elevator: " + elevatorController.getRotations());
    }

    // WORK ON A BETTER NAME \/
    public void elevator() {
        if (xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            System.out.println("Elevator Up: " + elevatorController.getRotations());
            // System.out.println("PID TARGET UP ELEVATOR PERCENT: " +
            // elevatorPIDController.calculate(elevatorController.getRotations(), 35));
            // elevatorController.moveAtPercent(elevatorPIDController.calculate(elevatorController.getRotations(),
            // 35));
            elevatorPIDController.setSetpoint(38);
        } else if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            System.out.println("Elevator Down: " + elevatorController.getRotations());
            // System.out.println("PID TARGET DOWN ELEVATOR PERCENT: " +
            // elevatorPIDController.calculate(elevatorController.getRotations(), 0));
            // elevatorController.moveAtPercent(elevatorPIDController.calculate(elevatorController.getRotations(),
            // 0));
            elevatorPIDController.setSetpoint(0);
        }

        elevatorController.moveAtPercent(elevatorPIDController.calculate(elevatorController.getRotations()));

        // pidElevator.setSetpoint(15);
        // elevate.moveAtPercent(elevatorPIDController.calculate(elevate.getRotations(),
        // 10));

        /*
         * if (elevate.getRotations() < 10) {
         * elevate.moveAtPercent(elevatorPIDController.calculate(elevate.getRotations(),
         * 10));
         * System.out.println("Elevator Up: " + elevate.getRotations());
         * } else if (elevate.getRotations() > 0) {
         * elevate.moveAtPercent(elevatorPIDController.calculate(elevate.getRotations(),
         * 0));
         * System.out.println("Elevator Down: " + elevate.getRotations());
         * }
         */

        // System.out.println("Elevator: " + elevate.getRotations());
    }
}
