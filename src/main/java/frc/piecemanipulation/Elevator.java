package frc.piecemanipulation;

import static frc.robot.Robot.robotSettings;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

        if (xbox2.get(DefaultControllerEnums.XBoxButtons.LEFT_BUMPER) == DefaultControllerEnums.ButtonStatus.DOWN) {
            // resetElevateEncoder();
        }

        elevator();
    }

    @Override
    public void initTest() {
    }

    @Override
    public void initTeleop() {
        elevatorController.resetEncoder();
    }

    @Override
    public void initAuton() {
        elevatorController.resetEncoder();
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
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT,
                BaseController.DefaultControllers.XBOX_CONTROLLER);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2,
                BaseController.DefaultControllers.XBOX_CONTROLLER);
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT1,
                BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2,
                BaseController.DefaultControllers.BUTTON_PANEL);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
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
        // elevate.moveAtPercent(15);
        // if(/*Math.abs(*/xbox2.get(DefaultControllerEnums.XBoxPOVButtons.UP/*XboxAxes.LEFT_JOY_Y))
        // >= .1*/) == DefaultControllerEnums.ButtonStatus.DOWN) {
        // elevate.moveAtVoltage(/*xbox2.get(DefaultControllerEnums.XBoxPOVButtons.UP.XboxAxes.LEFT_JOY_Y)
        // * -6)*/15);
        /*
         * } else {
         * elevate.moveAtVoltage(0);
         * }
         */
        // System.out.println("Manuel Drive: " + elevate.getRotations());
    }

    public void positionDrive() {
        /*
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_UP) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * elevate.moveAtPosition(-1);
         * System.out.println("top");
         * }
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.FIRST_STAGE_DOWN) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * elevate.moveAtPosition(-26);
         * System.out.println("MID");
         * }
         * if(panel.get(ControllerEnums.ButtonPanelButtons2022.AUX_5) ==
         * DefaultControllerEnums.ButtonStatus.DOWN){
         * elevate.moveAtPosition(-52);
         * System.out.println("bottom");
         * }
         * System.out.println("Elevator Position: " + elevate.getRotations());
         * System.out.println("Elevator Voltage: " + elevate.getVoltage());
         */
        // System.out.println("Position Drive: " + elevate.getRotations());
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
