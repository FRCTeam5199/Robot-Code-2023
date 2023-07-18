package frc.piecemanipulation;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.controllers.basecontrollers.DefaultControllerEnums.ButtonStatus;
import frc.misc.ISubsystem;
import frc.misc.SubsystemStatus;
import frc.misc.UserInterface;
import frc.motors.AbstractMotorController;
import frc.robot.Robot;

import static frc.robot.Robot.*;

public class Claw implements ISubsystem {
    public AbstractMotorController clawLeft, clawRight, clawBottom;
    private BaseController xbox, panel1, panel2, midiTop, midiBot;
    private I2C.Port i2cPort;
    public ColorSensorV3 m_colorSensor;
    public Timer closeTimer;

    public Claw() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        createControllers();
        // createMotors();
        closeTimer = new Timer();
        closeTimer.reset();
        closeTimer.start();
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
        if (robotSettings.CLAW_MANUAL)
            manuelDrive();
        updateGeneric();
    }

    @Override
    public void updateAuton() {
        updateGeneric();
    }

    @Override
    public void updateGeneric() {
        if (robotSettings.ENABLE_COLOR_SENSOR) {
            UserInterface.smartDashboardPutNumber("proxy cube", m_colorSensor.getProximity());
            if (m_colorSensor.getProximity() >= 350) {
                if (clawLeft.getVoltage() >= 10 || clawRight.getVoltage() >= 10) {
                    clawLeft.moveAtVoltage(0);
                    clawRight.moveAtVoltage(0);
                }
            }
        }
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

    /*
     * public void createMotors() {
     * if (robotSettings.CLAW_MOTOR_TYPE ==
     * AbstractMotorController.SupportedMotors.TALON_FX) {
     * clawLeft = new TalonMotorController(robotSettings.CLAW_MOTOR_LEFT_ID,
     * robotSettings.CLAW_MOTOR_CANBUS);
     * clawRight = new TalonMotorController(robotSettings.CLAW_MOTOR_RIGHT_ID,
     * robotSettings.CLAW_MOTOR_CANBUS);
     * }
     * if (robotSettings.CLAW_MOTOR_TYPE ==
     * AbstractMotorController.SupportedMotors.CAN_SPARK_MAX) {
     * clawLeft = new SparkMotorController(robotSettings.CLAW_MOTOR_LEFT_ID);
     * clawRight = new SparkMotorController(robotSettings.CLAW_MOTOR_RIGHT_ID);
     * }
     * if (robotSettings.CLAW_MOTOR_TYPE ==
     * AbstractMotorController.SupportedMotors.VICTOR) {
     * clawLeft = new VictorMotorController(robotSettings.CLAW_MOTOR_LEFT_ID);
     * clawRight = new VictorMotorController(robotSettings.CLAW_MOTOR_RIGHT_ID);
     * }
     * if (robotSettings.CLAW_MOTOR_BOTTOM_TYPE ==
     * AbstractMotorController.SupportedMotors.VICTOR) {
     * clawBottom = new VictorMotorController(robotSettings.CLAW_MOTOR_BOTTOM_ID);
     * }
     * clawLeft.setCurrentLimit(20);
     * clawRight.setCurrentLimit(20);
     * clawRight.setBrake(true);
     * clawLeft.setBrake(true);
     * clawRight.setInverted(true);
     * clawLeft.setInverted(true);
     * clawBottom.setBrake(true);
     * clawBottom.setInverted(false);
     * 
     * }
     */

    public void createControllers() {
        xbox = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT,
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

    public void manuelDrive() {
        if (!robotSettings.BRANDONISNOTHERE) {
            if (!manipulationManager.cubeConeMode) {
                if (xbox.get(
                        DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    // System.out.println("X is being pressed");
                    clawRight.moveAtVoltage(3);
                    clawLeft.moveAtVoltage(-3);
                    clawBottom.moveAtPercent(1);
                } else if (xbox
                        .get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    if (robotSettings.ENABLE_COLOR_SENSOR) {
                        if (m_colorSensor.getProximity() >= 350) {
                            clawLeft.moveAtVoltage(0);
                            clawRight.moveAtVoltage(0);
                            clawBottom.moveAtPercent(0);
                        } else {
                            // System.out.println("Y is being pressed");
                            clawRight.moveAtVoltage(-12);
                            clawLeft.moveAtVoltage(12);
                            clawBottom.moveAtPercent(-.6);
                        }
                    } else {
                        // System.out.println("Y is being pressed");
                        clawRight.moveAtVoltage(-12);
                        clawLeft.moveAtVoltage(12);
                        clawBottom.moveAtPercent(-.6);
                    }
                } else {
                    clawRight.moveAtVoltage(0);
                    clawLeft.moveAtVoltage(0);
                    clawBottom.moveAtPercent(0);
                }
            }
            if (manipulationManager.cubeConeMode) {
                // System.out.println(elevate.getRotations());
                clawRight.moveAtVoltage(0);
                clawLeft.moveAtVoltage(0);
                clawBottom.moveAtPercent(0);
                if (xbox.get(
                        DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kForward);
                    closeTimer.reset();
                }
                if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                    Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kReverse);
                    closeTimer.reset();
                }
                if (robotSettings.ENABLE_COLOR_SENSOR) {
                    // if (m_colorSensor.getProximity() >= 150 && closeTimer.get() >= .5) {
                    // Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kReverse);
                    // closeTimer.reset();
                    // }
                }
            }
        } else {
            if (xbox.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                // System.out.println("X is being pressed");
                clawRight.moveAtVoltage(6);
                clawLeft.moveAtVoltage(-6);
                clawBottom.moveAtPercent(1);
            } else if (xbox
                    .get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
                if (robotSettings.ENABLE_COLOR_SENSOR) {
                    if (m_colorSensor.getProximity() >= 500) {
                        clawLeft.moveAtVoltage(0);
                        clawRight.moveAtVoltage(0);
                        clawBottom.moveAtPercent(0);
                    } else {
                        // System.out.println("Y is being pressed");
                        clawRight.moveAtVoltage(-12);
                        clawLeft.moveAtVoltage(12);
                        clawBottom.moveAtPercent(-.6);
                    }
                } else {
                    // System.out.println("Y is being pressed");
                    clawRight.moveAtVoltage(-12);
                    clawLeft.moveAtVoltage(12);
                    clawBottom.moveAtPercent(-.6);
                }
            } else {
                clawRight.moveAtVoltage(0);
                clawLeft.moveAtVoltage(0);
                clawBottom.moveAtPercent(0);
            }
            if (xbox.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kForward);
                closeTimer.reset();
            }
            if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
                Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kReverse);
                closeTimer.reset();
            }
            if (closeTimer.hasElapsed(.5)) {
                if (m_colorSensor.getProximity() >= 1024) {
                    Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kForward);
                    closeTimer.reset();
                }
            }
        }
        if (!robotSettings.ARM_ELEVATOR_MANUAL) {
            switch (robotSettings.DRIVE_STYLE) {
                case MIDI: {
                    if (midiTop.get(ControllerEnums.MidiController.R2C4) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (midiTop.get(ControllerEnums.MidiController.R1C3) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (midiTop.get(ControllerEnums.MidiController.R1C2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                    }
                    break;
                }
                case STANDARD_2023: {
                    if (xbox.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == ButtonStatus.DOWN) {
                        pneumatics.clawPiston.set(DoubleSolenoid.Value.kReverse);
                    }
                    if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (panel1.get(ControllerEnums.ButtonPanelButtonsPlacement2023.Stable) == ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                    }
                    if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.GTShute) == ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeD) == ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.SpikeU) == ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                        break;
                    }
                    break;
                }
            }
            // }
        } else {
            switch (robotSettings.DRIVE_STYLE) {
                case MIDI: {
                    if (midiTop.get(ControllerEnums.MidiController.R2C5) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                    }
                    if (midiTop.get(ControllerEnums.MidiController.R2C6) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                    }
                    break;
                }
                case STANDARD_2023: {
                    /*
                     * if (panel2.get(ControllerEnums.MidiController.R2C5) ==
                     * DefaultControllerEnums.ButtonStatus.DOWN) {
                     * Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kForward);
                     * }
                     * if (panel2.get(ControllerEnums.MidiController.R2C6) ==
                     * DefaultControllerEnums.ButtonStatus.DOWN) {
                     * Robot.pneumatics.spikePiston.set(DoubleSolenoid.Value.kReverse);
                     * break;
                     * }
                     */
                }
            }
        }

    }

    public void clawIn() {
        Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kReverse);
    }

    public void clawOut() {
        Robot.pneumatics.clawPiston.set(DoubleSolenoid.Value.kForward);
    }
}