package frc.piecemanipulation;

import frc.controllers.ControllerEnums;
import frc.controllers.basecontrollers.BaseController;
import frc.controllers.basecontrollers.DefaultControllerEnums;
import frc.controllers.basecontrollers.DefaultControllerEnums.ButtonStatus;
import frc.misc.ISubsystem;
import frc.misc.LEDs;
import frc.misc.SubsystemStatus;
import frc.misc.LEDs.LEDEnums;
import frc.robot.Robot;
import java.io.IOError;
import java.io.IOException;
import static frc.robot.Robot.arm;
import static frc.robot.Robot.robotSettings;

import java.io.IOError;
import java.io.IOException;
import java.util.Objects;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class ManipulationManager implements ISubsystem {
    public BaseController panel1, panel2, xbox2, midiTop, midiBot;
    public static LEDs leds;
    public double armGoal = 0;
    public double slippageOffSet = 0;
    public double elevateGoal = 2.2;
    public boolean cubeConeMode = true; // true = Cone, false = Cube
    public boolean spikeUp = false;
    int[] rgby = { 255, 255, 0 };
    int[] rgbp = { 138, 43, 226 };

    public ManipulationManager() {
        addToMetaList();
        init();
    }

    @Override
    public void init() {
        enableControllers();
        cubeConeMode = true;
        if (robotSettings.ENABLE_LEDS) {
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

        if (xbox2.get(DefaultControllerEnums.XBoxButtons.B_CIRCLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            robotSettings.ARM_ELEVATOR_MANUAL = true;
        }
        if (xbox2.get(DefaultControllerEnums.XBoxButtons.X_SQUARE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            robotSettings.ARM_ELEVATOR_MANUAL = false;
            Robot.intake.intakeIn();
        }
        if (xbox2.get(DefaultControllerEnums.XBoxButtons.A_CROSS) == DefaultControllerEnums.ButtonStatus.DOWN) {
            slippageOffSet = slippageOffSet - 0.2;
        }
        if (xbox2.get(DefaultControllerEnums.XBoxButtons.Y_TRIANGLE) == DefaultControllerEnums.ButtonStatus.DOWN) {
            slippageOffSet = slippageOffSet + 0.2;
        }

        switch (robotSettings.MANIPULATION_STYLE) {
            case STANDARD_2023: {

                if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.Cone) == ButtonStatus.DOWN) {
                    changeCubeCone(true);
                    try {
                        leds.yellow();
                    } catch (IOError e) {
                        System.out.println("Yellow LED not working: most likely bc ENABLE_LEDS is false");
                    }
                }
                if (panel2.get(ControllerEnums.ButtonPanelButtonsElse2023.Cube) == ButtonStatus.DOWN) {
                    changeCubeCone(false);
                    try {
                        leds.purple();
                    } catch (IOError e) {
                        System.out.println("Purple LED not working: most likely bc ENABLE_LEDS is false");
                    }
                }

                if (!robotSettings.ARM_ELEVATOR_MANUAL) {
                    // HumanPlayerStation
                    if (panel2.get(
                            ControllerEnums.ButtonPanelButtonsElse2023.GTStation1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -7;
                        armGoal = -54.5 + slippageOffSet;
                    }
                    if (panel2.get(
                            ControllerEnums.ButtonPanelButtonsElse2023.GTStation2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -4;
                        armGoal = -57 + slippageOffSet;
                    }
                    // stable
                    if (panel1.get(
                            ControllerEnums.ButtonPanelButtonsPlacement2023.Stable) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -45;
                        armGoal = -135 + slippageOffSet;
                    }
                    // floor
                    if (panel2.get(
                            ControllerEnums.ButtonPanelButtonsElse2023.Floor) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -37;
                        armGoal = -30 + slippageOffSet;
                    }
                    // climb
                    if (panel2.get(
                            ControllerEnums.ButtonPanelButtonsElse2023.Climb) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -44;
                        armGoal = -55 + slippageOffSet;
                    }
                    // lift arm out of starting NOT USED ON PANEL RN
                    // if (panel.get(ControllerEnums.ButtonPanelButtonsElse2023.) ==
                    // DefaultControllerEnums.ButtonStatus.DOWN) {
                    // Robot.elevator.moveElevator(2.2);
                    // armGoal = 0;
                    // }
                    // pick up off of spike
                    if (panel1.get(
                            ControllerEnums.ButtonPanelButtonsPlacement2023.SpikePickU) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = 0.7;
                        armGoal = -15.5 + slippageOffSet;
                        spikeUp = true;
                    }

                    if (!cubeConeMode) {
                        // place high
                        if (panel2.get(
                                ControllerEnums.ButtonPanelButtonsElse2023.High) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -10;
                            armGoal = -217 + slippageOffSet;
                        }
                        // place mid
                        if (panel2.get(
                                ControllerEnums.ButtonPanelButtonsElse2023.Mid) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -5;
                            armGoal = -235 + slippageOffSet;
                        }
                        // place low
                        // if (panel2.get(
                        //         ControllerEnums.ButtonPanelButtonsElse2023.Low) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        //     elevateGoal = 0;
                        //     armGoal = -270 + slippageOffSet;
                        // }
                    }
                    if (cubeConeMode) {
                        // place high
                        if (panel2.get(
                                ControllerEnums.ButtonPanelButtonsElse2023.High) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -1;
                            armGoal = -212 + slippageOffSet;
                        }
                        // place mid
                        if (panel2.get(
                                ControllerEnums.ButtonPanelButtonsElse2023.Mid) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -40;
                            armGoal = -195 + slippageOffSet;
                        }
                        // place low
                        // if (panel2.get(
                        //         ControllerEnums.ButtonPanelButtonsElse2023.Low) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        //     elevateGoal = -0;
                        //     armGoal = -270 + slippageOffSet;
                        // }

                    }
                }
                break;
            }

            case MIDI: {

                if (midiTop.get(ControllerEnums.MidiController.R1C5) == DefaultControllerEnums.ButtonStatus.DOWN)
                    changeCubeCone(true);
                if (midiTop.get(ControllerEnums.MidiController.R1C6) == DefaultControllerEnums.ButtonStatus.DOWN)
                    changeCubeCone(false);

                if (!robotSettings.ARM_ELEVATOR_MANUAL) {
                    // HumanPlayerStation
                    if (midiTop.get(ControllerEnums.MidiController.R1C3) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -11;
                        armGoal = -62;
                    }
                    // stable
                    if (midiTop.get(ControllerEnums.MidiController.R1C2) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -44;
                        armGoal = -133;
                    }
                    // lift arm out of starting
                    if (midiTop.get(ControllerEnums.MidiController.R4C4) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        Robot.elevator.moveElevator(2.2);
                        armGoal = 0;
                    }
                    // pick up off of spike
                    if (midiTop.get(ControllerEnums.MidiController.R2C3) == DefaultControllerEnums.ButtonStatus.DOWN) {
                        elevateGoal = -8.42;
                        armGoal = -15.8;
                        spikeUp = true;
                    }

                    if (!cubeConeMode) {
                        // place high
                        if (midiTop
                                .get(ControllerEnums.MidiController.R1C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -1;
                            armGoal = -240;
                        }
                        // place mid
                        if (midiTop
                                .get(ControllerEnums.MidiController.R2C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -2;
                            armGoal = -254;
                        }
                        // place low
                        if (midiTop
                                .get(ControllerEnums.MidiController.R3C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = 0;
                            armGoal = -290;
                        }
                    }
                    if (cubeConeMode) {
                        // place high
                        if (midiTop
                                .get(ControllerEnums.MidiController.R1C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -3;
                            armGoal = -226;
                        }
                        // place mid
                        if (midiTop
                                .get(ControllerEnums.MidiController.R2C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -44;
                            armGoal = -206;
                        }
                        // place low
                        if (midiTop
                                .get(ControllerEnums.MidiController.R3C1) == DefaultControllerEnums.ButtonStatus.DOWN) {
                            elevateGoal = -0;
                            armGoal = -290;
                        }

                    }
                }
                break;
            }
        }
        if (!robotSettings.ARM_ELEVATOR_MANUAL) {
            // Robot.arm.moveArm(armGoal);

            if (checkArmCollision()) {
            } else if (checkArmPassover()) {
            } else {
                if ((spikeUp)) {
                    Robot.elevator.moveElevator(elevateGoal);
                    if (Math.abs(-15.5 - Robot.arm.armRotationController.getRotations()) <= .2) {
                        elevateGoal = -10.3;
                        if (Math.abs(-10.3 - Robot.elevator.elevatorController.getRotations()) <= .2) {
                            Robot.intake.intakeIn();
                            spikeUp = false;
                        }
                    }
                } else {
                    Robot.elevator.moveElevator(elevateGoal);
                }
            }
        }

        if (robotSettings.ENABLE_WRIST) {
            // -75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200
            if (Robot.arm.armRotationController.getRotations() + slippageOffSet >= -123) {
                if (Robot.wrist.wristController.getRotations() >= 0
                        && Robot.wrist.wristController.getRotations() <= 4000) {
                    Robot.wrist.wristController.moveAtVoltage(0);
                } else {
                    Robot.wrist.wristController.moveAtVoltage(-6);
                }
            } else {
                if (Robot.wrist.wristController.getRotations() <= 4009
                        && Robot.wrist.wristController.getRotations() >= 10) {
                    Robot.wrist.wristController.moveAtVoltage(0);
                } else {
                    Robot.wrist.wristController.moveAtVoltage(6);
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
        panel1 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT2,
                BaseController.DefaultControllers.BUTTON_PANEL);
        panel2 = BaseController.createOrGet(robotSettings.BUTTON_PANEL_USB_SLOT3,
                BaseController.DefaultControllers.BUTTON_PANEL);
        xbox2 = BaseController.createOrGet(robotSettings.XBOX_CONTROLLER_USB_SLOT_2,
                BaseController.DefaultControllers.XBOX_CONTROLLER);
        midiTop = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_TOP_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
        midiBot = BaseController.createOrGet(robotSettings.MIDI_CONTROLLER_BOT_ID,
                BaseController.DefaultControllers.BUTTON_PANEL);
    }

    public boolean checkArmPassover() {
        if (-70 + slippageOffSet >= Robot.arm.armRotationController.getRotations()
                && Robot.arm.armRotationController.getRotations() >= -160 + slippageOffSet) {
            Robot.elevator.moveElevator(-44);
            return true;
        } else if (-60 + slippageOffSet >= Robot.arm.armRotationController.getRotations()
                && Robot.arm.armRotationController.getRotations() >= -178 + slippageOffSet) {
            Robot.elevator.moveElevator(-27);
            return true;
        }
        return false;
    }

    public boolean checkArmCollision() {
        if (Robot.arm.armRotationController.getRotations() >= 10 + slippageOffSet
                || Robot.arm.armRotationController.getRotations() <= -300 + slippageOffSet) {
            return true;
        }
        return false;
    }

    public boolean goTo(double elevator, double arm) {

        if (Math.abs(elevator - Robot.elevator.elevatorController.getRotations()) <= .2
                && Math.abs(arm - Robot.arm.armRotationController.getRotations()) <= .2) {
            return true;
        }
        Robot.arm.moveArm(arm);

        if (checkArmCollision()) {
        } else if (checkArmPassover()) {
        } else {
            Robot.elevator.moveElevator(elevator);
            System.out.println("tring to move to: " + elevator);
        }
        System.out.println("current elevator position: " + Robot.elevator.elevatorController.getRotations());

        if (robotSettings.ENABLE_WRIST) {
            // -75 >= Robot.arm.arm.getRotations() && Robot.arm.arm.getRotations() >= -200
            if (Robot.arm.armRotationController.getRotations() >= -125) {
                if (Robot.wrist.wristController.getRotations() >= 0
                        && Robot.wrist.wristController.getRotations() <= 4000) {
                    Robot.wrist.wristController.moveAtVoltage(0);
                } else {
                    Robot.wrist.wristController.moveAtVoltage(-6);
                }
            } else {
                if (Robot.wrist.wristController.getRotations() <= 4011
                        && Robot.wrist.wristController.getRotations() >= 10) {
                    Robot.wrist.wristController.moveAtVoltage(0);
                } else {
                    Robot.wrist.wristController.moveAtVoltage(6);
                }
            }
        }

        return false;
    }

    public void changeCubeCone(boolean cubeCone) {
        cubeConeMode = cubeCone;
    }

    public static void ledenum(int rgbl[]) {
        LEDEnums rgbEnums = LEDEnums.SOLID_COLOR_RGB;
    }

    public enum ManipulationControlStyles {
        STANDARD_2023, MIDI;

        private static SendableChooser<ManipulationControlStyles> myChooser;

        public static SendableChooser<ManipulationControlStyles> getSendableChooser() {
            return Objects.requireNonNullElseGet(myChooser, () -> {
                myChooser = new SendableChooser<>();
                for (ManipulationControlStyles style : ManipulationControlStyles.values())
                    myChooser.addOption(style.name(), style);
                return myChooser;
            });
        }
    }
}
