// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.elevator.ElevatorSubsystem;

import org.json.simple.JSONArray;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.drive.AutoPositionToTagCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.RobotSystemsCheckCommand;
import frc.robot.commands.drive.TeleopDriveCommand;
import frc.robot.commands.elevator.MoveElevatorManual;
import frc.robot.commands.wrist.MoveWristManual;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.automation.AutomationSelector;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.WristConstants;
import frc.robot.automation.AutomatedScoring;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.drive.LimelightDriveCommand;
//Subsystem imports
//Command imports

public class RobotContainer {

        private final DriveSubsystem m_drive = new DriveSubsystem();
        public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        public final WristSubsystem wristSubsystem = new WristSubsystem();
        public static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
        public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(wristSubsystem, elevatorSubsystem);
        public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
        private final Joystick driveJoystick = new Joystick(RobotConstants.PortConstants.Controller.DRIVE_JOYSTICK);
        private final Joystick operatorJoystick = new Joystick(
                        RobotConstants.PortConstants.Controller.OPERATOR_JOYSTICK);
        public final AutomationSelector automationSelector = new AutomationSelector();

        SendableChooser<Command> m_autoPositionChooser = new SendableChooser<>();

        PowerDistribution pdp;

        private final Field2d field = new Field2d();

        public RobotContainer() {
                m_drive.setDefaultCommand(new TeleopDriveCommand(m_drive, driveJoystick));

                elevatorSubsystem.setDefaultCommand(new MoveElevatorManual(elevatorSubsystem, operatorJoystick));
                wristSubsystem.setDefaultCommand(new MoveWristManual(wristSubsystem, operatorJoystick));

                createNamedCommands();

                configureButtonBindings();

                try {
                        pdp = new PowerDistribution(CAN.PDH, ModuleType.kRev);
                        m_autoPositionChooser = AutoBuilder.buildAutoChooser("Test Auto");
                        Shuffleboard.getTab("Autonomous Selection").add(m_autoPositionChooser);
                        Shuffleboard.getTab("Power").add(pdp);
                } catch (Exception e) {
                        e.printStackTrace();
                }
        }

        private void updateMatchTime() {
                // Maç süresini al
                double matchTime = DriverStation.getMatchTime();

                // Shuffleboard'a yaz
                SmartDashboard.putNumber("Match Time", matchTime);
        }

        private void createNamedCommands() {
                // Add commands here to be able to execute in auto through pathplanner

                NamedCommands.registerCommand("Example", new RunCommand(() -> {
                        System.out.println("Running...");
                }));
                NamedCommands.registerCommand("Score L1",
                                AutomatedScoring.scoreCoralNoPathing(1, elevatorSubsystem, wristSubsystem));
                NamedCommands.registerCommand("Score L2",
                                AutomatedScoring.scoreCoralNoPathing(2, elevatorSubsystem, wristSubsystem));
                NamedCommands.registerCommand("Score L3",
                                AutomatedScoring.scoreCoralNoPathing(3, elevatorSubsystem, wristSubsystem));
                NamedCommands.registerCommand("CoralIn", shooterSubsystem.intakeCoral());
                NamedCommands.registerCommand("CoralOut", shooterSubsystem.shootCoral());
                NamedCommands.registerCommand("ProcessorHome",
                                AutomatedScoring.homeSubsystems(elevatorSubsystem, wristSubsystem));
                NamedCommands.registerCommand("StopShooter", shooterSubsystem.PPstopshooter());
                NamedCommands.registerCommand("Wrist Shooter",
                                AutomatedScoring.scoreWristCoralPathing(2, wristSubsystem));
                NamedCommands.registerCommand("Wrist Home", AutomatedScoring.scoreWristCoralPathing(0, wristSubsystem));

                NamedCommands.registerCommand("AutoShoot",
                                new AutoShootCommand(shooterSubsystem, 1.0, 2.0)); // 1.0 hızda, 2 saniye süreyle atış
                NamedCommands.registerCommand("AutoIntake",
                                new AutoIntakeCommand(shooterSubsystem, 0.25, 3.0));
               
        }

        private void configureButtonBindings() {

                new JoystickButton(driveJoystick, 9).onChange(m_drive.xCommand()); // Needs to be while true so the
                                                                                   // command ends
                new JoystickButton(driveJoystick, 4).whileTrue(m_drive.gyroReset());

                new POVButton(driveJoystick, 90)
                                .onTrue(AutomatedScoring.scoreCoralNoPathing(2, elevatorSubsystem, wristSubsystem));
                new JoystickButton(driveJoystick, 1)
                                .whileTrue(new LimelightDriveCommand(m_drive, driveJoystick, limelightSubsystem));

                new JoystickButton(driveJoystick, 2).whileTrue(
                                new AutoPositionToTagCommand(limelightSubsystem, m_drive, -1) // Any AprilTag
                );
                new JoystickButton(operatorJoystick, 4)
                                .onTrue(new InstantCommand(() -> {
                                        SmartDashboard.putBoolean("Button 4 Button Pressed", true);

                                        shooterSubsystem.shooterButton();
                                }));

                new JoystickButton(operatorJoystick, 1)
                                .whileTrue(new InstantCommand(() -> {
                                        SmartDashboard.putBoolean("KARE  Button Pressed", true);
                                        shooterSubsystem.reverseShooter();

                                }));

                new JoystickButton(operatorJoystick, 5)
                                .onTrue(shooterSubsystem.emergencyShootCommand());
                // new JoystickButton(operatorJoystick, 1)
                // .whileTrue(new InstantCommand(() -> {
                // SmartDashboard.putBoolean("Button 1 Button Pressed", true);

                // shooterSubsystem.shooterButtonNoLimit();
                // }));

                new JoystickButton(operatorJoystick, 1).whileFalse(new InstantCommand(() -> {
                        shooterSubsystem.stopShooter();
                }));

                /*
                 * new POVButton(operatorJoystick, 0)
                 * .whileTrue(AutomatedScoring.scoreCoralNoPathing(3, elevatorSubsystem,
                 * wristSubsystem));
                 * new POVButton(operatorJoystick, 90)
                 * .whileTrue(AutomatedScoring.scoreCoralNoPathing(2, elevatorSubsystem,
                 * wristSubsystem));
                 * new POVButton(operatorJoystick, 180)
                 * .whileTrue(AutomatedScoring.scoreCoralNoPathing(1, elevatorSubsystem,
                 * wristSubsystem));
                 * new POVButton(operatorJoystick, 270)
                 * .whileTrue(AutomatedScoring.scoreCoralNoPathing(0, elevatorSubsystem,
                 * wristSubsystem));
                 */
                new POVButton(operatorJoystick, 0) // L3
                                .whileTrue(AutomatedScoring.scoreWithSafeSequence(3, wristSubsystem,
                                                elevatorSubsystem));
                new POVButton(operatorJoystick, 90) // L2
                                .whileTrue(AutomatedScoring.scoreWithSafeSequence(2, wristSubsystem,
                                                elevatorSubsystem));
                new POVButton(operatorJoystick, 180) // L1
                                .whileTrue(AutomatedScoring.scoreWithSafeSequence(1, wristSubsystem,
                                                elevatorSubsystem));

                new POVButton(operatorJoystick, 270) // intake
                                .onTrue(AutomatedScoring.scoreWithSafeSequence(0, wristSubsystem, elevatorSubsystem));

                new JoystickButton(operatorJoystick, 5)
                                .onTrue(AutomatedScoring.scoreWithSafeSequence(4, wristSubsystem, elevatorSubsystem));
                new JoystickButton(operatorJoystick, 7)
                                .onTrue(AutomatedScoring.scoreWithSafeSequence(5, wristSubsystem, elevatorSubsystem));

                /*new JoystickButton(operatorJoystick, 5)
                                .whileTrue(AutomatedScoring.scoreElevatorAlgea(2, elevatorSubsystem));

                new JoystickButton(operatorJoystick, 7)
                                .whileTrue(AutomatedScoring.scoreElevatorAlgea(3, elevatorSubsystem));

                new JoystickButton(operatorJoystick, 6)
                                .whileTrue(AutomatedScoring.scoreWristAlgeaPathing(1, wristSubsystem));
                new JoystickButton(operatorJoystick, 2)
                                .whileTrue(AutomatedScoring.scoreWristCoralPathing(0, wristSubsystem));

                new JoystickButton(operatorJoystick, 3)
                                .whileTrue(AutomatedScoring.scoreWristCoralPathing(3, wristSubsystem));

                new JoystickButton(operatorJoystick, 8)
                                .whileTrue(AutomatedScoring.scoreElevatorAlgea(5, elevatorSubsystem));*/

                new JoystickButton(driveJoystick, 3).whileTrue(
                                new AutoPositionToTagCommand(limelightSubsystem, m_drive, -1, -2.0, 8.0)
                // ti tx ta
                );
                // elevator binds

                // new POVButton(operatorJoystick, 270) // POV 90 derece (sağ yön)
                // .whileTrue(AutomatedScoring.intakePosition(elevatorSubsystem,
                // wristSubsystem));

        }

        public Command getAutonomousCommand() {
                if (m_autoPositionChooser.getSelected() != null) {
                        return m_autoPositionChooser.getSelected();
                } else {
                        return m_drive.gyroReset();
                }
        }
        public Command getTestingCommand() {
                return new RobotSystemsCheckCommand(m_drive);
        }

        public Field2d getField() {
                return field;
        }

        public static final class UserPolicy {
                public static boolean xLocked = false;
                public static boolean isManualControlled = true;
        }
}