package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.WristConstants;

public class IntakeCoral extends SequentialCommandGroup {

    // Sadece shooter kullanan ve elevator/wrist konumunu değiştirmeyen alternatif
    // constructor
    public IntakeCoral(ShooterSubsystem shooterSubsystem) {
        addCommands(
                // 1. İntake motorunu çalıştır
                new InstantCommand(() -> shooterSubsystem.moveAtSpeed(-0.5), shooterSubsystem),

                // 2. Game element algılanana kadar bekle (max 3 saniye)
                new WaitCommand(3.0).until(() -> shooterSubsystem.hasGameElement()),

                // 3. Game element algılandığında motoru durdur
                new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem));
    }
}