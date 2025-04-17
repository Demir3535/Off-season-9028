package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AutoPositionToTagCommand;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class AutoPositionAndShoot extends SequentialCommandGroup {
    
    public AutoPositionAndShoot(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem, 
                               ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, 
                               WristSubsystem wristSubsystem, int scoreLevel, int targetId) {
        
        // Komut dizisini oluştur
        addCommands(
            // 1. AprilTag'e göre hizalanma
            new AutoPositionToTagCommand(limelightSubsystem, driveSubsystem, targetId, -2.0, 8.0),
            
            // 2. Atış sekansını başlat
            new AutoShootSequence(shooterSubsystem, elevatorSubsystem, wristSubsystem, scoreLevel)
        );
    }
}