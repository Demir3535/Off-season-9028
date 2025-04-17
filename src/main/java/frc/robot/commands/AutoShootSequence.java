package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.WristConstants;

public class AutoShootSequence extends SequentialCommandGroup {
    
    public AutoShootSequence(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, int level) {
        
        // Shooter seviyesine göre elevator ve wrist pozisyonlarını belirle
        double elevatorSetpoint;
        double wristSetpoint;
        
        switch (level) {
            case 1:
                elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L1;
                wristSetpoint = WristConstants.AngleSetpoints.Coral.L1;
                break;
            case 2:
                elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L2;
                wristSetpoint = WristConstants.AngleSetpoints.Coral.L2;
                break;
            case 3:
            default:
                elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L3;
                wristSetpoint = WristConstants.AngleSetpoints.Coral.L3;
                break;
        }
        
        // Komut dizisini oluştur - YÜKARIYA ÇIKARKEN ÖNCE BILEK SONRA ELEVATOR
        addCommands(
            // 1. Önce wrist'i doğru açıya getir
            new InstantCommand(() -> wristSubsystem.goToSetpoint(wristSetpoint), wristSubsystem),
            
            // 2. Wrist doğru pozisyona gelene kadar bekle
            new WaitCommand(0.5).until(() -> wristSubsystem.atSetpoint()),
            
            // 3. Sonra elevator'u doğru yüksekliğe getir
            new InstantCommand(() -> elevatorSubsystem.goToSetpoint(elevatorSetpoint), elevatorSubsystem),
            
            // 4. Elevator doğru pozisyona gelene kadar bekle
            new WaitCommand(0.5).until(() -> elevatorSubsystem.atSetpoint()),
            
            // 5. Atış yap
            new InstantCommand(() -> shooterSubsystem.moveAtSpeed(-0.50), shooterSubsystem),
            
            // 6. Game element atılana kadar bekle veya maximum 2 saniye
            new WaitCommand(2.0).until(() -> !shooterSubsystem.hasGameElement()),
            
            // 7. Shooter'ı durdur
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem),
            
            // İNERKEN ÖNCE ELEVATOR SONRA BILEK
            // 8. Önce elevator'u home pozisyonuna getir
            new InstantCommand(() -> elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME), elevatorSubsystem),
            
            // 9. Elevator home pozisyonuna gelene kadar bekle
            new WaitCommand(0.5).until(() -> elevatorSubsystem.atSetpoint()),
            
            // 10. Sonra wrist'i home pozisyonuna getir
            new InstantCommand(() -> wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME), wristSubsystem)
        );
    }
}