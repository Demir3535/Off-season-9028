package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.WristConstants;

public class AutoShootFromStart extends SequentialCommandGroup {
    
    public AutoShootFromStart(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, int level) {
        
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
        
        // Komut dizisini oluştur
        addCommands(
            // 1. İçeri almak için shooter'ı çalıştır
            new InstantCommand(() -> shooterSubsystem.moveAtSpeed(-0.3), shooterSubsystem),
            
            // 2. Game element algılanana kadar bekle (max 2 saniye)
            new WaitCommand(2.0).until(() -> shooterSubsystem.hasGameElement()),
            
            // 3. Game element algılandığında shooter'ı durdur
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem),
            
            // YÜKARIYA ÇIKARKEN ÖNCE BILEK SONRA ELEVATOR
            // 4. Önce wrist'i doğru açıya getir
            new InstantCommand(() -> wristSubsystem.goToSetpoint(wristSetpoint), wristSubsystem),
            
            // 5. Wrist doğru pozisyona gelene kadar bekle
            new WaitCommand(0.5).until(() -> wristSubsystem.atSetpoint()),
            
            // 6. Sonra elevator'u doğru yüksekliğe getir
            new InstantCommand(() -> elevatorSubsystem.goToSetpoint(elevatorSetpoint), elevatorSubsystem),
            
            // 7. Elevator doğru pozisyona gelene kadar bekle
            new WaitCommand(0.5).until(() -> elevatorSubsystem.atSetpoint()),
            
            // 8. Atış yap
            new InstantCommand(() -> shooterSubsystem.moveAtSpeed(-0.50), shooterSubsystem),
            
            // 9. Game element atılana kadar bekle veya maximum 2 saniye
            new WaitCommand(2.0).until(() -> !shooterSubsystem.hasGameElement()),
            
            // 10. Shooter'ı durdur
            new InstantCommand(() -> shooterSubsystem.stopShooter(), shooterSubsystem),
            
            // İNERKEN ÖNCE ELEVATOR SONRA BILEK
            // 11. Önce elevator'u home pozisyonuna getir
            new InstantCommand(() -> elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME), elevatorSubsystem),
            
            // 12. Elevator home pozisyonuna gelene kadar bekle
            new WaitCommand(0.5).until(() -> elevatorSubsystem.atSetpoint()),
            
            // 13. Sonra wrist'i home pozisyonuna getir
            new InstantCommand(() -> wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME), wristSubsystem)
        );
    }
}