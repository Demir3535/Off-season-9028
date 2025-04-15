

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;



public class AutoAlignToAprilTagCommand extends Command {
    private final LimelightSubsystem limelightSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final PIDController steerController;
    private final PIDController distanceController;
    private final PIDController strafeController;
    
    public AutoAlignToAprilTagCommand(LimelightSubsystem limelight, DriveSubsystem drive) {
        this.limelightSubsystem = limelight;
        this.driveSubsystem = drive;
        
        // Döndürme için PID kontrolcüsü (tag'e dik olarak bakmak için)
        steerController = new PIDController(0.03, 0.0, 0.0);
        steerController.setTolerance(1.0); // 1 derece tolerans
        
        // Mesafe için PID kontrolcüsü
        distanceController = new PIDController(0.05, 0.0, 0.0);
        distanceController.setTolerance(0.1); // 10 cm tolerans
        
        // Yana kayma için PID kontrolcüsü (merkeze hizalanmak için)
        strafeController = new PIDController(0.04, 0.0, 0.0);
        strafeController.setTolerance(1.0); // 1 derece tolerans
        
        addRequirements(limelight, drive);
    }
    
    @Override
    public void execute() {
        if (!limelightSubsystem.hasTargets()) {
            driveSubsystem.drive(0, 0, 0, true, false);
            return;
        }
        
        // Tx - yatay açı. Bu 0 olmalı (robot tag'e dik bakmalı)
        double rotation = -steerController.calculate(limelightSubsystem.getTx(), 0);
        
        // Mesafe kontrolü - istediğiniz mesafeye yaklaşma
        double targetDistance = 1.0; // 1 metre mesafede durmak istediğinizi varsayalım
        double forward = distanceController.calculate(limelightSubsystem.getDistanceToTarget(), targetDistance);
        
        // Yana kayma kontrolü - tag'e merkezi hizalama 
        // (Bu önemli - tag'in tam karşısında olmak için)
        double strafe = strafeController.calculate(limelightSubsystem.getTx(), 0);
        
        // Hızları sınırla
        forward = MathUtil.clamp(forward, -0.3, 0.3);
        rotation = MathUtil.clamp(rotation, -0.3, 0.3);
        strafe = MathUtil.clamp(strafe, -0.3, 0.3);
        
        // Field-relative drive (alan bağlantılı sürüş)
        driveSubsystem.drive(forward, strafe, rotation, true, false);
    }
    
    @Override
    public boolean isFinished() {
        // Eğer hem açı hem de mesafe doğruysa ve hedef algılanıyorsa bitir
        return (steerController.atSetpoint() && 
                distanceController.atSetpoint() && 
                strafeController.atSetpoint() &&
                limelightSubsystem.hasTargets());
    }
    
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true, false);
    }
}