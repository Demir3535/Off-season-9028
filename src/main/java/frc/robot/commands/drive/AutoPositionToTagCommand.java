package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.RobotConstants.LimelightConstants;

public class AutoPositionToTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final DriveSubsystem drive;
    private final int targetTagID; // if -1 its connect to any tag
    
    // Hedef değerler
    private final double targetTx; // Hedef yatay açı
    private final double targetTa; // Hedef alan

    /**
     * Creates a command that automatically positions the robot to an AprilTag.
     * 
     * @param limelight   The limelight subsystem
     * @param drive       The drive subsystem
     * @param targetTagID The specific AprilTag ID to target, or -1 for any visible tag
     */
    public AutoPositionToTagCommand(LimelightSubsystem limelight, DriveSubsystem drive, int targetTagID) {
        this(limelight, drive, targetTagID, 0.0, LimelightConstants.DESIRED_TARGET);
    }
    
    /**
     * Creates a command that automatically positions the robot to an AprilTag with specific target values.
     * 
     * @param limelight   The limelight subsystem
     * @param drive       The drive subsystem
     * @param targetTagID The specific AprilTag ID to target, or -1 for any visible tag
     * @param targetTx    Hedef yatay açı (0 = merkez)
     * @param targetTa    Hedef alan (büyük değer = daha yakın)
     */
    public AutoPositionToTagCommand(LimelightSubsystem limelight, DriveSubsystem drive, 
                                   int targetTagID, double targetTx, double targetTa) {
        this.limelight = limelight;
        this.drive = drive;
        this.targetTagID = targetTagID; // specific id or any id for -1
        this.targetTx = targetTx;
        this.targetTa = targetTa;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Log that we're starting the command
        System.out.println("Starting AutoPositionToTag command" +
                (targetTagID == -1 ? " for any tag" : " for tag ID: " + targetTagID) +
                " with target tx: " + targetTx + " and target area: " + targetTa);
    }
    
    @Override
    public void execute() {
        if (limelight.hasTargets()) {
            // Check if we're targeting a specific tag and that's what we see
            if (targetTagID != -1 && limelight.getTargetID() != targetTagID) {
                // We see a tag, but not the one we want - stop
                drive.drive(0, 0, 0, true, true);
                return;
            }
            
            // Mevcut Limelight değerlerini al
            double currentTx = limelight.getTx();
            double currentTa = limelight.getTa();
            
            // SmartDashboard'a değerleri yaz
            SmartDashboard.putNumber("Current TX", currentTx);
            SmartDashboard.putNumber("Target TX", targetTx);
            SmartDashboard.putNumber("Current TA", currentTa);
            SmartDashboard.putNumber("Target TA", targetTa);
            
            // Tx hatasını hesapla
            double txError = currentTx - targetTx;
            // Steer hesapla (hedef tx'e göre)
            double steer = -txError * LimelightConstants.STEER_K;
            
            // Yatay hizalama durumunu kontrol et
            boolean isAligned = Math.abs(txError) < LimelightConstants.ACCEPTABLE_TX_ERROR;
            
            // İleri/geri hızını hesapla
            double driveSpeed = 0;
            
            // Önce hizalan, sonra yaklaş
            if (isAligned) {
                // Hizalanmış durumdayız, şimdi mesafeyi kontrol et
                
                // Hedef alana göre hızı ayarla
                if (currentTa < targetTa) {
                    // Hedefe ulaşmadık, ileri git
                    double distanceRatio = 1.0 - (currentTa / targetTa);
                    driveSpeed = LimelightConstants.MAX_DRIVE_SPEED * distanceRatio;
                    
                    // Minimum hız ayarla
                    driveSpeed = Math.max(driveSpeed, LimelightConstants.MAX_DRIVE_SPEED * 0.1);
                } else if (currentTa > targetTa * 1.1) {
                    // Hedefin çok ötesindeyiz, geri git
                    double distanceRatio = (currentTa / targetTa) - 1.0;
                    driveSpeed = -LimelightConstants.MAX_DRIVE_SPEED * distanceRatio;
                    driveSpeed = Math.min(driveSpeed, -LimelightConstants.MAX_DRIVE_SPEED * 0.1);
                }
            } else {
                // Hizalanmadıysak, sadece dönüş yap, ileri gitme
                driveSpeed = 0;
            }
            
            // Robotun dönüş hızını sınırla
            steer = Math.max(-LimelightConstants.MAX_TURN_SPEED, Math.min(steer, LimelightConstants.MAX_TURN_SPEED));
            
            // Hareket ve dönüşü uygula
            drive.drive(driveSpeed, 0, steer, true, true);
            
            // Hata ve hız bilgilerini SmartDashboard'a yaz
            SmartDashboard.putNumber("TX Error", txError);
            SmartDashboard.putNumber("Drive Speed", driveSpeed);
            SmartDashboard.putNumber("Steer Value", steer);
            SmartDashboard.putBoolean("Is Aligned", isAligned);
            
        } else {
            // Hedef yok - robotu durdur
            drive.drive(0, 0, 0, true, true);
        }
    }

    @Override
    public boolean isFinished() {
        // Komut şu durumlarda biter:
        // 1. Bir hedefimiz var
        // 2. Hedefle hizalandık (tx hedef değere yakın)
        // 3. İstenen mesafedeyiz (alan hedef değere yakın)
        // 4. Belirli bir tag hedefliyorsak, o tag'i görüyoruz

        if (!limelight.hasTargets()) {
            return false; // Hedef yok, çalışmaya devam et
        }

        if (targetTagID != -1 && limelight.getTargetID() != targetTagID) {
            return false; // Bir tag görüyoruz, ama istediğimiz değil
        }

        boolean alignedWithTarget = Math.abs(limelight.getTx() - targetTx) < LimelightConstants.ACCEPTABLE_TX_ERROR;
        boolean atDesiredDistance = Math.abs(limelight.getTa() - targetTa) < (targetTa * 0.1); // %10 tolerans

        return alignedWithTarget && atDesiredDistance;
    }

    @Override
    public void end(boolean interrupted) {
        // Komut bittiğinde robotu durdur
        drive.drive(0, 0, 0, true, true);

        if (interrupted) {
            System.out.println("AutoPositionToTag command was interrupted");
        } else {
            System.out.println("AutoPositionToTag command completed successfully");
        }
    }
}