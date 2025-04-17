package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.RobotConstants.ElevatorConstants;
import frc.robot.RobotConstants.WristConstants;

public class AutoGrabAndShoot extends Command {
    
    private final ShooterSubsystem shooterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final WristSubsystem wristSubsystem;
    private final int level;
    
    private enum State {
        POSITIONING_FOR_INTAKE,   // Intake için konumlandırma
        INTAKE,                  // Gamepiece alınıyor
        WAITING_FOR_GAMEPIECE,   // Gamepiece'in algılanmasını bekliyoruz
        POSITIONING_WRIST,       // Bilek atış pozisyonuna geçiyor
        POSITIONING_ELEVATOR,    // Elevator atış pozisyonuna geçiyor
        SHOOTING,                // Atış yapılıyor
        RETURNING_ELEVATOR,      // Elevator home pozisyonuna dönüyor
        RETURNING_WRIST,         // Bilek home pozisyonuna dönüyor
        DONE                     // İşlem tamamlandı
    }
    
    private State currentState = State.POSITIONING_FOR_INTAKE;
    private Timer stateTimer = new Timer();
    private boolean wasGameElementDetected = false;
    
    public AutoGrabAndShoot(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem, 
                           WristSubsystem wristSubsystem, int level) {
        this.shooterSubsystem = shooterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.wristSubsystem = wristSubsystem;
        this.level = level;
        
        addRequirements(shooterSubsystem, elevatorSubsystem, wristSubsystem);
    }
    
    @Override
    public void initialize() {
        // Başlangıçta gamepiece var mı kontrol et
        if (shooterSubsystem.hasGameElement()) {
            // Gamepiece zaten var, doğrudan bilek konumlandırma aşamasına geç
            currentState = State.POSITIONING_WRIST;
            
            // Bilek pozisyonuna geç (önce bilek)
            double wristSetpoint;
            
            switch (level) {
                case 1:
                    wristSetpoint = WristConstants.AngleSetpoints.Coral.L1;
                    break;
                case 2:
                    wristSetpoint = WristConstants.AngleSetpoints.Coral.L2;
                    break;
                case 3:
                default:
                    wristSetpoint = WristConstants.AngleSetpoints.Coral.L3;
                    break;
            }
            
            wristSubsystem.goToSetpoint(wristSetpoint);
        } else {
            // Gamepiece yok, intake sekansına başla
            currentState = State.POSITIONING_FOR_INTAKE;
            
            // İlk olarak intake pozisyonuna geçeriz
            elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
            wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
        }
        
        // Zamanlayıcıyı başlat
        stateTimer.reset();
        stateTimer.start();
        
        // Gamepiece algılama durumunu kaydet
        wasGameElementDetected = shooterSubsystem.hasGameElement();
    }
    
    @Override
    public void execute() {
        // Mevcut gamepiece algılama durumunu kontrol et
        boolean isGameElementDetected = shooterSubsystem.hasGameElement();
        
        // Algılama durumu değiştiğinde özel işlemler
        if (isGameElementDetected && !wasGameElementDetected) {
            // Yeni gamepiece algılandı - özel işlemler yapabilirsiniz
        } else if (!isGameElementDetected && wasGameElementDetected) {
            // Gamepiece artık algılanmıyor (atıldı) - özel işlemler yapabilirsiniz
        }
        
        // Algılama durumunu güncelle
        wasGameElementDetected = isGameElementDetected;
        
        // Durum makinesi
        switch (currentState) {
            case POSITIONING_FOR_INTAKE:
                // Elevator ve wrist intake pozisyonuna geldiyse, intake'e başla
                if (elevatorSubsystem.atSetpoint() && wristSubsystem.atSetpoint()) {
                    shooterSubsystem.moveAtSpeed(-0.5); // Intake başlat
                    currentState = State.INTAKE;
                    stateTimer.reset();
                }
                
                // Zaman aşımı kontrolü
                if (stateTimer.get() > 3.0) {
                    // 3 saniye içinde konumlandırma tamamlanmadı, tekrar deneyelim
                    elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
                    wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
                    stateTimer.reset();
                }
                break;
                
            case INTAKE:
                // Intake başladı, gamepiece beklemesine geç
                currentState = State.WAITING_FOR_GAMEPIECE;
                stateTimer.reset();
                break;
                
            case WAITING_FOR_GAMEPIECE:
                // Game piece algılandı mı?
                if (isGameElementDetected) {
                    // Game piece algılandı, intake'i durdur ve bilek konumlandırma aşamasına geç
                    shooterSubsystem.stopShooter();
                    
                    // Bilek atış pozisyonuna geç (önce bilek)
                    double wristSetpoint;
                    
                    switch (level) {
                        case 1:
                            wristSetpoint = WristConstants.AngleSetpoints.Coral.L1;
                            break;
                        case 2:
                            wristSetpoint = WristConstants.AngleSetpoints.Coral.L2;
                            break;
                        case 3:
                        default:
                            wristSetpoint = WristConstants.AngleSetpoints.Coral.L3;
                            break;
                    }
                    
                    wristSubsystem.goToSetpoint(wristSetpoint);
                    
                    currentState = State.POSITIONING_WRIST;
                    stateTimer.reset();
                }
                
                // Zaman aşımı kontrolü
                if (stateTimer.get() > 5.0) {
                    // 5 saniye içinde gamepiece algılanmadı, intake'i durdur ve yeniden başlat
                    shooterSubsystem.stopShooter();
                    currentState = State.POSITIONING_FOR_INTAKE;
                    stateTimer.reset();
                }
                break;
                
            case POSITIONING_WRIST:
                // Bilek pozisyona geldiyse, elevator konumlandırma aşamasına geç
                if (wristSubsystem.atSetpoint()) {
                    // Elevator atış pozisyonuna geç (sonra elevator)
                    double elevatorSetpoint;
                    
                    switch (level) {
                        case 1:
                            elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L1;
                            break;
                        case 2:
                            elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L2;
                            break;
                        case 3:
                        default:
                            elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L3;
                            break;
                    }
                    
                    elevatorSubsystem.goToSetpoint(elevatorSetpoint);
                    
                    currentState = State.POSITIONING_ELEVATOR;
                    stateTimer.reset();
                }
                
                // Zaman aşımı kontrolü
                if (stateTimer.get() > 3.0) {
                    // 3 saniye içinde bilek konumlandırma tamamlanmadı, tekrar deneyelim
                    double wristSetpoint;
                    
                    switch (level) {
                        case 1:
                            wristSetpoint = WristConstants.AngleSetpoints.Coral.L1;
                            break;
                        case 2:
                            wristSetpoint = WristConstants.AngleSetpoints.Coral.L2;
                            break;
                        case 3:
                        default:
                            wristSetpoint = WristConstants.AngleSetpoints.Coral.L3;
                            break;
                    }
                    
                    wristSubsystem.goToSetpoint(wristSetpoint);
                    stateTimer.reset();
                }
                break;
                
            case POSITIONING_ELEVATOR:
                // Elevator pozisyona geldiyse atış yap
                if (elevatorSubsystem.atSetpoint()) {
                    shooterSubsystem.moveAtSpeed(-0.5); // Atış yap
                    currentState = State.SHOOTING;
                    stateTimer.reset();
                }
                
                // Zaman aşımı kontrolü
                if (stateTimer.get() > 3.0) {
                    // 3 saniye içinde elevator konumlandırma tamamlanmadı, tekrar deneyelim
                    double elevatorSetpoint;
                    
                    switch (level) {
                        case 1:
                            elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L1;
                            break;
                        case 2:
                            elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L2;
                            break;
                        case 3:
                        default:
                            elevatorSetpoint = ElevatorConstants.HeightSetpoints.Coral.L3;
                            break;
                    }
                    
                    elevatorSubsystem.goToSetpoint(elevatorSetpoint);
                    stateTimer.reset();
                }
                break;
                
            case SHOOTING:
                // Game piece atıldı mı?
                if (!isGameElementDetected) {
                    // Atış tamamlandı, motorları durdur
                    shooterSubsystem.stopShooter();
                    
                    // Önce elevator'u home pozisyonuna getir
                    elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
                    
                    currentState = State.RETURNING_ELEVATOR;
                    stateTimer.reset();
                }
                
                // Zaman aşımı kontrolü
                if (stateTimer.get() > 3.0) {
                    // 3 saniye içinde atış tamamlanmadı, motorları durdur ve dönmeye başla
                    shooterSubsystem.stopShooter();
                    
                    // Önce elevator'u home pozisyonuna getir
                    elevatorSubsystem.goToSetpoint(ElevatorConstants.HeightSetpoints.HOME);
                    
                    currentState = State.RETURNING_ELEVATOR;
                    stateTimer.reset();
                }
                break;
                
            case RETURNING_ELEVATOR:
                // Elevator home pozisyonuna geldiyse, bilek'i home pozisyonuna getir
                if (elevatorSubsystem.atSetpoint()) {
                    wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
                    
                    currentState = State.RETURNING_WRIST;
                    stateTimer.reset();
                }
                
                // Zaman aşımı kontrolü
                if (stateTimer.get() > 3.0) {
                    // 3 saniye içinde elevator dönüş tamamlanmadı, bilek'e geç
                    wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.HOME);
                    
                    currentState = State.RETURNING_WRIST;
                    stateTimer.reset();
                }
                break;
                
            case RETURNING_WRIST:
                // Bilek home pozisyonuna geldiyse işlemi tamamla
                if (wristSubsystem.atSetpoint()) {
                    currentState = State.DONE;
                }
                
                // Zaman aşımı kontrolü
                if (stateTimer.get() > 3.0) {
                    // 3 saniye içinde bilek dönüş tamamlanmadı, doğrudan tamamla
                    currentState = State.DONE;
                }
                break;
                
            case DONE:
                // İşlem tamamlandı
                break;
        }
    }
    
    @Override
    public boolean isFinished() {
        return currentState == State.DONE;
    }
    
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        stateTimer.stop();
    }
}