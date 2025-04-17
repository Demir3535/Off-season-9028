package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants.PortConstants.CAN;
import frc.robot.RobotConstants.PortConstants.DIO;
import frc.robot.RobotConstants.WristConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.PWM;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;;

public class ShooterSubsystem extends SubsystemBase {
    private SparkMax shooterMotor;
    private DigitalInput limitSwitch;
    private boolean isShooterRunning = false;
    private boolean gameElementDetected = false;
    private PWM blinkinController;
    private static final double RED = -0.41;
    private static final double GREEN = -0.05;
    public boolean isShooting = false;
    public boolean readyToShoot = false; // Is game piece ready?
    private boolean wasGameElementDetected = false; // Önceki durumu takip etmek için
    private WristSubsystem wristSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    
    
    public ShooterSubsystem(WristSubsystem wristSubsystem, ElevatorSubsystem elevatorSubsystem) {
        limitSwitch = new DigitalInput(DIO.SHOOTER_DISTANCE_SENSOR);
        shooterMotor = new SparkMax(CAN.SHOOTER_MOTOR_1, MotorType.kBrushless);
        blinkinController = new PWM(0);
           this.wristSubsystem = wristSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void periodic() {
        // Check limit switch status
        gameElementDetected = limitSwitch.get();
        if (gameElementDetected && !wasGameElementDetected) {
            // Game piece yeni tespit edildi, wrist'i pozisyona getir
            wristSubsystem.goToSetpoint(WristConstants.AngleSetpoints.Coral.L3);
        }

        // Game element atıldıktan sonra (yani gameElementDetected false olduğunda)
        if (!gameElementDetected && wasGameElementDetected) {
            // Game piece artık yok, önce elevator sonra wrist home pozisyonuna gelsin
            autoReturnToHomePosition();
        }

        wasGameElementDetected = gameElementDetected;

        // If game piece is detected and motor is in intake mode, stop the motor
        if (isShooting && !gameElementDetected) {
            stopShooter();
            isShooting = false;

        } else if (isShooting) {

        }

        else if (gameElementDetected && isShooterRunning && !readyToShoot) {

            stopShooter();
            readyToShoot = true; // Ready to shoot
        }
        updateLEDStatus();
        // Debug information
        SmartDashboard.putBoolean("Limit Switch Status", limitSwitch.get());
        SmartDashboard.putBoolean("Is Shooter Running", isShooterRunning);
        SmartDashboard.putBoolean("Game Element Detected", gameElementDetected);
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot);
    }
private void autoReturnToHomePosition() {
    // SequentialCommandGroup oluştur ve çalıştır
    new SequentialCommandGroup(
        // Önce elevator'un home pozisyonuna inmesini bekle
        elevatorSubsystem.goToCoralScoreSetpoint(0), // 0 = Home pozisyonu
        // Sonra wrist'in home pozisyonuna gelmesini sağla
      new WaitUntilCommand(wristSubsystem::atSetpoint),

        wristSubsystem.goToCoralScoreSetpoint(0)     // 0 = Home pozisyonu
    ).schedule();
}
    private void updateLEDStatus() {
        if (readyToShoot) {
            blinkinController.setSpeed(GREEN); // Yeşil renk
        } else if (!gameElementDetected) {
            blinkinController.setSpeed(RED); // Kırmızı renk
        }
    }

    public void shooterButton() {
        // Debug info
        SmartDashboard.putBoolean("Button Pressed", true);
        SmartDashboard.putNumber("Motor Output", shooterMotor.get());

        if (!gameElementDetected && !isShooterRunning) {
            SmartDashboard.putString("Shooter State", "Starting Intake");
            moveAtSpeed(-0.50);
            readyToShoot = false;
        } else if (readyToShoot && gameElementDetected) {
            isShooting = true;
            SmartDashboard.putString("Shooter State", "Shooting");
            moveAtSpeed(-0.50);
            readyToShoot = false;
        } else if (isShooterRunning) {
            SmartDashboard.putString("Shooter State", "Stopping");
            stopShooter();
        }
    }

    public Command ShootAuto() {
        return new InstantCommand(() -> {
            moveAtSpeed(1);
        }, this);
    }

    public void reverseShooter() {
        SmartDashboard.putBoolean("Reverse Button Pressed", true);
        SmartDashboard.putString("Shooter State", "Reversing");

        moveAtSpeed(0.40);

        readyToShoot = false;
    }

    public void emergencyShoot() {
        SmartDashboard.putString("Shooter State", "Emergency Shooting");
        // Limit switch'in durumuna bakmaksızın doğrudan atış yapar
        moveAtSpeed(-0.50); // veya istediğiniz başka bir değer
        isShooting = true;
        readyToShoot = false;
    }

    public Command emergencyShootCommand() {
        return new InstantCommand(() -> {
            emergencyShoot();
        }, this);
    }

    public void moveAtSpeed(double speed) {
        shooterMotor.set(speed);
        isShooterRunning = true;
    }

    public Command intakeCoral() {
        return new InstantCommand(() -> {
            moveAtSpeed(.5);
        }, this);
    }

    public Command shootCoral() {
        return new InstantCommand(() -> {
            moveAtSpeed(1);
        }, this);
    }

    public void stopShooter() {
        shooterMotor.set(0);
        isShooterRunning = false;
    }

    public Command PPstopshooter() {
        return new InstantCommand(() -> {
            moveAtSpeed(0);
        }, this);
    }

    public boolean isShooterRunning() {
        return isShooterRunning;
    }

    public boolean hasGameElement() {
        return gameElementDetected;
    }
}