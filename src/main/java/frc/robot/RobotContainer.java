package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.IntakeSubsystem;
// İttifak rengi için gerekli

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveTrainSubsystems m_robotDrive = new DriveTrainSubsystems();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final ConveyorSubsystem m_conveyorSubsystem = new ConveyorSubsystem();
    private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final PneumaticSubsystem m_pneumatic = new PneumaticSubsystem();
    // Yeni eklendi

    // PID Kontrolcü (Hizalama için)
    private final PIDController m_turnController = new PIDController(
            VisionConstants.kTurnP,
            VisionConstants.kTurnI,
            VisionConstants.kTurnD);
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final edu.wpi.first.wpilibj.smartdashboard.SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        // Auto Chooser yapılandırması - Hata korumalı
        if (com.pathplanner.lib.auto.AutoBuilder.isConfigured()) {
            autoChooser = com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser();
        } else {
            // Eğer PathPlanner ayarlanmadıysa boş bir menü oluştur, böylece robot çökmez
            autoChooser = new edu.wpi.first.wpilibj.smartdashboard.SendableChooser<>();
            autoChooser.setDefaultOption("PathPlanner Ayarlanmadı!",
                    new edu.wpi.first.wpilibj2.command.InstantCommand());
        }
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Varsayılan sürüş komutu (Arcade Drive) - TEK JOYSTICK MODU
        // Sol Stick Y: İleri/Geri Hız
        // Sol Stick X: Sağa/Sola Dönüş
        m_robotDrive.setDefaultCommand(
                new JoystickDriveCommand(
                        m_robotDrive,
                        () -> -m_driverController.getRawAxis(1),
                        () -> m_driverController.getRawAxis(3)));

        // Shooter Kontrolü: 6. Buton (Xbox Right Bumper / RB)
        // Basılı tutulduğunda çalışır, bırakıldığında durur.
        m_driverController.button(1).whileTrue(
                Commands.parallel(

                        m_intakeSubsystem.runIntakeCommand(-0.8),
                        m_shooterSubsystem.runShooterCommand(-0.8),
                        Commands.sequence(
                                Commands.waitSeconds(0.5),
                                m_conveyorSubsystem
                                        .runConveyorCommand(frc.robot.Constants.ConveyorConstants.kConveyorSpeed))));

        // BUTON 20: Shooter Hemen Baslar, 3 Saniye Sonra Conveyor Baslar
        // Ikisi de tus birakilinca durur. ATIS ICERIDEN ALMA
        m_driverController.button(6).whileTrue(
                Commands.parallel(
                        m_shooterSubsystem.runShooterCommand(1.0),
                        m_intakeSubsystem.runIntakeCommand(-1.0),
                        Commands.sequence(
                                Commands.waitSeconds(1.5),
                                m_conveyorSubsystem.runConveyorCommand(-0.5))));

        // buton2
        m_driverController.button(2).whileTrue(
                Commands.parallel(
                        m_shooterSubsystem.runShooterCommand(-1.0),
                        m_intakeSubsystem.runIntakeCommand(-1.0),
                        Commands.sequence(
                                Commands.waitSeconds(0.5),
                                m_conveyorSubsystem.runConveyorCommand(0.5))));

        // BUTON 21: Tam Ters Yön (Shooter Geri + 3sn Bekle + Conveyor Geri)
        m_driverController.button(5).whileTrue(
                Commands.parallel(
                        m_intakeSubsystem.runIntakeCommand(0.2),
                        Commands.sequence(
                                Commands.waitSeconds(0.0),
                                m_conveyorSubsystem.runConveyorCommand(0.8))));

        // VISION HİZALAMA: Buton 7 (Back Tuşu)
        // Alliance rengine göre doğru AprilTag'e hizalanır.
        // Mavi İttifak: ID 25, 26 (Mavi Pota Altı)
        // Kırmızı İttifak: ID 9, 10 (Kırmızı Pota Altı)
        m_driverController.button(6).whileTrue(
                m_robotDrive.run(() -> {
                    // İttifak rengini al
                    var alliance = DriverStation.getAlliance();
                    int[] targetTags;

                    // Renge göre hedef tag ID'lerini belirle
                    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                        targetTags = new int[] { 1, 8 };
                    } else {
                        // Varsayılan Mavi veya renk alınamazsa Mavi (Pota Altı Tagler)
                        targetTags = new int[] { 3, 4 };
                    }

                    // Vision subsystem'den bu ID'lerden birini bulmasını iste
                    var target = m_visionSubsystem.getTargetWithId(targetTags);
                    double turnSpeed = 0.0;

                    if (target != null) {
                        // Hedef bulunduysa PID hesapla
                        double yawError = target.getYaw();
                        turnSpeed = m_turnController.calculate(yawError, 0.0);
                        turnSpeed = edu.wpi.first.math.MathUtil.clamp(-turnSpeed, -0.2, 0.2);
                    } else {
                        // Hedef yoksa dur
                        turnSpeed = 0.0;
                    }

                    m_robotDrive.arcadeDrive(0.0, turnSpeed);
                }));

        // VISION POZİSYON GİTME: Buton 4 (Y Tuşu)
        // Robotun o anki (0,0) kabul edilen başlangıç noktasından 2 metre ileriye
        // gitmesini sağlar.
        /*
         * m_driverController.button(4).whileTrue(
         * new DriveToPoseCommand(
         * m_robotDrive,
         * new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0))
         * )
         * );
         */

        // --- TEST BUTONLARI ---

    // --- PNEUMATICS (örnek) ---
    // Buton 7: Toggle both solenoids birlikte
    m_driverController.button(7).onTrue(m_pneumatic.toggleBothCommand());

    // Buton 8: Solenoid B toggle
    m_driverController.button(8).onTrue(m_pneumatic.toggleBCommand());

    // Buton 9: İki solenoidi aç (forward)
    m_driverController.button(9).onTrue(m_pneumatic.extendBoth());

    // Buton 10: İki solenoidi kapat (reverse)
    m_driverController.button(10).onTrue(m_pneumatic.retractBoth());

    // --- TEST BUTONLARI ---
    // BUTON 2: Sadece Leader Motorlar (Follower'lar da dönecek çünkü follow
    // yapılandı)

    }

    /**
     * Vision verisi ile PoseEstimator'ı günceller.
     * Robot.java'nın robotPeriodic metodundan çağrılmalıdır.
     */
    public void updatePoseEstimator() {
        Optional<EstimatedRobotPose> visionEstimate = m_visionSubsystem.getEstimatedGlobalPose();
        if (visionEstimate.isPresent()) {
            EstimatedRobotPose estPose = visionEstimate.get();
            m_robotDrive.addVisionMeasurement(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Vision Valid", true);
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putString("Vision Pose",
                    estPose.estimatedPose.toPose2d().toString());
        } else {
            edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Vision Valid", false);
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}