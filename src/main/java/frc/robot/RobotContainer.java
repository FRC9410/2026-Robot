package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  // Subsystems
  private final Swerve drive;
  //   private final Vision vision;
  private final RobotContainer robotContainer = this;
  private final StateMachine stateMachine;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController characterizationController = new CommandXboxController(1);

  private final Telemetry logger = new Telemetry(MAX_SPEED);

  // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = TunerConstants.createDrivetrain();

    drive.resetPose(new Pose2d());

    stateMachine =
        new StateMachine(

            drive);

   

    // Configure the button bindings
    configureButtonBindings();
    configureCharacterizationButtonBindings();
    drive.registerTelemetry(logger::telemeterize);
  }

  
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(new SwerveDriveCommand(drive, controller, stateMachine, false));

    controller.a().whileTrue(new SwerveDriveCommand(drive, controller, stateMachine, true));
    controller.a().onFalse(new SwerveDriveCommand(drive, controller, stateMachine, false));
   

  }

  private void configureCharacterizationButtonBindings() {
    // Example POV button binding
    characterizationController
        .a()
        .whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    characterizationController
        .x()
        .whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    characterizationController.b().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    characterizationController.y().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  // new POVButton(driverController, 0)
  // .whenPressed();

  

  public Swerve getDrive() {
    return drive;
  }

  public RobotContainer getRobotContainer() {
    return robotContainer;
  }

  public Dashboard getDashboard() {
    return dashboard;
  }
}
