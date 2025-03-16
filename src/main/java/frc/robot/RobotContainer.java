// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.RobotCommands;
import frc.robot.controllers.DriverMapping6237MR;
import frc.robot.controllers.OperatorMapping6237MR;
import frc.robot.generated.TunerConstants;
import frc.robot.limelightlib.LimelightHelpers;
import frc.robot.rusthounds_util.CoralSim;
import frc.robot.rusthounds_util.PositionTracker;
import frc.robot.rusthounds_util.ScoreLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CommandXboxController driver = new CommandXboxController(Constants.kXboxDriverPort);
  private final CommandXboxController operatorAlterante = new CommandXboxController(Constants.kXboxOperatorPort);


  private Mechanism2d mechanisms = new Mechanism2d(5, 3);
  private MechanismRoot2d root = mechanisms.getRoot("root", 2.5, 0.25);
  private MechanismLigament2d elevatorLigament = root
            .append(new MechanismLigament2d("elevatorStage", Units.inchesToMeters(10), 90,
                    4,
                    new Color8Bit(Color.kOrange)));
  private MechanismLigament2d armLigament = elevatorLigament
            .append(new MechanismLigament2d("armLigament", Units.inchesToMeters(10), 270,
                    5,
                    new Color8Bit(Color.kRed)));

  PositionTracker positionTracker = new PositionTracker();
  Elevator elevator = new Elevator(positionTracker, elevatorLigament);
  Arm arm = new Arm(positionTracker, armLigament, elevator::getCarriageComponentPose);
  Intake intake = new Intake();
  Climber climber = new Climber();
  CoralSim coralSim = new CoralSim(() -> drivetrain.getState().Pose, arm::getClawComponentPose);
  // IntakeArm intakeArm = new IntakeArm();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser("1dot5m straight with 90degree rotation");
    autoChooser = AutoBuilder.buildAutoChooser("");
    // SmartDashboard.putData("Auto Mode", autoChooser);
    // SmartDashboard.putData("pigeon2", drivetrain.getPigeon2());
    // SmartDashboard.putNumber("drivetrain module 0 (front-left) encoder", drivetrain.getModule(0).getEncoder().getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("drivetrain module 1 (front-right) encoder", drivetrain.getModule(1).getEncoder().getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("drivetrain module 2 (back-left) encoder", drivetrain.getModule(2).getEncoder().getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putNumber("drivetrain module 3 (back-right) encoder", drivetrain.getModule(3).getEncoder().getAbsolutePosition().getValueAsDouble());
    // SmartDashboard.putData("Steer motor module 0 (front-left)", drivetrain.getModule(0).getSteerMotor());
    // SmartDashboard.putData("Steer motor module 1 (front-right)", drivetrain.getModule(1).getSteerMotor());
    // SmartDashboard.putData("Steer motor module 2 (back-left)", drivetrain.getModule(2).getSteerMotor());
    // SmartDashboard.putData("Steer motor module 3 (back-right)", drivetrain.getModule(3).getSteerMotor());
    // SmartDashboard.putData("Drive motor module 0 (front-left)", drivetrain.getModule(0).getDriveMotor());
    // SmartDashboard.putData("Drive motor module 1 (front-right)", drivetrain.getModule(1).getDriveMotor());
    // SmartDashboard.putData("Drive motor module 2 (back-left)", drivetrain.getModule(2).getDriveMotor());
    // SmartDashboard.putData("Drive motor module 3 (back-right)", drivetrain.getModule(3).getDriveMotor());
    // SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
    // SmartDashboard.putNumber("Elevator velocity", elevator.getVelocity());
    // // SmartDashboard.putData("Elevator command", elevator.getCurrentCommand());
    // SmartDashboard.putBoolean("Elevator at goal", elevator.atGoal());
    // SmartDashboard.putBoolean("Elevator initialized", elevator.getInitialized());
    // SmartDashboard.putNumber("Elevator position tracker position", positionTracker.getElevatorPosition());
    // SmartDashboard.putNumber("Intake Arm Position", intakeArm.getPosition());
    // SmartDashboard.putNumber("Intake Arm Velocity", intakeArm.getVelocity());
    // SmartDashboard.putBoolean("Intake Arm at goal", intakeArm.atGoal());
    // SmartDashboard.putBoolean("Intake arm initialized", intakeArm.getInitialized());

  
    // Configure the trigger bindings

    // NamedCommands.registerCommand("prepareCoralScoreCommand", RobotCommands.prepareCoralScoreCommand(null, elevator, arm, coralSim));
    NamedCommands.registerCommand("autoPrepareCoralScoreCommandL2", RobotCommands.autoPrepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
    NamedCommands.registerCommand("autoPrepareCoralScoreCommandL3", RobotCommands.autoPrepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
    NamedCommands.registerCommand("autoPrepareCoralScoreCommandL4", RobotCommands.autoPrepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
    NamedCommands.registerCommand("scoreCoralCommand", RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim));
    NamedCommands.registerCommand("prepareIntakeCoralCommand", RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim));
    NamedCommands.registerCommand("intakeCoralCommand", RobotCommands.intakeCoralCommand(elevator, arm, coralSim));
    NamedCommands.registerCommand("intakeIntoScoreCommandL2", RobotCommands.intakeIntoScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
    NamedCommands.registerCommand("intakeIntoScoreCommandL3", RobotCommands.intakeIntoScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
    NamedCommands.registerCommand("intakeIntoScoreCommandL4", RobotCommands.intakeIntoScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
    NamedCommands.registerCommand("prepareAlgaeL2RemoveCommand", RobotCommands.prepareAlgaeL2RemoveCommand(elevator, arm));
    NamedCommands.registerCommand("prepareAlgaeL3RemoveCommand", RobotCommands.prepareAlgaeL3RemoveCommand(elevator, arm));
    NamedCommands.registerCommand("algaeRemoveCommand", RobotCommands.algaeRemoveCommand(elevator, arm));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    DriverMapping6237MR.mapXboxController(driver, drivetrain, NetworkTableInstance.getDefault().getTable("limelight"));
    OperatorMapping6237MR.mapXboxController(operatorAlterante, drivetrain, elevator, arm, intake, climber, coralSim);
  }

  public double getElevatorPosition(){
    return elevator.getPosition();
  }

  public double getArmPosition(){
    return arm.getPosition();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
  public void getSimPeriodic(Field2d field) {
    field.setRobotPose(drivetrain.getState().Pose);
  }

  public void getAutoPeriodic(Timer timer) {
    // drivetrain.addVisionMeasurement(LimelightHelpers.getBotPose2d(Constants.kLimelightName), Timer.getMatchTime());
  }
}
