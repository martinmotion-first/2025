// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//REF: https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
//ORIG from limelight documentation about event prep
// import edu.wpi.first.wpiutil.net.PortForwarder;
//END ORIG
//MODDED from limelight documentation about event prep to 2025 library that exists
import edu.wpi.first.net.PortForwarder;
//END MODDED from limelight documentation about event prep to 2025 library that exists


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static CTREConfigs ctreConfigs;

  private final RobotContainer m_robotContainer;
  private Field2d m_field = new Field2d();
  //MODDED
  // NetworkTable m_networkTable = NetworkTableInstance.getDefault().getTable("limelight");
  //END MODDED

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // //**** START LimeLight Dashboard *****
    // //MODDED
    // NetworkTableEntry tx = m_networkTable.getEntry("tx");
    // NetworkTableEntry ty = m_networkTable.getEntry("ty");
    // NetworkTableEntry ta = m_networkTable.getEntry("ta");
    // //END MODDED

    // //ORIG
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // //END ORIG

    // //ORIG
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");
    // //END ORIG

    // //read values periodically
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    // //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

    // //***** END LimeLight Dashboard ****
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    SmartDashboard.putData("Field", m_field);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  @Override
  public void robotInit(){
    double xOffset = 3;
    double yOffset = 3;

    double maxVelocity = Units.feetToMeters(3.0);
    double maxAcceleration = Units.feetToMeters(3.0);
    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    Trajectory m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0 + xOffset, 0 + yOffset, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1 + xOffset, 1 + yOffset), new Translation2d(2 + xOffset, -1 + yOffset)),
            new Pose2d(3 + xOffset, 0 + yOffset, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(maxVelocity, maxAcceleration));

    // Trajectory m_trajectoryOrig =
    //   TrajectoryGenerator.generateTrajectory(
    //       new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    //       List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //       new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
    //       new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.

    m_field.getObject("traj").setTrajectory(m_trajectory);
    // m_field.getObject("trajOrig").setTrajectory(m_trajectoryOrig);

      //ADDED from limelight event prep: https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
      // Make sure you only configure port forwarding once in your robot code.
      // Do not place these function calls in any periodic functions
      for (int port = 5800; port <= 5809; port++) {
        PortForwarder.add(port, "limelight.local", port);
      }
      //END ADDED from limelight event prep: https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices

  }

}
