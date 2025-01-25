// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.*;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

 public final class Constants {
  public static final double stickDeadband = 0.2;
  public static final double kTriggerButtonThreshold = 0.3;

  public static final int kXboxDriverPort = 0;
  public static final int kXboxOperatorPort = 1;
  
  public static final class Swerve {
      public static final int pigeonID = 8;
      public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

      //HERE!!!! - Removing old drivetrain constants. Will need to bring over the new CTRE P6 info
      /* Drivetrain Constants */
    //   public static final double trackWidth = Units.inchesToMeters(19.75); //TODO: This must be tuned to specific robot
    //   public static final double wheelBase = Units.inchesToMeters(19.75); //TODO: This must be tuned to specific robot
    //   public static final double wheelCircumference = chosenModule.wheelCircumference;

      /* Swerve Kinematics  
       * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    //    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
    //       new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
    //       new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
    //       new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
    //       new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    //   /* Module Gear Ratios */
    //   public static final double driveGearRatio = chosenModule.driveGearRatio;
    //   public static final double angleGearRatio = chosenModule.angleGearRatio;

    //   /* Motor Inverts */
    //   public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive; //=  chosenModule.angleMotorInvert; // NOTE! The other option here is InvertedValue.Clockwise_Positive
    //   public static final InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive; //= chosenModule.driveMotorInvert; // NOTE! The other option here is InvertedValue.Clockwise_Positive

    //   /* Angle Encoder Invert */
    //   public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    //   /* Swerve Current Limiting */
    //   // public static final int angleContinuousCurrentLimit = 2;//25;
    //   // public static final int anglePeakCurrentLimit = 5;//40;
    //   public static final int angleContinuousCurrentLimit = 25;//25;
    //   public static final int anglePeakCurrentLimit = 40;//40;
    //   public static final double anglePeakCurrentDuration = 0.1;
    //   public static final boolean angleEnableCurrentLimit = true;

    //   // public static final int driveContinuousCurrentLimit = 2;//35;
    //   // public static final int drivePeakCurrentLimit = 5; //60;
    //   public static final int driveContinuousCurrentLimit = 35;//35;
    //   public static final int drivePeakCurrentLimit = 60; //60;
    //   public static final double drivePeakCurrentDuration = 0.1;
    //   public static final boolean driveEnableCurrentLimit = true;

    //   /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
    //    * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    //   // public static final double openLoopRamp = 1.0;//.25;
    //   public static final double openLoopRamp = .25;
    //   public static final double closedLoopRamp = 0.0;

    //   /* Angle Motor PID Values */
    //   public static final double angleKP = chosenModule.angleKP;
    //   public static final double angleKI = chosenModule.angleKI;
    //   public static final double angleKD = chosenModule.angleKD;
    //   public static final double angleKF = chosenModule.angleKF;

    //   /* Drive Motor PID Values */
    //   public static final double driveKP = 0.01; //TODO: This must be tuned to specific robot
    //   public static final double driveKI = 0.0;
    //   public static final double driveKD = 0.0;
    //   public static final double driveKF = 0.0;

    //   /* Drive Motor Characterization Values 
    //    * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    //   public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
    //   public static final double driveKV = (1.51 / 12);
    //   public static final double driveKA = (0.27 / 12);

    //   /* Swerve Profiling Values */
    //   /** Meters per Second */
    //   // public static final double maxSpeed = .1; //4.5; //TODO: This must be tuned to specific robot
    //   public static final double maxSpeed = .5;
    //   // public static final double maxSpeed = 2.5; //tuning this down to try to limit the overall drive power temporarily


    //   /** Radians per Second */
    //   // public static final double maxAngularVelocity = .1; //10; //TODO: This must be tuned to specific robot
    //   public static final double maxAngularVelocity = .5; //10; //TODO: This must be tuned to specific robot

    //   /* Neutral Modes */
    //   //   public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    //   //   public static final NeutralMode driveNeutralMode = NeutralMode.Brake;
    //   public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    //   public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    //   /* Module Specific Constants */
    //   /* Front Left Module - Module 0 */
    //   public static final class Mod0 { //TODO: This must be tuned to specific robot
    //       public static final int driveMotorID = 7;//3;
    //       public static final int angleMotorID = 6;//5;
    //       public static final int canCoderID = 12;//10;
    //       public static final Rotation2d angleOffset = Rotation2d.fromDegrees(170.15);
    //       public static final SwerveModuleConstants constants = 
    //           new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    //   }

    //   /* Front Right Module - Module 1 */
    //   public static final class Mod1 { //TODO: This must be tuned to specific robot
    //       public static final int driveMotorID = 1;//7;
    //       public static final int angleMotorID = 2;//6;
    //       public static final int canCoderID = 9;//12; //9 probably....
    //       public static final Rotation2d angleOffset = Rotation2d.fromDegrees(346.992);
    //       public static final SwerveModuleConstants constants = 
    //           new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    //   }
      
    //   /* Back Left Module - Module 2 */
    //   public static final class Mod2 { //TODO: This must be tuned to specific robot
    //       public static final int driveMotorID = 0;//1;
    //       public static final int angleMotorID = 4;//2;
    //       public static final int canCoderID = 11;//9;
    //       public static final Rotation2d angleOffset = Rotation2d.fromDegrees(317.988);
    //       public static final SwerveModuleConstants constants = 
    //           new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    //   }

    //   /* Back Right Module - Module 3 */
    //   public static final class Mod3 { //TODO: This must be tuned to specific robot
    //       public static final int driveMotorID = 3;//0;
    //       public static final int angleMotorID = 5;//4;
    //       public static final int canCoderID = 10;//11;
    //       public static final Rotation2d angleOffset = Rotation2d.fromDegrees(221.484);
    //       public static final SwerveModuleConstants constants = 
    //           new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    //   }
    //HERE!!!! - END Removing old drivetrain constants. Will need to bring over the new CTRE P6 info
    
    //new CTRE P6 constants
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  }

  public static final class AutonomousModeConstants { 
      //Autonomous-only drivetrain constants

      //NOTE! - TURNING DOWN THE AUTONOMOUS SPEEDS SIGNIFICANTLY FOR NOW 
      // public static final double kMaxSpeedMetersPerSecond = 3;
      // public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxSpeedMetersPerSecond = .6;
      public static final double kMaxAccelerationMetersPerSecondSquared = 1;

      public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
      public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  
      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;
  
      /* Constraint for the motion profilied robot angle controller */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
              kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

      //Autonomous-only Arm subsytem constants
      public static final double kAutonomousArmWaitTime = 1;
      //Autonomous-only Intake Subsystem constants
      // public static final double kAutonomonousIntakeRunTime = 1;
      public static final double kAutonomonousIntakeRunTime = 1; //running intake longer since speeds are lower...
      public static final double kLauncherRunBeforeFiringDelay = .25;
  }

  public static final class SimulatorConstants6237MR{
      public static final double kBlueRightStartingPositionX = 1.971;
      public static final double kBlueRightStartingPositionY = 2.107;

      public static final double kRedLeftStartingPositionX = 15.741;
      public static final double kRedLeftStartingPositionY = 2.107;

      public static final double kBlueCenterStartingPositionX = 1.8;
      public static final double kBlueCenterStartingPositionY = 3.5;

      public static final double kRedCenterStartingPositionX = 16.123;
      public static final double kRedCenterStartingPositionY = 3.5;

      public static final double kBlueLeftStartingPositionX = 1.716;
      public static final double kBlueLeftStartingPositionY = 7.032;

      public static final double kRedRightStartingPositionX = 15.955;
      public static final double kRedRightStartingPositionY = 7.032;
  }

  public static final class Arm {
      public static final int kArmCanId = 33;//2;
      public static final boolean kArmInverted = true;
      public static final int kCurrentLimit = 40;

      public static final double kSoftLimitReverse = -1.1498;
      public static final double kSoftLimitForward = 0.0;

      public static final double kArmGearRatio = (1.0 / 75.0) * (28.0 / 50.0) * (16.0 / 64.0);
      public static final double kPositionFactor =
          kArmGearRatio
              * 2.0
              * Math.PI; // multiply SM value by this number and get arm position in radians
      public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
      public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
      public static final double kArmZeroCosineOffset =
          1.342; // radians to add to converted arm position to get real-world arm position (starts at
      // ~76.9deg angle)
      public static final ArmFeedforward kArmFeedforward =
          new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);
      public static final PIDGainsArm kArmPositionGains = new PIDGainsArm(2.5, 0.0, 0.0);
      public static final TrapezoidProfile.Constraints kArmMotionConstraint =
          new TrapezoidProfile.Constraints(1.0, 2.0);

      public static final double kHomePosition = 0.3;
      public static final double kScoringPosition = 0.34;
      public static final double kIntakePosition = -1.23;
}

public static final class Intake {
  public static final int kCanId = 30;//1;
  public static final boolean kMotorInverted = true;
  public static final int kCurrentLimit = 80;

  public static final PIDGainsArm kPositionGains = new PIDGainsArm(1.0, 0.0, 0.0);
  public static final double kPositionTolerance = 0.5;

  public static final double kIntakePower = 0.7;

  public static final double kRetractDistance = -3.5;

  public static final double kShotFeedTime = 1.0;
  // public static final double kShotFeedTime = 10.0; //testing with this temporarily, but resetting for operator
}

public static final class Launcher {
  public static final int kTopCanId = 31;//3;
  public static final int kBottomCanId = 32;//4;

  public static final int kCurrentLimit = 80;

  public static final double kTopPower = -0.7;
  public static final double kBottomPower = -0.8;

  public static final double kTopLowPower = -0.2;
  public static final double kBottomLowPower = -0.2;
}

}