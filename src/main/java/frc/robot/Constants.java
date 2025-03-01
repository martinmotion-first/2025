// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.generated.TunerConstants;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.math.util.Units;

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
  public static final int kXboxOperatorManualOnlyPort = 2;

  public static final String kLimelightName = "";
  
  public static final class Swerve {
      public static final int pigeonID = 8;
      public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-
    //new CTRE P6 constants
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  }

  public static final class Elevator {
        public static enum ElevatorPosition {
            // BOTTOM(0.0698),
            // INTAKE_PREP(0.55),
            // INTAKE(0.355),
            // ALGAE_L2(0.884),
            // ALGAE_L3(1.234),
            
            // L1(0.323),
            // L2(0.31),
            // L3(0.70),
            // L4(1.27),
            // TOP(1.57);
            BOTTOM(-0.007),
            INTAKE_PREP(-.154),
            INTAKE(-.063),
            ALGAE_L2(-.308),
            ALGAE_L3(-.4732),

            L1(-.047),
            L2(-.042),
            L3(-.224),
            L4(-.49),
            TOP(-.63);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        public static final double MIN_HEIGHT_TO_ALLOW_ARM_EXTENSION = -0.3;

        public static final double SCORING_MOVEMENT = -0.25;

        public static final int MOTOR_ID = 1;
        public static final boolean MOTOR_INVERTED = false;

        public static final int MOTOR_ID2 = 3;
        public static final boolean MOTOR_INVERTED2 = true;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 15.0; // changing from 5.0;
        public static final double MASS_KG = Units.lbsToKilograms(20);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.32) / 2.0; // TODO
        public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE / GEARING;

        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.57; // TODO

        public static final int CURRENT_LIMIT = 60;

        public static final double kP = 50; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 5; // TODO
        public static final double kS = 0.095388; // TODO
        public static final double kG = 0.54402; // TODO
        public static final double kV = 7.43; // TODO
        public static final double kA = 1.0; // TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1; // TODO HERE!!! (tuning way down for intial attempt (from 1.3))
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1; // TODO HERE!!! (tuning way down for intial attempt (from 3))
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Arm {
        public static enum ArmPosition {
            // BOTTOM(-Math.PI / 2.0 + Units.degreesToRadians(5)),
            // HORIZONTAL(0),
            // L1(0),
            // L2(Units.degreesToRadians(55)), // reef angle
            // L3(Units.degreesToRadians(55)),
            // L4(1.033),
            // TOP(Math.PI / 2.0);

            // BOTTOM(1.57),
            // HORIZONTAL(2.47),
            // L1(2.47),
            // L2(3.073), 
            // L3(3.073),
            // L4(3.15),
            // TOP(3.379);

            BOTTOM(0),
            HORIZONTAL(1.3),
            L1(1.3),
            L2(Units.degreesToRadians(115)), 
            L3(Units.degreesToRadians(115)),
            L4(Units.degreesToRadians(160)),
            TOP(2.5);

            public final double value;

            private ArmPosition(double value) {
                this.value = value;
            }
        }

        public static final double MAX_ARM_EXTENSION_TO_ALLOW_ELEVATOR_DESCENT = 1.8;

        public static final double SCORING_MOVEMENT = -0.8;

        public static final int MOTOR_ID = 2;
        public static final boolean MOTOR_INVERTED = true;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 45.0; //changing from 40.0;
        public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = Math.PI;

        public static final int CURRENT_LIMIT = 50;

        public static final double kP = 10; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kS = 0.017964; // TODO
        public static final double kG = 0.321192; // TODO
        public static final double kV = 0.876084;// TODO
        public static final double kA = 0.206676;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = .1; // TODO HERE!!! (tuning way down for intial attempt (from 8))
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = .1; // TODO HERE!!! (tuning way down for intial attempt (from 4))
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Intake {
        public static final int MOTOR_ID = 6;
        public static final boolean MOTOR_INVERTED = true;
        public static final int CURRENT_LIMIT = 40; //dropping down to 60;
    }

    public static final class Climber {
        public static final int MOTOR_ID = 4;
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 60;

        public static final double MIN_POSITION_METERS = 0.0;
        public static final double MAX_POSITION_METERS = 0.1; // TODO HERE!!! (tuning way down for intial attempt (from 1.0))

        public static final double GEARING = 80.0; //changing from 64.0
        public static final double MASS_KG = Units.lbsToKilograms(80); // robot weight
        public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(0.5);
        public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = SPOOL_CIRCUMFERENCE * GEARING;

    }

    public static final class IntakeArm {
        public static enum IntakeArmPosition {
            BOTTOM(4.3),
            INTERMEDIATE(.2),
            TOP(0);

            public final double value;

            private IntakeArmPosition(double value) {
                this.value = value;
            }
        }

        public static final int MOTOR_ID = 5;
        public static final boolean MOTOR_INVERTED = true;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 20.0; //changing from 40.0;
        public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

        public static final int CURRENT_LIMIT = 50;

        public static final double kP = 10; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kS = 0.017964; // TODO
        public static final double kG = 0.321192; // TODO
        public static final double kV = 0.876084;// TODO
        public static final double kA = 0.206676;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 3; // TODO HERE!!! (tuning way down for intial attempt (from 8))
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2; // TODO HERE!!! (tuning way down for intial attempt (from 4))
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

}