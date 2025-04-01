package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.rusthounds_util.CoralSim;
import frc.robot.rusthounds_util.ScoreLevel;
import frc.robot.rusthounds_util.GlobalStates;
import frc.robot.Constants;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class OperatorMapping6237MR {

    public static void mapXboxController(CommandXboxController controller, CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Intake intake,
            Climber climber, CoralSim coralSim) {

        // controller.back().whileTrue(RobotCommands.kill(elevator, arm, intake, climber));

        // controller.leftBumper().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.ALGAE_L2));       // Left Stick Left (Negative Y-Axis)
        // controller.rightBumper().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.ALGAE_L3));   // Left Stick Right (Positive Y-Axis)
        controller.axisLessThan(1, -0.2).whileTrue(intake.reverseRollersCommand());//.onFalse(intake.stopRollersCommand())       //Left Stick Up
        controller.axisGreaterThan(1, 0.2).onTrue(RobotCommands.intakeCoralCommand(elevator, arm, coralSim));       //Left Stick Up

        controller.start().onTrue(RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim));
        // controller.y().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.TOP)); 
        // controller.x().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.L3));  
        // controller.b().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.L2));
        // controller.a().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.ARM_FREE));

        controller.povDown().whileTrue(RobotCommands.armOnlyGiveNegativeVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));
        controller.povUp().whileTrue(RobotCommands.armOnlyGivePositiveVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));
        
        controller.axisLessThan(5, -0.2).whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        controller.axisGreaterThan(5, 0.2).whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        

        climber.setDefaultCommand(Commands
        .run(() -> climber.setVoltage(MathUtil
                        .applyDeadband((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 8,
                                0.1)),
                        climber));
    }

    // NOTES
    /* 
     *         
     *  // Left Stick
     *  // Up (Negative Y-Axis)
        driverController.axisLessThan(1, -0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Up")));

        // Down (Positive Y-Axis)
        driverController.axisGreaterThan(1, 0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Down")));

        // Left (Negative X-Axis)
        driverController.axisLessThan(0, -0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Left")));

        // Right (Positive X-Axis)
        driverController.axisGreaterThan(0, 0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Right")));
     *
     * 
     * 
     * 
     * // RIght Stick
     *         // Right Stick Up (Negative Y-Axis)
        driverController.axisLessThan(5, -0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Right Stick Up")));

        // Right Stick Down (Positive Y-Axis)
        driverController.axisGreaterThan(5, 0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Right Stick Down")));

        // Right Stick Left (Negative X-Axis)
        driverController.axisLessThan(4, -0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Right Stick Left")));

        // Right Stick Right (Positive X-Axis)
        driverController.axisGreaterThan(4, 0.2)
            .onTrue(new InstantCommand(() -> activateCommand("Right Stick Right"))); 
    */
}
