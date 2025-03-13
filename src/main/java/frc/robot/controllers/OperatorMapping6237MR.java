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
import frc.robot.Constants.IntakeArm.IntakeArmPosition;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

public class OperatorMapping6237MR {
    public static void mapXboxController(CommandXboxController controller, CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Intake intake,
    Climber climber, CoralSim coralSim) {
        
        controller.back().whileTrue(RobotCommands.kill(elevator, arm, intake, climber));
        // controller.a().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, elevator, arm, coralSim));
        // controller.x().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
        // controller.b().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
        // controller.y().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
        controller.y().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.TOP));
        controller.x().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.L3));
        controller.b().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.L2));
        controller.a().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.ARM_FREE));

        controller.rightBumper().whileTrue(RobotCommands.armOnlyGivePositiveVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));
        controller.leftBumper().whileTrue(RobotCommands.armOnlyGiveNegativeVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));
        // controller.axisLessThan(5, -0.2).whileTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));       // Right Stick Up (Negative Y-Axis)
        // controller.axisGreaterThan(5, 0.2).whileTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));        // Right Stick Down (Positive Y-Axis)
        
        // controller.start().whileTrue(RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim));
        // controller.povUp().whileTrue(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim));
        // controller.povDown().whileTrue(RobotCommands.intakeCoralCommand(elevator, arm, coralSim));
        // controller.povLeft().whileTrue(RobotCommands.prepareAlgaeL2RemoveCommand(elevator, arm));
        // controller.povRight().whileTrue(RobotCommands.prepareAlgaeL3RemoveCommand(elevator, arm));
        // controller.leftStick().whileTrue(RobotCommands.algaeRemoveCommand(elevator, arm));

        
        climber.setDefaultCommand(Commands
        .run(() -> climber.setVoltage(MathUtil
                        .applyDeadband((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 4,
                                0.1)),
                        climber));
        
    }

    public static void mapXboxControllerManualOnlyControl(CommandXboxController controller, CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Intake intake,
            Climber climber, CoralSim coralSim) {

        controller.back().whileTrue(RobotCommands.kill(elevator, arm, intake, climber));

        // controller.x().whileTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));
        // controller.y().whileTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));

        // controller.povLeft().whileTrue(RobotCommands.armOnlyGivePositiveVoltage(arm));
        // controller.povRight().whileTrue(RobotCommands.armOnlyGiveNegativeVoltage(arm));

        // controller.povRight().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.povLeft().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.a().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.b().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));

        // controller.povLeft().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.povRight().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.x().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.y().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.a().whileTrue(RobotCommands.armOnlyGivePositiveVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));
        // controller.b().whileTrue(RobotCommands.armOnlyGiveNegativeVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));

        //CONFIRMED - intake arm positions
        // controller.x().onTrue(RobotCommands.intakeArmMoveToPosition(intakeArm, IntakeArmPosition.TOP));
        // controller.y().onTrue(RobotCommands.intakeArmMoveToPosition(intakeArm, IntakeArmPosition.INTERMEDIATE));
        // controller.b().onTrue(RobotCommands.intakeArmMoveToPosition(intakeArm, IntakeArmPosition.BOTTOM));
        // controller.leftBumper().whileTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));
        // controller.rightBumper().whileTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));
        //END CONFIRMED

        //NOT CONFIRMED - elevator positions
        // controller.x().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L1));
        // controller.y().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L2));
        // controller.b().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L3));
        // controller.a().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L4));
        // controller.leftBumper().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.rightBumper().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        //NOT CONFIRMED

        //(MOSTLY) CONFIRMED, leaving off here - arm positions
        // controller.x().onTrue(RobotCommands.armOnlyMoveToPosition(arm, ArmPosition.BOTTOM));
        // controller.y().onTrue(RobotCommands.armOnlyMoveToPosition(arm, ArmPosition.HORIZONTAL));
        // controller.b().onTrue(RobotCommands.armOnlyMoveToPosition(arm, ArmPosition.L2));
        // controller.a().onTrue(RobotCommands.armOnlyMoveToPosition(arm, ArmPosition.L3));
        // controller.leftTrigger().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));
        // controller.rightTrigger().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator)).onFalse(RobotCommands.elevatorOnlyGiveZero(elevator));


        controller.axisLessThan(0, -0.2).onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.ALGAE_L2));       // Left Stick Left (Negative Y-Axis)
        controller.axisGreaterThan(0, 0.2).onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.ALGAE_L3));   // Left Stick Right (Positive Y-Axis)
        controller.axisLessThan(1, -0.2).onTrue(RobotCommands.armOnlyMoveToPosition(arm, ArmPosition.L2));       //Left Stick Up
        controller.axisGreaterThan(1, 0.2).onTrue(RobotCommands.intakeCoralCommand(elevator, arm, coralSim));       //Left Stick Up
        // controller.x().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L1));
        // controller.y().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L2));
        // controller.b().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L3));
        // controller.a().onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L4));
        // controller.axisLessThan(1, Constants.operatorStickDeadband).onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L1));  //1 is leftStick Y axis, up is negactive
        // controller.axisGreaterThan(1,  Constants.operatorStickDeadband).onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.L2)); 
        // controller.axisLessThan(0,  Constants.operatorStickDeadband).onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.ARM_FREE)); //0 is leftStick X axis, left is negative
        // controller.axisGreaterThan(0,  Constants.operatorStickDeadband).onTrue(RobotCommands.elevatorOnlyMoveToPosition(elevator, ElevatorPosition.TOP));


        controller.start().onTrue(RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim));
        controller.y().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.TOP)); //TEMP TEMP
        controller.x().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.L3));  //TEMP TEMP
        controller.b().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.L2));
        controller.a().onTrue(RobotCommands.elevatorCombinedCommand(elevator, arm, ElevatorPosition.ARM_FREE));

        // controller.rightBumper().whileTrue(RobotCommands.armOnlyGivePositiveVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));
        // controller.leftBumper().whileTrue(RobotCommands.armOnlyGiveNegativeVoltage(arm)).onFalse(RobotCommands.armOnlyGiveZeroVoltage(arm));
        // controller.rightBumper().whileTrue(intake.runRollersCommand());
        // controller.leftBumper().whileTrue(intake.reverseRollersCommand());



        // controller.axisLessThan(5, -0.2).whileTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));       // Right Stick Up (Negative Y-Axis)
        // controller.axisGreaterThan(5, 0.2).whileTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm)).onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));        // Right Stick Down (Positive Y-Axis)
        // controller.axisLessThan(5, -0.2).onTrue(RobotCommands.intakeArmMoveToPosition(intakeArm, IntakeArmPosition.TOP));       // Right Stick Up (Negative Y-Axis)
        // controller.axisGreaterThan(5, 0.2).onTrue(RobotCommands.intakeArmMoveToPosition(intakeArm, IntakeArmPosition.INTERMEDIATE));       // Right Stick Down (Positive Y-Axis)
        // controller.x().onTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm));//.onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));  //TEMP TEMP 
        // controller.y().onTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm));//.onFalse(RobotCommands.intakeArmGiveZeroVoltage(intakeArm));  //TEMP TEMP
        // controller.x().onTrue(RobotCommands.intakeArmMoveToPosition(intakeArm, IntakeArmPosition.TOP));  //TEMP TEMP 
        // controller.y().onTrue(RobotCommands.intakeArmMoveToPosition(intakeArm, IntakeArmPosition.INTERMEDIATE));   //TEMP TEMP
        
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
