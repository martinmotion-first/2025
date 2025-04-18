package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.Constants.IntakeArm.IntakeArmPosition;
import frc.robot.rusthounds_util.CoralSim;
import frc.robot.rusthounds_util.ScoreLevel;
import frc.robot.rusthounds_util.CoralSim.CoralSimLocation;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

public class RobotCommands {
    public static ScoreLevel lastScore = ScoreLevel.None;

    public static Command prepareCoralScoreCommand(ScoreLevel level, Elevator elevator, Arm arm, CoralSim coralSim) {
        System.out.println("In prepareCoralScoreCommand w level:" + level);
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        })
                .andThen(Commands.parallel(
                        arm.moveToPositionCommand(() -> armPosition).asProxy(),
                        Commands.waitSeconds(0.5)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command autoPrepareCoralScoreCommand(ScoreLevel level, Elevator elevator, Arm arm,
            CoralSim coralSim) {
        ElevatorPosition elevatorPosition;
        ArmPosition armPosition;
        switch (level) {
            case L1 -> {
                elevatorPosition = ElevatorPosition.L1;
                armPosition = ArmPosition.L1;
            }
            case L2 -> {
                elevatorPosition = ElevatorPosition.L2;
                armPosition = ArmPosition.L2;
            }
            case L3 -> {
                elevatorPosition = ElevatorPosition.L3;
                armPosition = ArmPosition.L3;
            }
            case L4 -> {
                elevatorPosition = ElevatorPosition.L4;
                armPosition = ArmPosition.L4;
            }
            default -> {
                throw new IllegalArgumentException("Invalid ScoreLevel");
            }
        }

        return Commands.runOnce(() -> {
            lastScore = level;
        })
                .andThen(Commands.parallel(
                        Commands.waitSeconds(0.5).andThen(arm.moveToPositionCommand(() -> armPosition)).asProxy(),
                        Commands.waitSeconds(0)
                                .andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())));
    }

    public static Command scoreCoralCommand(CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, CoralSim coralSim) { //Drivetrain drivetrain, 
        // Map<ScoreLevel, Command> commandMap = Map.ofEntries(
        //         Map.entry(
        //                 ScoreLevel.L1, Commands.parallel(
        //                         // drivetrain.moveVoltageTimeCommand(4, 0.5),
        //                         elevator.movePositionDeltaCommand(() -> Constants.Elevator.SCORING_MOVEMENT)
        //                                 .asProxy())),
        //         Map.entry(
        //                 ScoreLevel.L2,
        //                 Commands.parallel(
        //                         arm.movePositionDeltaCommand(() -> Constants.Arm.SCORING_MOVEMENT).asProxy())),
        //         Map.entry(
        //                 ScoreLevel.L3,
        //                 Commands.parallel(
        //                         arm.movePositionDeltaCommand(() -> Constants.Arm.SCORING_MOVEMENT).asProxy())),
        //         Map.entry(
        //                 ScoreLevel.L4,
        //                 Commands.parallel(
        //                         arm.movePositionDeltaCommand(() -> Constants.Arm.SCORING_MOVEMENT).asProxy(),
        //                         Commands.waitSeconds(0.5)
        //                                 .andThen(
        //                                         elevator.movePositionDeltaCommand(
        //                                                 () -> Constants.Elevator.SCORING_MOVEMENT))
        //                                 .asProxy())),
        //         Map.entry(
        //                 ScoreLevel.None,
        //                 Commands.none()));

        // return Commands.select(commandMap, () -> lastScore);


        // return Commands.sequence(
        //     arm.moveToScoreCoral(ArmPosition.HORIZONTAL),
        //     drivetrain.
        // );

        return arm.moveToScoreCoral(ArmPosition.HORIZONTAL);
    }

    public static Command prepareIntakeCoralCommand(Elevator elevator, Arm arm, CoralSim coralSim) {
        System.out.println("In prepareIntakeCoralCommand");
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.ARM_FREE).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()));
    }

    public static Command armOnlyMoveToBottom(Elevator elevator, Arm arm, CoralSim coralSim) {
        return arm.moveToPositionCommand(() -> ArmPosition.BOTTOM);
    }
    public static Command armOnlyMoveToHorizontal(Elevator elevator, Arm arm, CoralSim coralSim) {
        return arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL);
    }
    public static Command elevatorOnlyMoveToBottom(Elevator elevator, Arm arm, CoralSim coralSim) {
        return elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE);
    }
    public static Command elevatorOnlyMoveToL3(Elevator elevator, Arm arm, CoralSim coralSim) {
        return elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L3);
    }
    public static Command elevatorOnlyMoveToPosition(Elevator elevator, ElevatorPosition position) {
        return Commands.sequence(
            elevator.moveToPositionCommand(() -> position).asProxy(),
            elevator.holdCurrentPositionCommand()
            // elevator.testSetVoltage(0)
        );
    }
    public static Command elevatorHoldPosition(Elevator elevator) {
        return elevator.holdCurrentPositionCommand();
    }
    public static Command armHoldPosition(Arm arm) {
        return arm.holdCurrentPositionCommand();
    }
    public static Command elevatorCombinedCommand(Elevator elevator, Arm arm, ElevatorPosition elevatorPosition) {
        // System.out.println("********************************** IN ELEVATOR COMBINED COMMAND ****************************");
        // System.out.println("Elevator detected curren position before moving: " + elevator.getPosition());
        // System.out.println("********************************** END COMBINED COMMAND ****************************");
        ArmPosition armPosition = null;
        if(elevatorPosition == ElevatorPosition.L3){
            armPosition = ArmPosition.L3;
        }
        else if(elevatorPosition == ElevatorPosition.L2){
            armPosition = ArmPosition.L2;
        }else if (elevatorPosition == ElevatorPosition.TOP){
            armPosition = ArmPosition.TOP;
        }else if(elevatorPosition == ElevatorPosition.ALGAE_L2 || elevatorPosition == ElevatorPosition.ALGAE_L3){
            armPosition = ArmPosition.ALGAE;
        } else{
            armPosition = ArmPosition.BOTTOM;
        }

        if(elevator.getPosition() >= ElevatorPosition.ARM_FREE.value && (elevatorPosition == ElevatorPosition.BOTTOM || 
                                                                        elevatorPosition == ElevatorPosition.L2) 
                                                                     && arm.getPosition() <= Constants.kArmPositionSafeLowerLimit){ //negative voltage is up on our (inverted) motor
            return Commands.sequence(
                Commands.parallel(
                    elevator.moveToPositionCommand(() -> ElevatorPosition.ARM_FREE).asProxy(),
                    Commands.waitSeconds(Constants.kSafeElevatorInitialDelay)
                ),
                Commands.parallel(
                    arm.moveToPositionCommandAlternate(armPosition).asProxy(),
                    Commands.waitSeconds(Constants.kSafeElevatorWaitTime).andThen(elevator.moveToPositionCommand(() -> elevatorPosition).asProxy())
                )  
            );
        }else{
            return Commands.parallel(
                elevator.moveToPositionCommand(() -> elevatorPosition).asProxy(),
                arm.moveToPositionCommandAlternate(armPosition).asProxy()
            );
        }

    }

    public static Command elevatorCombinedCommandAutoArmFree(Elevator elevator, Arm arm) {
        ArmPosition armPosition = ArmPosition.BOTTOM;
        ElevatorPosition elevatorPosition = ElevatorPosition.ARM_FREE;

        return Commands.parallel(
            elevator.moveToPositionCommand(() -> elevatorPosition).asProxy(),
            arm.moveToPositionCommandAlternate(armPosition).asProxy()
        ).raceWith(new WaitCommand(3));
    }

    public static Command elevatorCombinedCommandAutoL2(Elevator elevator, Arm arm) {
        ArmPosition armPosition = ArmPosition.L2;
        ElevatorPosition elevatorPosition = ElevatorPosition.L2;
        
        // return Commands.parallel(
        //     elevator.moveToPositionCommand(() -> elevatorPosition).asProxy(),
        //     arm.moveToPositionCommandAlternate(armPosition).asProxy()
        // ).raceWith(new WaitCommand(3));
        return RobotCommands.elevatorCombinedCommand(elevator, arm, elevatorPosition).raceWith(new WaitCommand(3));
    }

    public static Command elevatorCombinedCommandAutoL3(Elevator elevator, Arm arm) {
        ArmPosition armPosition = ArmPosition.L3;
        ElevatorPosition elevatorPosition = ElevatorPosition.L3;
        
        return Commands.parallel(
            elevator.moveToPositionCommand(() -> elevatorPosition).asProxy(),
            arm.moveToPositionCommandAlternate(armPosition).asProxy()
        ).raceWith(new WaitCommand(3));
    }
    public static Command elevatorCombinedCommandAutoScoreCoralL2(Elevator elevator, Arm arm) {
        ArmPosition armPosition = ArmPosition.HORIZONTAL;
        ElevatorPosition elevatorPosition = ElevatorPosition.L2;
        
        return Commands.parallel(
            elevator.moveToPositionCommand(() -> elevatorPosition).asProxy(),
            arm.moveToPositionCommandAlternate(armPosition).asProxy()
        ).raceWith(new WaitCommand(3));
    }

    public static Command elevatorCombinedCommandAutoScoreCoralL3(Elevator elevator, Arm arm) {
        ArmPosition armPosition = ArmPosition.HORIZONTAL;
        ElevatorPosition elevatorPosition = ElevatorPosition.L3;
        
        return Commands.parallel(
            elevator.moveToPositionCommand(() -> elevatorPosition).asProxy(),
            arm.moveToPositionCommandAlternate(armPosition).asProxy()
        ).raceWith(new WaitCommand(3));
    }

    public static Command elevatorCombinedCommandAuto(Elevator elevator, Arm arm, ElevatorPosition elevatorPosition) {
        // System.out.println("********************************** IN ELEVATOR COMBINED COMMAND AUTO ****************************");
        // System.out.println("Elevator detected current position before moving: " + elevator.getPosition());
        // System.out.println("********************************** END COMBINED COMMAND AUTO ****************************");
        ArmPosition armPosition = null;
        if(elevatorPosition == ElevatorPosition.L3){
            armPosition = ArmPosition.L3;
        }
        else if(elevatorPosition == ElevatorPosition.L2){
            armPosition = ArmPosition.L2;
        }else if (elevatorPosition == ElevatorPosition.TOP){
            armPosition = ArmPosition.TOP;
        }else if(elevatorPosition == ElevatorPosition.ALGAE_L2 || elevatorPosition == ElevatorPosition.ALGAE_L3){
            armPosition = ArmPosition.ALGAE;
        } else{
            armPosition = ArmPosition.BOTTOM;
        }

        return Commands.parallel(
            elevator.moveToPositionCommand(() -> elevatorPosition).asProxy(),
            arm.moveToPositionCommandAlternate(armPosition).asProxy()
        );

    }

    public static Command armOnlyMoveToPosition(Arm arm, ArmPosition position) {
        return arm.moveToPositionCommand(() -> position);
    }

    public static Command intakeArmGivePositiveVoltage(IntakeArm intakeArm) {
        return intakeArm.testSetVoltage(1);
    }
    public static Command intakeArmGiveNegativeVoltage(IntakeArm intakeArm) {
        return intakeArm.testSetVoltage(-1);
    }
    public static Command intakeArmGiveZeroVoltage(IntakeArm intakeArm) {
        return intakeArm.testSetVoltage(0);
    }
    public static Command intakeArmMoveToPosition(IntakeArm intakeArm, IntakeArmPosition position) {
        return intakeArm.moveToPositionCommand(() -> position);
    }
    // public static Command intakeArmIntakeAlgae(IntakeArm intakeArm, Intake intake) {
    //     return Commands.sequence(
    //         intakeArm.moveToPositionCommand(() -> IntakeArmPosition.INTERMEDIATE).asProxy(),
    //         intake.setRollerVoltage(3),
    //         Commands.waitSeconds(3)).andThen(intake.setRollerVoltage(0))
    //     );
    // }

    // public static Command intakeArmScoreAlgae(IntakeArm intakeArm, Intake intake) {
    //     return Commands.sequence(
    //             intakeArm.moveToPositionCommand(() -> IntakeArmPosition.INTERMEDIATE),
    //             intake.setRollerVoltage(-12),
    //             intakeArm.moveToPositionCommand(() -> IntakeArmPosition.TOP),
    //             intake.setRollerVoltage(0)
    //     );
    // }

    public static Command elevatorOnlyGivePositiveVoltage(Elevator elevator){
        return elevator.testSetVoltage(7);
    }

    public static Command elevatorOnlyGiveNegativeVoltage(Elevator elevator){
        return elevator.testSetVoltage(-7);
    }

    public static Command elevatorOnlyGiveZero(Elevator elevator){
        return elevator.testSetVoltage(0);
    }

    public static Command armOnlyGivePositiveVoltage(Arm arm){
        return arm.testSetVoltage(2.5);
    }

    public static Command armOnlyGiveNegativeVoltage(Arm arm){
        return arm.testSetVoltage(-2.5);
    }

    public static Command armOnlyGiveZeroVoltage(Arm arm){
        return arm.testSetVoltage(0);
    }

    public static Command kill(Elevator elevator, Arm arm, Intake intake, Climber climber){
        return Commands.sequence(
            Commands.runOnce(() -> arm.testSetVoltage(0)),
            Commands.runOnce(() -> elevator.testSetVoltage(0)),
            Commands.runOnce(() -> climber.setVoltage(0)),
            Commands.runOnce(() -> intake.setRollerVoltage(0))
        );
    }

    public static Command intakeCoralCommand(Elevator elevator, Arm arm, CoralSim coralSim) {
        System.out.println("In intakeCoralCommand");
        return Commands.sequence(
                // prepareIntakeCoralCommand(elevator, arm, coralSim),
                elevatorCombinedCommand(elevator, arm, ElevatorPosition.ARM_FREE),
                // elevatorCombinedCommand(elevator, arm, ElevatorPosition.BOTTOM), //HERE!!!
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                elevatorCombinedCommand(elevator, arm, ElevatorPosition.ARM_FREE)
        );
    }

    public static Command intakeIntoScoreCommand(ScoreLevel level, Elevator elevator, Arm arm, CoralSim coralSim) {
        System.out.println("In intakeIntoScoreCommand");
        return Commands.sequence(
                Commands.parallel(
                        elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.BOTTOM).asProxy()),
                autoPrepareCoralScoreCommand(level, elevator, arm, coralSim).alongWith(
                        Commands.waitSeconds(0.1).andThen(coralSim.setLocationCommand(CoralSimLocation.CLAW))));
    }

    public static Command prepareAlgaeL2RemoveCommand(Elevator elevator, Arm arm) {
        System.out.println("In prepareAlgaeL2RemoveCommand");
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L2).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    public static Command prepareAlgaeL3RemoveCommand(Elevator elevator, Arm arm) {
        System.out.println("In prepareAlgaeL2RemoveCommand");
        return Commands.sequence(
                Commands.parallel(elevator.moveToPositionCommand(() -> ElevatorPosition.ALGAE_L3).asProxy(),
                        arm.moveToPositionCommand(() -> ArmPosition.HORIZONTAL).asProxy()));
    }

    public static Command algaeRemoveCommand(Elevator elevator, Arm arm) { //Drivetrain drivetrain, 
        return Commands.sequence(
                Commands.parallel(
                        // drivetrain.moveVoltageTimeCommand(-2, 0.5),
                        elevator.movePositionDeltaCommand(() -> -0.06).asProxy()));
    }

    // public static Command intakeAlgaeCommand(IntakeArm intakeArm, Intake intake){
    //     return Commands.sequence(
    //         intakeArm.moveToPositionCommand(() -> IntakeArmPosition.TOP),
    //         intake.runRollersCommand(),
    //         Commands.waitSeconds(3),
    //         Commands.runOnce(() -> intake.setRollerVoltage(0))
    //     );
    // }

    // public static Command scoreAlgaeCommand(IntakeArm intakeArm, Intake intake){
    //     return Commands.sequence(
    //         intake.reverseRollersCommand(),
    //         intakeArm.moveToPositionCommand(() -> IntakeArmPosition.INTERMEDIATE),
    //         Commands.waitSeconds(1.5),
    //         Commands.runOnce(() -> intake.setRollerVoltage(0)),
    //         intakeArm.moveToPositionCommand(() -> IntakeArmPosition.TOP)
    //     );
    // }
}
