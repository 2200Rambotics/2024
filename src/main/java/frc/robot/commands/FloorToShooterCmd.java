package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.FloorIntakeSubsystem.FloorIntakeState;
import frc.robot.subsystems.ShooterSubsystem.IntakeState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class FloorToShooterCmd extends Command {
  private final FloorIntakeSubsystem floorIntake;
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;
  boolean isForwards;
  ArmPosition target = ArmPosition.Intake;

  public FloorToShooterCmd(FloorIntakeSubsystem floorIntake, ShooterSubsystem shooter, ArmSubsystem arm, boolean isForwards) {
    this.floorIntake = floorIntake;
    this.shooter = shooter;
    this.arm = arm;
    this.isForwards = isForwards;
    addRequirements(floorIntake, shooter, arm);
  }

  /**
   * Sets the target position of the arm to the Intake position.
   */
  @Override
  public void initialize() {
    arm.unsafeSetPosition(target);
  }

  /**
   * Once the robot reaches the Intake position, run the floorIntake and shooter intake motors to either take in or spit out a note.
   */
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Is Shoulder at Position",arm.leftShoulderMotor.atPosition());
    // SmartDashboard.putBoolean("Is Wrist at Position",arm.wristMotor.atPosition());
    if (arm.leftShoulderMotor.atPosition() && arm.wristMotor.atPosition()) {
      if (isForwards) {
        floorIntake.set(FloorIntakeState.Eat);
        shooter.intakeState = IntakeState.Intake;
      } else {
        floorIntake.set(FloorIntakeState.Spit);
        shooter.intakeState = IntakeState.ReverseIntake;
      }
    }
  }

  /**
   * Once finished, halt floorIntake, pull back note into a prefire position, and return the arm to the stowed position.
   */
  @Override
  public void end(boolean interrupted) {
    floorIntake.set(FloorIntakeState.Spit);

    // shooter.intakeState = IntakeState.Preload;
    // arm.unsafeSetPosition(ArmPosition.Stowed);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
