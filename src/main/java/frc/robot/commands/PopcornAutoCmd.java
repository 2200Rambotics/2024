package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FloorIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.FloorIntakeSubsystem.FloorIntakeState;
import frc.robot.subsystems.ShooterSubsystem.IntakeState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import edu.wpi.first.wpilibj2.command.Command;

public class PopcornAutoCmd extends Command {
  private final FloorIntakeSubsystem floorIntake;
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;
  ArmPosition target = ArmPosition.Popcorn;
  double speed;

  public PopcornAutoCmd(FloorIntakeSubsystem floorIntake, ShooterSubsystem shooter, ArmSubsystem arm, double speed) {
    this.floorIntake = floorIntake;
    this.shooter = shooter;
    this.arm = arm;
    this.speed = speed;
    addRequirements(floorIntake, shooter, arm);
  }

  /**
   * Sets the target position of the arm to the Popcorn position.
   */
  @Override
  public void initialize() {
    arm.unsafeSetPosition(target);

    shooter.shooterState = ShooterState.SpinFixed;
    shooter.shooterV = speed;
  }

  /**
   * Once the robot reaches the Intake position, run the floorIntake and shooter intake motors to either take in or spit out a note.
   */
  @Override
  public void execute() {
    // SmartDashboard.putBoolean("Is Shoulder at Position",arm.leftShoulderMotor.atPosition());
    // SmartDashboard.putBoolean("Is Wrist at Position",arm.wristMotor.atPosition());
    if (arm.leftShoulderMotor.atPosition() && arm.wristMotor.atPosition()) {
    
        // Intake Control
        floorIntake.set(FloorIntakeState.Eat);
        shooter.intakeState = IntakeState.ShootNow;
    
    }
  }

 
  @Override
  public void end(boolean interrupted) {
    floorIntake.set(FloorIntakeState.Spit);

    shooter.shooterState = ShooterState.Idle;
    shooter.spinDownShooters();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}