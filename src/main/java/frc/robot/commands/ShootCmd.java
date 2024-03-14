package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.IntakeState;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootCmd extends Command {
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;
  public int isForwards = 0;

  public ShootCmd(ArmSubsystem arm, ShooterSubsystem shooter, int isForwards) {
    this.arm = arm;
    this.shooter = shooter;
    this.isForwards = isForwards;
    // addRequirements(shooter);
  }

  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    if (shooter.okToShoot) {
      // isForwards 0 = yes, 1 = no slow, 2 = no fast
      if(arm.target == ArmPosition.Amp2 || isForwards == 2){
        shooter.intakeState = IntakeState.ReverseIntake;
      } else if (isForwards == 1){
        shooter.intakeState = IntakeState.Spit;
      } else {
        shooter.intakeState = IntakeState.ShootNow;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.intakeState = IntakeState.Idle;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
