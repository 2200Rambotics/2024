package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPosition;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class TempSubwooferCmd extends Command {
  private final ArmSubsystem arm;
  private final ShooterSubsystem shooter;
  boolean isFinished = false;
  Timer timer = new Timer();
  
  public TempSubwooferCmd(ArmSubsystem arm, ShooterSubsystem shooter) {
    this.arm = arm;
    this.shooter = shooter;
    addRequirements(arm, shooter);

    isFinished = false;
    timer.restart();
  }

  @Override
  public void initialize() {
    arm.unsafeSetPosition(ArmPosition.SubWoofer);
    shooter.shooterState = ShooterState.SpinFixed;
    shooter.shooterV = Constants.SUBWOOFER_SHOOT_SPEED;
    if(timer.get() > 1.5){
      isFinished = true;
    }
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.shooterState = ShooterState.Idle;
    shooter.spinDownShooters();
    arm.unsafeSetPosition(ArmPosition.Stowed);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
