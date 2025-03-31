package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;
import java.util.function.Supplier;

public class ClimbCommand extends Command {
  private final Climb climber;

  private final Supplier<Double> velocity;

  public ClimbCommand(Climb climber, Supplier<Double> velocity) {

    this.climber = climber;
    this.velocity = velocity;

    addRequirements(climber);
  }

  @Override
  public void execute() {

    climber.setVelocity(velocity.get());
  }
}
