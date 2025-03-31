package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTunableNumber;

public class ReefTagAlignCommand {

  /*
   * Auto-aligns to the visible tag using a closed-loop system
   */
  public static Command reefTagAlign(Drive drive, Vision vision) {

    LoggedTunableNumber backwardsOffsetInches =
        new LoggedTunableNumber("ReefTagAlign/BackwardsOffsetInches", 12);
    LoggedTunableNumber leftOffsetInches =
        new LoggedTunableNumber("ReefTagAlign/LeftOffsetInches", 12);
    LoggedTunableNumber rightOffsetInches =
        new LoggedTunableNumber("ReefTagAlign/RightOffsetInches", 12);

    return Commands.none();
  }
}
