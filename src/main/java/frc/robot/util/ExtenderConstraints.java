package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.LinkedList;

public class ExtenderConstraints {

  public record AngleConstraint(
      double inchesFromGround, double minDegreesFromVertical, double maxDegreesFromVertical) {}

  public static AngleConstraint[] angleConstraints;

  public ExtenderConstraints(String filename, double heightOfHighestConstraintInches) {

    loadConstraintsFromFile(
        Filesystem.getDeployDirectory().toString() + "/" + filename,
        heightOfHighestConstraintInches);
  }

  public void loadTestConstraints() {
    angleConstraints =
        new AngleConstraint[] {
          new AngleConstraint(0, 0, 0),
          new AngleConstraint(12.5, 0, 0),
          new AngleConstraint(25, 0, 0),
          new AngleConstraint(35, 10, 20),
          new AngleConstraint(45, 30, 40),
          new AngleConstraint(75, 50, 60)
        };
  }

  public void loadConstraintsFromFile(String filepath, double heightOfHighestConstraintInches) {

    try (BufferedReader br = new BufferedReader(new FileReader(filepath))) {
      String line;

      int numLines = 0;
      LinkedList<Double> maxList = new LinkedList<>();
      LinkedList<Double> minList = new LinkedList<>();

      while ((line = br.readLine()) != null) {

        String[] parts = line.split(",");

        maxList.add(Double.parseDouble(parts[0]));
        minList.add(Double.parseDouble(parts[1]));

        numLines += 1;
      }

      angleConstraints = new AngleConstraint[numLines];

      for (int i = 0; i < numLines; i++) {
        double inchesFromGround = (i + 1) * (heightOfHighestConstraintInches / numLines);
        double minDegreesFromVertical = minList.get(i);
        double maxDegreesFromVertical = maxList.get(i);

        angleConstraints[i] =
            new AngleConstraint(inchesFromGround, minDegreesFromVertical, maxDegreesFromVertical);

        System.out.println(
            "Inches from ground: "
                + inchesFromGround
                + ", min degrees from vertical: "
                + minDegreesFromVertical
                + ", max degrees from vertical: "
                + maxDegreesFromVertical);
      }

    } catch (IOException e) {
      e.printStackTrace();

      loadTestConstraints();
    }
  }

  public AngleConstraint getAngleConstraint(double inchesFromGround) {
    if (inchesFromGround < angleConstraints[0].inchesFromGround) {
      return angleConstraints[0];
    } else if (inchesFromGround > angleConstraints[angleConstraints.length - 1].inchesFromGround) {
      return angleConstraints[angleConstraints.length - 1];
    }

    var belowConstraint = angleConstraints[0];
    var aboveConstraint = angleConstraints[0];

    for (int i = 0; i < angleConstraints.length; i++) {
      if (angleConstraints[i].inchesFromGround > inchesFromGround) {
        aboveConstraint = angleConstraints[i];

        if (i > 0) {
          belowConstraint = angleConstraints[i - 1];
        } else {
          belowConstraint = angleConstraints[i];
        }
        break;
      }
    }

    // Linear interpolation
    var percent =
        (inchesFromGround - belowConstraint.inchesFromGround)
            / (aboveConstraint.inchesFromGround - belowConstraint.inchesFromGround);

    var minDegreesFromVertical =
        belowConstraint.minDegreesFromVertical
            + (aboveConstraint.minDegreesFromVertical - belowConstraint.minDegreesFromVertical)
                * percent;

    var maxDegreesFromVertical =
        belowConstraint.maxDegreesFromVertical
            + (aboveConstraint.maxDegreesFromVertical - belowConstraint.maxDegreesFromVertical)
                * percent;

    return new AngleConstraint(inchesFromGround, minDegreesFromVertical, maxDegreesFromVertical);
  }
}
