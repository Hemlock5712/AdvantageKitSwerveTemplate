package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.auto.AutoConstants;
import java.util.*;

public class AutoConfigParser {
  public record AutoPart(
      Translation2d note, Translation2d shootingTranslation, Optional<Pose2d> notePickupPose) {}

  public static final Map<Character, Pose2d> pickupPoseMap = new HashMap<>();

  static {
    pickupPoseMap.put('x', AutoConstants.NotePickupLocations.X);
    pickupPoseMap.put('y', AutoConstants.NotePickupLocations.Y);
    pickupPoseMap.put('z', AutoConstants.NotePickupLocations.Z);
  }

  public static final Map<Character, Translation2d> shootingPoseMap = new HashMap<>();

  static {
    shootingPoseMap.put('a', AutoConstants.ShootingTranslations.A);
    shootingPoseMap.put('b', AutoConstants.ShootingTranslations.B);
    shootingPoseMap.put('c', AutoConstants.ShootingTranslations.C);
    shootingPoseMap.put('d', AutoConstants.ShootingTranslations.D);
    shootingPoseMap.put('e', AutoConstants.ShootingTranslations.E);
    shootingPoseMap.put('f', AutoConstants.ShootingTranslations.F);
    shootingPoseMap.put('g', AutoConstants.ShootingTranslations.G);
  }

  /**
   * this method flips the positions based on the driverstation state when it is called
   *
   * @param config the config string
   * @return a list of AutoParts. is empty if the string is invalid
   */
  public static Optional<List<AutoPart>> parseAutoConfig(String config) {
    try {
      final ArrayList<AutoPart> output = new ArrayList<>();
      final String[] parts = config.split("-");
      for (String part : parts) {
        if (part.length() < 2) {
          return Optional.empty();
        }
        final var shootingPose =
            Optional.ofNullable(shootingPoseMap.get(part.charAt(part.length() - 1)));
        final var pickupPose = Optional.ofNullable(pickupPoseMap.get(part.charAt(0)));
        if (shootingPose.isEmpty()) {
          return Optional.empty();
        }
        final var autoParts =
            part.substring(1, part.length() - 1)
                .chars()
                .map(Character::getNumericValue)
                .mapToObj(i -> AutoConstants.AUTO_NOTES[i])
                .map(
                    note ->
                        new AutoPart(
                            AllianceFlipUtil.apply(note),
                            AllianceFlipUtil.apply(shootingPose.get()),
                            pickupPose.map(AllianceFlipUtil::apply)));

        output.addAll(autoParts.toList());
      }
      return Optional.of(output);
    } catch (Exception e) {
      return Optional.empty();
    }
  }
}
