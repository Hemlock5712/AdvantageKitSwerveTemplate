package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.function.Supplier;

public class DriveController {
  private Optional<Supplier<Rotation2d>> headingSupplier = Optional.empty();

  public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
    this.headingSupplier = Optional.of(headingSupplier);
  }

  public boolean isHeadingControlled() {
    return this.headingSupplier.isPresent();
  }

  public void disableHeadingSupplier() {
    this.headingSupplier = Optional.empty();
  }

  public Rotation2d getHeadingAngle() {
    return headingSupplier.get().get();
  }
}
