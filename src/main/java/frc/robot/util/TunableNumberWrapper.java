package frc.robot.util;

public class TunableNumberWrapper {
  private final String ntPrefix;

  public TunableNumberWrapper(String ntPrefix) {
    this.ntPrefix = ntPrefix;
  }
  ;

  public TunableNumberWrapper(Class constansClass) {
    ntPrefix = constansClass.getSimpleName();
  }

  public LoggedTunableNumber makeField(String name, double defaultValue) {
    return new LoggedTunableNumber(ntPrefix + "/" + name, defaultValue);
  }
}
