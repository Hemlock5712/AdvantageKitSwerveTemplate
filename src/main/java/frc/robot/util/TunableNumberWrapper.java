package frc.robot.util;

/*
 * copy these two lines to use in a constants class


private static final TunableNumberWrapper tunableTable =
    new TunableNumberWrapper(MethodHandles.lookup().lookupClass());


 MethodHandles.lookup().lookupClass(); gets the name of the static class it is in

 */
public class TunableNumberWrapper {
  private final String ntPrefix;

  public TunableNumberWrapper(Class constansClass) {
    ntPrefix = constansClass.getSimpleName();
  }

  public LoggedTunableNumber makeField(String name, double defaultValue) {
    return new LoggedTunableNumber(ntPrefix + "/" + name, defaultValue);
  }
}
