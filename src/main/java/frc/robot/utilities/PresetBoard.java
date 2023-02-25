package frc.robot.utilities;

public class PresetBoard {
    public static int kA = 1;
    public static int kB = 2;
    public static int kX = 3;
    public static int kY = 4;
    public static int kLB = 5;
    public static int kRB = 6;
    public static int kShare = 7;
    public static int kOptions = 8;
    public static int kL3 = 9;
    public static int kR3 = 10;

    public static int kXAxis = 0;
    public static int kYAxis = 1;
    public static int kLTAxis = 2;
    public static int kRTAxis = 3;

    /* Code to test it */
    // new RepeatCommand(new InstantCommand(
      // () -> System.out.format("%s, %.2f, %.2f\n", armSubsystem.getEndEffectorPosition().toString(), armSubsystem.getLowerPosition(), armSubsystem.getUpperPosition())
      // () -> System.out.format("%b, %b, %b, %b, %b\n",
      //   PresetBoard.povIsActive(presetBoard.getPOV()),
      //   PresetBoard.povIsUpwards(presetBoard.getPOV()),
      //   PresetBoard.povIsLeftwards(presetBoard.getPOV()),
      //   PresetBoard.povIsDownwards(presetBoard.getPOV()),
      //   PresetBoard.povIsRightwards(presetBoard.getPOV())
      // )
      // () -> System.out.format("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
      //   presetBoard.getRawButton(PresetBoard.kA),
      //   presetBoard.getRawButton(PresetBoard.kB),
      //   presetBoard.getRawButton(PresetBoard.kX),
      //   presetBoard.getRawButton(PresetBoard.kY),
      //   presetBoard.getRawButton(PresetBoard.kLB),
      //   presetBoard.getRawButton(PresetBoard.kRB),
      //   presetBoard.getRawButton(PresetBoard.kShare),
      //   presetBoard.getRawButton(PresetBoard.kOptions),
      //   presetBoard.getRawButton(PresetBoard.kL3),
      //   presetBoard.getRawButton(PresetBoard.kR3)
      // )
    //   () -> System.out.format("%d, %d, %d, %d",
    //     PresetBoard.isTriggerActive(presetBoard.getRawAxis(PresetBoard.kXAxis)),
    //     PresetBoard.isTriggerActive(presetBoard.getRawAxis(PresetBoard.kYAxis)),
    //     PresetBoard.isTriggerActive(presetBoard.getRawAxis(PresetBoard.kLTAxis)),
    //     PresetBoard.isTriggerActive(presetBoard.getRawAxis(PresetBoard.kRTAxis))
    //   )
    // )).ignoringDisable(true).schedule();

    public static boolean povIsActive(int pov) {
        return pov >= 0;
    }

    public static boolean povIsUpwards(int pov) {
        return povIsActive(pov) && (pov <= 45 || pov >= 315);
    }

    public static boolean povIsDownwards(int pov) {
        return povIsActive(pov) && (pov >= 135 && pov <= 225);
    }

    public static boolean povIsRightwards(int pov) {
        return povIsActive(pov) && (pov >= 45 && pov <= 135);
    }

    public static boolean povIsLeftwards(int pov) {
        return povIsActive(pov) && (pov >= 225 && pov <= 315);
    }

    public static boolean isTriggerActive(double trigger) {
        return (trigger >= 0.5) || (trigger <= -0.5);
    }
}
