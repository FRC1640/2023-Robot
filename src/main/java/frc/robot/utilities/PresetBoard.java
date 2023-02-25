package frc.robot.utilities;

import edu.wpi.first.wpilibj.GenericHID;

public class PresetBoard extends GenericHID {
    public static class Button {
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
    }

    public static class Axis {
        public static int kXAxis = 0;
        public static int kYAxis = 1;
        public static int kLTAxis = 2;
        public static int kRTAxis = 3;
    }

    public PresetBoard(int port) {
        super(port);
    }

    public boolean povIsActive() {
        return povIsActive(getPOV());
    }

    public boolean povIsUpwards() {
        int pov = getPOV();
        return povIsActive(pov) && (pov <= 45 || pov >= 315);
    }

    public boolean povIsDownwards() {
        int pov = getPOV();
        return povIsActive(pov) && (pov >= 135 && pov <= 225);
    }

    public boolean povIsRightwards() {
        int pov = getPOV();
        return povIsActive(pov) && (pov >= 45 && pov <= 135);
    }

    public boolean povIsLeftwards() {
        int pov = getPOV();
        return povIsActive(pov) && (pov >= 225 && pov <= 315);
    }

    /*
     * Can be used to treat an axis like a button.
     */
    public boolean getAxisButton(int axis) {
        double value = getRawAxis(axis);
        return (value >= 0.5) || (value <= -0.5);
    }

    private boolean povIsActive(int pov) {
        return pov >= 0;
    }
}
