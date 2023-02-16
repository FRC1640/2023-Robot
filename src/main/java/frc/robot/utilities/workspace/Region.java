package frc.robot.utilities.workspace;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public interface Region {
    public Vector<N2> calcNearestPoint();
    public boolean isInRegion(Vector<N2> vector);
}
