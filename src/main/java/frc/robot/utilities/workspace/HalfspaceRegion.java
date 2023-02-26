package frc.robot.utilities.workspace;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

public class HalfspaceRegion implements Region {
    Vector<N2> normal;
    public HalfspaceRegion(Vector<N2> normal){
        this.normal = normal;
    }
    @Override
    public Vector<N2> calcNearestPoint() {
        
        return null;
    }
    @Override
    public boolean isInRegion(Vector<N2> vector) {
        // TODO Auto-generated method stub
        return false;
    }
    
}
