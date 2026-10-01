package frc.robot.lib;
import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
class DriverInputTest {
  @Test void deadbandNeverReversesSignAndCurveIsMonotonicSymmetricAndBounded() {
    for(boolean cubic:new boolean[]{true,false}) {
      for(double x:new double[]{-.08,-.04,-.001,0,.001,.04,.08}) assertEquals(0,DriverInput.shape(x,.08,cubic));
      double previous=0;
      for(int i=9;i<=100;i++) {
        double x=i/100.0, value=DriverInput.shape(x,.08,cubic);
        assertTrue(value>=previous && value<=1); previous=value;
        assertEquals(value,-DriverInput.shape(-x,.08,cubic),1e-12);
      }
      assertEquals(1,DriverInput.shape(1,.08,cubic),1e-12);
      assertEquals(0,DriverInput.shape(Double.NaN,.08,cubic));
    }
  }
}
