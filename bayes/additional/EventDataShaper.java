package bayes.additional;
import robocode.*;

public class EventDataShaper {
    public static double getBearingRadiansFromX(Robot r, ScannedRobotEvent e) {
		return DataShaper.truncateRad(
                DataShaper.angleFromX(e.getBearingRadians() + DataShaper.getHeadingRadiansFromY(r)));
	}

	public static double getBearingRadiansFromX(Robot r, HitRobotEvent e) {
		return DataShaper.truncateRad(
                DataShaper.angleFromX(e.getBearingRadians() + DataShaper.getHeadingRadiansFromY(r)));
    }

     public static double getBearingRadiansFromX(Robot r, HitByBulletEvent e) {
       return DataShaper.truncateRad(
                DataShaper.angleFromX(e.getBearingRadians() + DataShaper.getHeadingRadiansFromY(r)));
     }

    public static double[] getAbsPositionArray(Robot r, ScannedRobotEvent e) {
        double[] pos =  DataShaper.getCartesian(e.getDistance(),  getBearingRadiansFromX(r, e));
        pos[0] += r.getX();
        pos[1] += r.getY();
        return pos;
    }

    public static Matrix getAbsPosition(Robot r, ScannedRobotEvent e) {
        return new Matrix(2, 1, getAbsPositionArray(r, e));
    }

    public static double getHeadingRadiansFromX(
            ScannedRobotEvent e) {
        return DataShaper.truncateRad(
            DataShaper.angleFromX(e.getHeadingRadians()));
    }


}
