package bayes.additional;
import robocode.*;


public class DataShaper {
    public static final double MAX_V = 8;
    public static final double MAX_A = 1;
    public static final double MAX_D = 2;
    public static final double BODY_TURN_V = Math.toRadians(10);
    public static final double GUN_TURN_V = Math.toRadians(20);


    public static double[] getCartesian(double r, double theta) {
        return new double[] {r * Math.cos(theta), r * Math.sin(theta)};
    }

    public static double truncateRad(double x) {
        return Math.atan2(Math.sin(x), Math.cos(x)); //atan2 return angles in a range [-pi, pi]
    }

    public static double truncateRadPi(double x) {
        return Math.atan(Math.tan(x)); //atan return angles in a range [-pi/2, pi/2]
    }

    public static double angleFromX(double angleFromY) {
        return (Math.PI/2 - angleFromY) % (2*Math.PI);
    }

    public static double random(double from, double to)  {
         return (to - from) * Math.random() + from;
    }

    public static double getTheta(Matrix v) {
        return Math.atan2(v.get(1,0), v.get(0,0));
    }

    public static Matrix getMyPosition(Robot r) {
        return new Matrix(2, 1, new double[] {r.getX(), r.getY()});
    }

    public static double getScaledTime(double from, double to) {
			return getScaledTime(to - from);
	}

    public static double getScaledTime(double time) {
            return time / 100f;
    }

	public static double getScaledDistance(double d) {
			return d / 300f;
	}

    public static double hypot(Matrix p) {
        return Math.hypot(p.get(0, 0), p.get(1, 0));
    }


    public static double getBulletSpeed(double energy) {
        return 20 - 3 * energy;
    }


    public static boolean almostEq(double d1, double d2) {
        return almostZero(Math.abs(d1 - d2));
    }

    public static boolean almostZero(double d1) {
        return Math.abs(d1) < 0.0001;
    }


    public static double getEnergyFromDistance(Robot r, double distance) {
        if( r.getEnergy() < 5)
            return 0.1;
            
        if( r.getEnergy() < 15)
            return 0.5;

        return 1;
    }

    //round velocity to have as value of 8
    public static double simplifiedV(double v) {
        v = Math.signum(v) * 8;
		if(v == 0) {
			v = 8;
		}
        return v;
    }

    public static double getHeadingRadiansFromX(Robot r) {
        return truncateRad(angleFromX(getHeadingRadiansFromY(r)));
    }

    public static double getHeadingRadiansFromY(Robot r) {
        return	Math.toRadians(r.getHeading());
    }


    public static double getAngleToPoint(Robot r, Matrix point) {
        Matrix shiftedPoint = point.add(getMyPosition(r).inplaceScale(-1));
        return getTheta(shiftedPoint);
    }

    public static double getAngleToPoint(Robot r, double x, double y) {
        return getAngleToPoint(r, new Matrix(2, 1, new double[] {x, y}));
    }

    public static double getGunHeadingFromX(Robot r) {
        return truncateRad(angleFromX(Math.toRadians(r.getGunHeading())));
    }

    public static double getRadarHeadingFromX(Robot r) {
        return truncateRad(angleFromX(Math.toRadians(r.getRadarHeading())));
    }
}
