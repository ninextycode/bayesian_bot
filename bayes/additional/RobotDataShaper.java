package bayes.additional;
import robocode.*;


public class RobotDataShaper {
    public static Matrix getMyPosition(Robot r) {
        return new Matrix(2, 1, new double[] {r.getX(), r.getY()} );
    }

    public static double getHeadingRadiansFromX(Robot r) {
        return truncateRad(angleFromX(getHeadingRadiansFromY(r)));
    }

    public static double getHeadingRadiansFromY(Robot r) {
        return	Math.toRadians(r.getHeading());
    }

    public static double getGunHeadingFromX(Robot r) {
        return truncateRad(angleFromX(Math.toRadians(r.getGunHeading())));
    }

    public static double getRadarHeadingFromX(Robot r) {
        return truncateRad(angleFromX(Math.toRadians(r.getRadarHeading())));
    }

    public static double getGunHeadingRobotAsZero(Robot r) {
        return getGunHeadingFromX(r) - getHeadingRadiansFromX(r);

    public static double getAngleToPoint(Robot r, Matrix point) {
        Matrix shiftedPoint = point.add(getMyPosition(r).inplaceScale(-1));
        return getTheta(shiftedPoint);
    }

    public static double getAngleToPoint(Robot r, double x, double y) {
        return getAngleToPoint(r, new Matrix(2, 1, new double[] {x, y}));
    }

    public static double getAngleToHit(Robot r, double x, double y) {
        return getAngleToPoint(r, new Matrix(2, 1, new double[] {x, y}));
    }

    public static Matrix getVelocity2D(Robot r) {
        double v = 	r.getVelocity();
        double a = getHeadingRadiansFromX(r);
        return
            new Matrix(2, 1, new double[] {
                v * Math.cos(a), v * Math.sin(a)
            });
    }

    //x-axis is from the front
    //y axis is from the left side
    public static Matrix getRotationTransform(Robot r) {
        double a = getHeadingRadiansFromX(r);
        return new Matrix(2, 2, new double[] {
             Math.cos(a), Math.sin(a),
            -Math.sin(a), Math.cos(a)
        });
    }

    public static Matrix transfromWithRobotAsZero(Robot r, Matrix v) {
        v = v.add(getMyPosition(r).inplaceScale(-1));
        return getRotationTransform(r).multiply(v);
    }

    public static boolean isGunAndRadarTogeter(Robot r) {
        return DataShaper.almostEq(
                DataShaper.truncateRad(
                    DataShaper.getRadarHeadingFromX(r) - DataShaper.getGunHeadingFromX(r)),
                0);
    }

    public static double getDistance(Robot r, double x, double y) {
        double d =  Math.hypot(x - r.getX(), y - r.getY());
        double robotBody = Math.hypot(r.getWidth(), r.getHeight());
        return d - robotBody;
    }

    public static double getScaledDistance(Robot r, double x, double y) {
        return getScaledDistance(getDistance(r, x, y));
    }
}
