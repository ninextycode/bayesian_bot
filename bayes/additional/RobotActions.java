package bayes.additional;
import robocode.*;

public class RobotActions {
    public static void moveAt(Robot r, double x, double y) {
        double a = DataShaper.getAngleToPoint(r, x, y);

        double toTurn2PI = DataShaper.truncateRad(a - DataShaper.getHeadingRadiansFromX(r));
        double distance = Math.hypot(x - r.getX(), y - r.getY());

        double toTurnPI = Math.atan(Math.tan(toTurn2PI)); //to [-pi/2; pi/2] interval

        turnRad(r, toTurnPI);
        if(Math.abs(toTurn2PI - toTurnPI) > 0.0001) {
            distance = -distance;
        }
        r.ahead(distance);
    }

    public static void moveAt(Robot r, Matrix p) {
        moveAt(r, p.get(0, 0), p.get(1, 0));
    }

    public static void moveAtFaceForward(Robot r, Matrix p) {
        moveAtFaceForward(r, p.get(0, 0), p.get(1, 0));
    }

    public static void moveAtFaceForward(Robot r, double x, double y) {
        double a = DataShaper.getAngleToPoint(r, x, y);
        double aForTargeters = a + Math.PI / 2;

        double toTurn2PI = DataShaper.truncateRad(a - DataShaper.getHeadingRadiansFromX(r));
        double distance = Math.hypot(x - r.getX(), y - r.getY());

        boolean isBodyOk = DataShaper.almostEq(
            DataShaper.truncateRad(
                DataShaper.getHeadingRadiansFromX(r) - a), 0);

        boolean isGunOk = DataShaper.almostEq(
            DataShaper.truncateRad(
                DataShaper.getGunHeadingFromX(r) - aForTargeters), 0);

        boolean isRadarOk = DataShaper.almostEq(
            DataShaper.truncateRad(
                DataShaper.getRadarHeadingFromX(r) - aForTargeters), 0);

        if(isRadarOk) {
        	r.setAdjustRadarForGunTurn(true);
            r.setAdjustRadarForRobotTurn(true);
        }

        if(isGunOk) {
            r.setAdjustGunForRobotTurn(true);
        }

        turnRad(r, toTurn2PI);
        if(!isRadarOk) {
            rotateRadarRadAt(r, DataShaper.getGunHeadingFromX(r));
        }
        if(!isGunOk) {
            rotateGunRadAt(r, aForTargeters);
        }

        if(isRadarOk) {
            r.setAdjustRadarForGunTurn(false);
            r.setAdjustRadarForRobotTurn(false);
        }

        if(isGunOk) {
            r.setAdjustGunForRobotTurn(false);
        }

        r.ahead(distance);
    }

    public static void rotateGunAtPoint(Robot r, Matrix t) {
        rotateGunAtPoint(r, t.get(0, 0), t.get(1, 0));
    }

    public static void rotateGunAtPoint(Robot r, double x, double y) {
        double a = DataShaper.getAngleToPoint(r, x, y);

        double toTurn2PI = DataShaper.truncateRad(a - DataShaper.getGunHeadingFromX(r));

        rotateGunRad(r, toTurn2PI);
    }

    public static void rotateRadarAtPoint(Robot r, Matrix t) {
        rotateRadarAtPoint(r, t.get(0, 0), t.get(1, 0));
    }

    public static void rotateRadarAtPoint(Robot r, double x, double y) {
        double a = DataShaper.getAngleToPoint(r, x, y);

        double toTurn2PI = DataShaper.truncateRad(a -
            DataShaper.getRadarHeadingFromX(r));

        rotateRadarRad(r, toTurn2PI);
    }


    public static void shootAt(Robot r, Matrix t, double energy) {
        shootAt(r, t.get(0, 0), t.get(1, 0),  energy);
    }

    public static void shootAt(Robot r, Matrix t) {
        shootAt(r, t, 1);
    }

    public static void shootAt(Robot r, double x, double y) {
        shootAt(r, x, y, 1);
    }

    public static void shootAt(Robot r, double x, double y, double energy) {
        rotateGunAtPoint(r, x, y);
        r.fire(energy);
    }

    public static void rotateRadarRad(Robot r, double rad) {
		rad = DataShaper.truncateRad(rad);
		r.turnRadarLeft(Math.toDegrees(rad));
	}

	public static void rotateGunRad(Robot r, double rad) {
		rad = DataShaper.truncateRad(rad);
		r.turnGunLeft(Math.toDegrees(rad));
	}

	public static void turnRad(Robot r, double rad) {
		rad = DataShaper.truncateRad(rad);
		r.turnLeft(Math.toDegrees(rad));
	}

    public static void turnRadAt(Robot r, double target) {
        double a = target - DataShaper.getHeadingRadiansFromX(r);
        turnRad(r, a);
    }

    public static void rotateRadarRadAt(Robot r, double target) {
        double a = target - DataShaper.getRadarHeadingFromX(r);
        rotateRadarRad(r, a);
    }

    public static void rotateGunRadAt(Robot r, double target) {
        double a = target - DataShaper.getGunHeadingFromX(r);
        rotateGunRad(r, a);
    }

    public static void rotateRadarByMatrtix(Robot r, Matrix m) {
        rotateRadarRadAt(r, getTargetFromMatrix(m));
    }

    public static void rotateGunByMatrtix(Robot r, Matrix m) {
        rotateGunRadAt(r, getTargetFromMatrix(m));
    }

    private static double getTargetFromMatrix( Matrix m) {
        double max = -1;
        double maxi = 0;

        for(int i = 0; i < m.rows(); i++) {
            if (m.get(i, 0) > max) {
                max = m.get(i, 0);
                maxi = i;
            }
        }

        double a = 2 * Math.PI / ((double)m.rows()) * maxi;
        return a;
    }

    private void randomMove(Robot r, double mind, double maxd) {
        r.ahead(Math.signum(DataShaper.random(-1, 1)) * DataShaper.random(mind, maxd));
    }

    private  void randomRotate(Robot r, double mina, double maxa) {
        turnRad(r, Math.signum(DataShaper.random(-1, 1)) * DataShaper.random(mina, maxa));
    }

	public static boolean standFlankToBearing(Robot r, double bearing) {
        double alpha =
            DataShaper.truncateRad(DataShaper.getHeadingRadiansFromX(r) - bearing);

        alpha = Math.atan(Math.tan(alpha));
        if(Math.abs(alpha) < Math.PI / 6) {
            r.setAdjustGunForRobotTurn(true);
            RobotActions.turnRad(r, Math.signum(alpha) * (Math.PI / 2 - Math.abs(alpha)));
            r.setAdjustGunForRobotTurn(false);
            return true;
        }
        return false;
    }
}
