package bayes.additional;
import robocode.*;


public class DataShaper {
    public static final Mapper sigmoid  = new Mapper() {
		public double map(double  x) {
			return x / (2 * (1 + Math.abs(x))) + 0.5f;
		}
	};

    public static double[] getCartesian(double r, double theta) {
        return new double[] {r * Math.cos(theta), r * Math.sin(theta)};
    }

    public static double truncateRad(double x) {
        return Math.atan2(Math.sin(x), Math.cos(x)); //atan return angles in a range [-pi, pi]
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



    public static double getScaledTime(double from, double to) {
			return getScaledTime(to - from);
	}

    public static double getScaledTime(double time) {
            return time / 100f;
    }

	public static double getScaledDistance(double d) {
			return d / 300f;
	}

    public static double length(Matrix p) {
        return Math.hypot(p.get(0, 0), p.get(1, 0));
    }


    public static double getBulletSpeed(double energy) {
        return 20 - 3 * energy;
    }

    public static final double MAX_V = 8;
    public static final double MAX_A = 1;
    public static final double MAX_D = 2;
    public static final double BODY_TURN_V = Math.toRadians(10);
    public static final double GUN_TURN_V = Math.toRadians(20);


    public static boolean almostEq(double d1, double d2) {
        return almostZero(Math.abs(d1 - d2));
    }

    public static boolean almostZero(double d1) {
        return Math.abs(d1 - d2) < 0.0001;
    }





    public static Matrix toVelocityCoordinares(double vSimple, double a, long t, Matrix p) {
        double vtx  = vSimple * Math.cos(a) * t;
        double vty =  vSimple * Math.sin(a) * t;

        double vtxPerp  = vSimple * Math.cos(a + Math.PI / 2) * t;
        double vtyPerp  = vSimple * Math.sin(a + Math.PI / 2) * t;

        Matrix ax0 = new Matrix(2, 1, new double[] {vtx, 	vty});
        Matrix ax1 = new Matrix(2, 1, new double[] {vtxPerp, vtyPerp});
        Matrix dp = DataShaper.toNewOrthoCoords(p , ax0, ax1);

        return dp;
    }

    public static Matrix fromVelocityCoordinares(double vSimple, double a, long t, Matrix p) {
        double vtx  = vSimple * t * Math.cos(a);
        double vty =  vSimple * t * Math.sin(a);

        double vtxPerp  = vSimple * t * Math.cos(a + Math.PI / 2);
        double vtyPerp  = vSimple * t * Math.sin(a + Math.PI / 2);

        Matrix ax0 = new Matrix(2, 1, new double[] {vtx, 	vty});
        Matrix ax1 = new Matrix(2, 1, new double[] {vtxPerp, vtyPerp});
        Matrix dp = DataShaper.fromOldOrthoCoords(p , ax0, ax1);

        return dp;
    }

    public static Matrix fromOldOrthoCoords(Matrix v, Matrix ax0, Matrix ax1) {
        double s0 = Math.hypot(ax0.get(0, 0), ax0.get(1, 0));
        double s1 = Math.hypot(ax1.get(0, 0), ax1.get(1, 0));

        Matrix inverseRotate = new Matrix(2, 2, new double[] {
            ax0.get(0, 0) / s0,  ax1.get(0, 0) / s1,
            ax0.get(1, 0) / s0,  ax1.get(1, 0) / s1
        });

        Matrix inverseScale = new Matrix(2,2, new double[] {
            s0, 0,
            0, s1
        });

        return inverseRotate.multiply(inverseScale.multiply(v));
    }

    public static Matrix toNewOrthoCoords(Matrix v, Matrix ax0, Matrix ax1) {
        double s0 = Math.hypot(ax0.get(0, 0), ax0.get(1, 0));
        double s1 = Math.hypot(ax1.get(0, 0), ax1.get(1, 0));

        Matrix rotate = new Matrix(2, 2, new double[] {
            ax0.get(0, 0) / s0, ax0.get(1, 0) / s0,
            ax1.get(0, 0) / s1, ax1.get(1, 0) / s1
        });

        Matrix scale = new Matrix(2,2, new double[] {
            1 / s0, 0,
            0, 1 / s1
        });

        return scale.multiply(rotate.multiply(v));
    }



    public static double simplifiedV(double v) {
        v = Math.signum(v) * 8;
		if(v == 0) {
			v = 8;
		}
        return v;
    }

    public static double getHistoryX(Matrix h) {
        return h.get(0, 0);
    }

    public static double getHistoryY(Matrix h) {
        return h.get(1, 0);
    }

    public static double getHistoryTime(Matrix h) {
        return h.get(2, 0);
    }

    public static double getHistoryVelocity(Matrix h) {
        return h.get(3, 0);
    }

    public static double getHistoryHeading(Matrix h) {
        return h.get(4, 0);
    }

    public static double getHistoryDV(Matrix h) {
        return h.get(5, 0);
    }

    public static double calculateDV(double vCurrent, double vOld) {
        if(Math.signum(vCurrent) != Math.signum(vOld)) {
            return Math.abs(vCurrent - vOld);
        }
        return Math.abs(vCurrent) - Math.abs(vOld);
    }

    public static double getHistoryDT(Matrix h) {
        return h.get(6, 0);
    }

    public static boolean isStopMode(Matrix oldData) {
        return DataShaper.getHistoryDV(oldData) < 0 || //stopping or stoped
                    DataShaper.getHistoryVelocity(oldData) == 0;
    }



    public static double getEnergyFromDistance(double distance) {
        if(distance > 600) {
            return 1.1;
        }

        if(distance > 400) {
            return 1.5;
        }

        if(distance > 300) {
            return 2;
        }

        if(distance > 200) {
            return 2.5;
        }
        return 3;
    }
}
