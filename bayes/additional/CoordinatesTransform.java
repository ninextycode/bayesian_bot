package bayes.additional;


public class CoordinatesTransform {

    public static Matrix toVelocityCoordinares(double v, double a, long t, Matrix p) {
        double vSimple = DataShaper.simplifiedV(v);
        double vtx  = vSimple * Math.cos(a) * t;
        double vty =  vSimple * Math.sin(a) * t;

        double vtxPerp  = vSimple * Math.cos(a + Math.PI / 2) * t;
        double vtyPerp  = vSimple * Math.sin(a + Math.PI / 2) * t;

        Matrix ax0 = new Matrix(2, 1, new double[] {vtx, 	vty});
        Matrix ax1 = new Matrix(2, 1, new double[] {vtxPerp, vtyPerp});
        Matrix dp = CoordinatesTransform.toNewOrthoCoords(p , ax0, ax1);

        return dp;
    }

    public static Matrix fromVelocityCoordinares(double v, double a, long t, Matrix p) {
        double vSimple = DataShaper.simplifiedV(v);
        double vtx  = vSimple * t * Math.cos(a);
        double vty =  vSimple * t * Math.sin(a);

        double vtxPerp  = vSimple * t * Math.cos(a + Math.PI / 2);
        double vtyPerp  = vSimple * t * Math.sin(a + Math.PI / 2);

        Matrix ax0 = new Matrix(2, 1, new double[] {vtx, 	vty});
        Matrix ax1 = new Matrix(2, 1, new double[] {vtxPerp, vtyPerp});
        Matrix dp = CoordinatesTransform.fromOldOrthoCoords(p , ax0, ax1);

        return dp;
    }

    public static Matrix fromOldOrthoCoords(Matrix v, Matrix ax0, Matrix ax1) {
        double s0 = DataShaper.hypot(ax0);
        double s1 = DataShaper.hypot(ax1);

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
        double s0 = DataShaper.hypot(ax0);
        double s1 = DataShaper.hypot(ax1);

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

}
