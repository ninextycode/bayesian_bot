package bayes.additional;
import robocode.*;


public class BayesianNormal {
    private double m, k, a, b;

    public BayesianNormal(double m, double k, double a, double b) {
        this.m = m;
        this.k = k;
        this.a = a;
        this.b = b;
    }



    public double var() {
        return b / a;
    }

    public double mean() {
        return m;
    }

    public void update(double x) {
        double new_m = (k * m + x) / (k + 1);
        double new_k = k + 1;
        double new_a = a + 0.5d;
        double new_b = b + (k * (x - m) * (x - m) / (2 * (k + 1)));

        a = new_a;
        k = new_k;
        a = new_a;
        b = new_b;
    }

    public double sample() {
        return BayesianNormal.getNormal(mean(), var());
    }

    public static double getNormal(double mean, double var) {
        // Box–Muller transform
        double z1;
        boolean generate;

        double u1, u2;
        do
         {
           u1 = 1 - Math.random();
           u2 = 1 - Math.random();
         }
        while ( u1 <= 0 ); //we dont wont log(0)

        double z0;
        z0 = Math.sqrt(-2.0 * Math.log(u1)) *
            Math.cos(2 * Math.PI * u2);

        return z0 * Math.sqrt(var) + mean;
    }
}
