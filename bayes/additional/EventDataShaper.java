public class EventShaper {

        public static double getHeadingRadiansFromX(ScannedRobotEvent e) {
            return truncateRad(angleFromX(e.getHeadingRadians()));
        }

        public static Matrix getVelocity2D(ScannedRobotEvent e) {
            double v = e.getVelocity();
            double a = getHeadingRadiansFromX(e);
            return
                new Matrix(2, 1, new double[] {
                    v * Math.cos(a), v * Math.sin(a)
                });
        }

        public static double getBearingRadiansFromX(Robot r, ScannedRobotEvent e) {
    		return truncateRad(angleFromX(e.getBearingRadians() + getHeadingRadiansFromY(r)));
    	}

    	public static double getBearingRadiansFromX(Robot r, HitRobotEvent e) {
    		return truncateRad(angleFromX(e.getBearingRadians() + getHeadingRadiansFromY(r)));
        }

         public static double getBearingRadiansFromX(Robot r, HitByBulletEvent e) {
           return truncateRad(angleFromX(e.getBearingRadians() + getHeadingRadiansFromY(r)));
         }

        public static double[] getAbsPositionArray(Robot r, ScannedRobotEvent e) {
            double[] pos =  getCartesian(e.getDistance(),  getBearingRadiansFromX(r, e));
            pos[0] += r.getX();
            pos[1] += r.getY();
            return pos;
        }

        public static Matrix getAbsPosition(Robot r, ScannedRobotEvent e) {
            return new Matrix(2, 1, getAbsPositionArray(r, e));
        }


}
