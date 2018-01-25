package bayes.additional;

import robocode.*;
import java.util.LinkedList;
import java.util.ListIterator;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;


public class DataFiller {

    public static Matrix fillDecisionInput_MTGTR(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap,
            long lastTimeTG, long lastTimeTR, Matrix decisionInput) {
        decisionInput.inplaceScale(0);
        fillMatrixCircleT(robot, observationsMap, decisionInput, 0, 16);
        fillMatrixCircleD(robot, observationsMap, decisionInput, 16, 16);
        fillTimesSince(robot, lastTimeTG, lastTimeTR, decisionInput, 32);
        fillMyScaledPosition(robot, decisionInput, 35);

        fillMyScaledAngle(robot, decisionInput, 37);
        fillMyScaledGunAngle(robot, decisionInput, 38);
        fillMyScaledRadarAngle(robot, decisionInput, 39);
        fillRandom(decisionInput, 40);
        return decisionInput;
    }

    public static Matrix fillMatrixCircleT(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap,
                                            Matrix out, int start, int nOfSectors) {
        Iterator<Map.Entry<String, LinkedList<Matrix>>> it = observationsMap.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, LinkedList<Matrix>> pair = it.next();
            LinkedList<Matrix> obsList = pair.getValue();
            if(obsList.size() == 0) {
                continue;
            }

            Matrix obs = obsList.getLast();
            double a = DataShaper.getAngleToPoint(robot, obs.get(0, 0), obs.get(1, 0));
            if (a < 0) {
                a = 2 * Math.PI + a;
            }
            int i = (int)((nOfSectors * a) / (2 * Math.PI));
            double t = DataShaper.getScaledTime(obs.get(2, 0), robot.getTime());

            out.set(start + i, 0, out.get(start + i, 0) + Math.exp(-t));

        }
        return out;
    }

    public static Matrix fillMatrixCircleD(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap, Matrix out, int start, int nOfSectors) {
        Iterator<Map.Entry<String, LinkedList<Matrix>>> it = observationsMap.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, LinkedList<Matrix>> pair = it.next();
            LinkedList<Matrix> obsList = pair.getValue();
            if(obsList.size() == 0) {
                continue;
            }

            Matrix obs = obsList.getLast();
            double a = DataShaper.getAngleToPoint(robot, obs.get(0, 0), obs.get(1, 0));
            if (a < 0) {
                a = 2 * Math.PI + a;
            }
            int i = (int)((nOfSectors * a) / (2 * Math.PI));
            double robotsBody = Math.hypot(robot.getWidth(), robot.getHeight());
            double d = DataShaper.getScaledDistance(robot, obs.get(0, 0), obs.get(1, 0));

            out.set(start + i, 0, out.get(start + i, 0) + Math.exp(-d));
        }
        return out;
    }

    public static Matrix fillTimesSince(Robot r, long lastTimeTG, long lastTimeTR, Matrix out, int start) {
        double timeSinveMove = 0; // TODO change after moving introduced
        double timeSinceTG = DataShaper.getScaledTime(lastTimeTG, r.getTime());
        double timeSinceTR = DataShaper.getScaledTime(lastTimeTR, r.getTime());

        out.set(start, 0, timeSinveMove);
        out.set(start + 1, 0, timeSinceTG);
        out.set(start + 2, 0, timeSinceTR);
        return out;
    };

    public static Matrix fillMyScaledPosition(Robot r, Matrix out, int start) {
        out.set(start, 0,
                ((double)r.getX()) / ((double) r.getBattleFieldWidth()));
        out.set(start + 1, 0,
                ((double)r.getY()) / ((double) r.getBattleFieldHeight()));
        return out;
    }

    public static Matrix fillHisScaledPosition(Robot r, Matrix out, ScannedRobotEvent e, int start) {
        double[] position = DataShaper.getAbsPositionArray(r, e);
        out.set(start, 0,
                position[0] / ((double) r.getBattleFieldWidth()));
        out.set(start + 1, 0,
                position[1] / ((double) r.getBattleFieldHeight()));
        return out;
    }

    public static Matrix fillMyScaledAngle(Robot r, Matrix out, int start) {
        out.set(start, 0, DataShaper.getHeadingRadiansFromX(r));
        return out;
    }

    public static Matrix fillMyScaledGunAngle(Robot r, Matrix out, int start) {
        out.set(start, 0, DataShaper.getGunHeadingFromX(r));
        return out;
    }

    public static Matrix fillMyScaledRadarAngle(Robot r, Matrix out, int start) {
        out.set(start, 0, DataShaper.getRadarHeadingFromX(r));
        return out;
    }

    public static Matrix fillAngleToTurn(Robot robot, ScannedRobotEvent e, Matrix out, int start) {
        double targetA = DataShaper.getAngleToPoint(robot, DataShaper.getAbsPosition(robot, e));
        double toTurn = DataShaper.truncateRad(targetA - DataShaper.getGunHeadingFromX(robot));

        out.set(start, 0, toTurn);
        return out;
    }

    public static Matrix fillDistance(Robot r, ScannedRobotEvent e, Matrix out, int start) {
        double[] position = DataShaper.getAbsPositionArray(r, e);

        double d = DataShaper.getScaledDistance(DataShaper.getDistance(r, position[0], position[1]));
        out.set(start, 0, d);
        return out;
    }

    public static Matrix fillRandom(Matrix out, int start) {
        out.set(start, 0, DataShaper.random(-1, 1));
        return out;
    }

    public static Matrix fillNumberOfObservations(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap, String name,
                        Matrix out, int start) {
        if(observationsMap.containsKey(name))  {
            out.set(start, 0, Math.min(5, ((double)observationsMap.get(name).size()) / 10d));
        } else {
            out.set(start, 0, 0);
        }
        return out;
    }

    public static Matrix fillDeepConvo(Matrix[] in, Matrix out, int start, DeepConvo[][] dcv) {
        Matrix[] intermediate = new Matrix[dcv[0].length];

        for(int i = 0; i < dcv.length; i++) {
            for(int j = 0; j < dcv[i].length; j++) {
                if(i == 0) {
                    intermediate[j] = dcv[i][j].apply(in);
                } else {
                    Matrix localResult = dcv[i][j].apply(intermediate);
                    unrollMatrix(localResult, out, start);
                    start += localResult.rows() * localResult.cols();
                }
            }
        }
        return out;
    }

    public static void unrollMatrix(Matrix in, Matrix out, int start) {
        for(int r = 0; r < in.rows(); r++) {
            for(int c = 0; c < in.cols(); c++) {
                out.set(start++, 0, in.get(r, c));
            }
        }
    }

    public static void clearFromOldObservations(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap,
                                                    double timeLimit) {
        Iterator<Map.Entry<String, LinkedList<Matrix>>> it = observationsMap.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, LinkedList<Matrix>> pair = it.next();
            LinkedList<Matrix> obsList = pair.getValue();
            Iterator<Matrix> observationsIt = obsList.iterator();
            while (observationsIt.hasNext()) {
                Matrix obs = observationsIt.next();
                double t = DataShaper.getScaledTime(obs.get(2, 0), robot.getTime());
                if (t > timeLimit || obsList.size() > 10000) {
                    observationsIt.remove();
                }
            }
        }
    }

    public static Matrix fillAdditionalInputToRotateNet(Robot r, Matrix inputToNet) {
        fillMyScaledPosition(r, inputToNet, 16);
        fillMyScaledAngle(r, inputToNet, 18);
        fillRandom(inputToNet, 19);

        fillMyScaledGunAngle(r, inputToNet, 20);
        fillMyScaledRadarAngle(r, inputToNet, 21);
        return inputToNet;
    }

    public static void fillInputMatricesRotate(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap,
     Matrix[] inputMatricesRotate, int contractCoeff) {

        for (int i = 0; i < inputMatricesRotate.length; i++) {
            inputMatricesRotate[i].inplaceScale(0);
        }

        Iterator<Map.Entry<String, LinkedList<Matrix>>> it = observationsMap.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, LinkedList<Matrix>> pair = it.next();
            LinkedList<Matrix> obsList = pair.getValue();
            Iterator<Matrix> observationsIt = obsList.descendingIterator();

            int index = 0;
            while (observationsIt.hasNext() && index < inputMatricesRotate.length) {
                Matrix obs = observationsIt.next();

                double t = DataShaper.getScaledTime(obs.get(2, 0), robot.getTime());
                Matrix mTime = inputMatricesRotate[index++];
                Matrix mDist = inputMatricesRotate[index++];

                int offsetX = (mTime.rows() / 2);
                int offsetY = (mTime.cols() / 2);
                if(Math.abs(obs.get(0, 0) - robot.getX()) <  offsetX * contractCoeff
                    && Math.abs(obs.get(1, 0) - robot.getY()) <  offsetY * contractCoeff) {

                    int r = toMatIndex((int)(obs.get(0, 0) - robot.getX()) +
                        offsetX * contractCoeff, contractCoeff, mTime.rows());
                    int c = toMatIndex((int)(obs.get(1, 0) - robot.getY()) +
                        offsetY * contractCoeff, contractCoeff, mTime.cols());

                    mTime.set(r, c, mTime.get(r, c) + Math.exp(-t));



                    double d = DataShaper.getScaledDistance(robot, obs.get(0, 0), obs.get(1, 0));

                    mDist.set(r, c, mDist.get(r, c) + Math.exp(-d));
                }
            }
        }
    }

    public static int toMatIndex(double x, int contractCoeff, int sideLength) {
        return Math.min(sideLength, Math.max(0, ((int)x) / contractCoeff));
    }

    public static Matrix fillTimesToReach(Matrix m, Matrix myPos, Matrix hisPos,
                            double v, double a, int contractCoeff) {
        //y=kx+b
        if( v < 0 ) {
            return fillTimesToReach(m, myPos, hisPos, -v, Math.PI + a, contractCoeff);
        }

        double upscaleTime = 10;
        double expC = 3;

        double hisX = hisPos.get(0, 0);
        double hisY = hisPos.get(1, 0);

        double myX = myPos.get(0, 0);
        double myY = myPos.get(1, 0);

        double k = Math.tan(a);
        double b = hisY - k * hisX;


        m.inplaceScale(0);

        double offsetX = ((double)m.rows() / 2d) * contractCoeff;
        double offsetY = ((double)m.cols() / 2d) * contractCoeff;

        double ceilOffset = contractCoeff / 2d;

        for(int r = 0; r < m.rows(); r++) {
            double absX = r * contractCoeff + myX - offsetX;

            double absY = k * absX + b;
            double yRelMe = absY - myY;

            if(-offsetY <= yRelMe + ceilOffset && yRelMe + ceilOffset < offsetY) {
                double xRelMe = absX - myX;

                int row = toMatIndex(xRelMe + offsetX, contractCoeff, m.rows());
                int col = toMatIndex(yRelMe + ceilOffset + offsetY, contractCoeff, m.cols());

                double xRelHim = absX - hisX;
                double yRelHim = absY - hisY;

                boolean forward = DataShaper.almostEq(
                    DataShaper.truncateRad(a),
                    DataShaper.truncateRad(Math.atan2(yRelHim, xRelHim))
                );

                double t = upscaleTime * DataShaper.getScaledTime(
                                    DataShaper.linearTimeToReach(
                                        Math.hypot(xRelHim, yRelHim),
                                        (forward ? v : -v)));
                m.set(row, col,
                    Math.max(Math.pow(expC, -t), m.get(row, col)));
            }

            if(-offsetY <= yRelMe - ceilOffset && yRelMe - ceilOffset < offsetY) {
                double xRelMe = absX - myX;

                int row = toMatIndex(xRelMe + offsetX, contractCoeff, m.rows());
                int col = toMatIndex(yRelMe - ceilOffset + offsetY, contractCoeff, m.cols());

                double xRelHim = absX - hisX;
                double yRelHim = absY - hisY;

                boolean forward = DataShaper.almostEq(
                    DataShaper.truncateRad(a),
                    DataShaper.truncateRad(Math.atan2(yRelHim, xRelHim))
                );

                double t = upscaleTime * DataShaper.getScaledTime(
                                    DataShaper.linearTimeToReach(
                                        Math.hypot(xRelHim, yRelHim),
                                        (forward ? v : -v)));
                m.set(row, col,
                    Math.max(Math.pow(expC, -t), m.get(row, col)));
            }
        }

        for(int c = 0; c < m.cols(); c++) {
            double absY = c * contractCoeff + myY - offsetY;

            double absX = (absY - b) / k;
            double xRelMe = absX - myX;

            if(-offsetX <= xRelMe + ceilOffset && xRelMe + ceilOffset < offsetX) {
                double yRelMe = absY - myY;

                int row = toMatIndex(xRelMe + offsetX + ceilOffset, contractCoeff, m.rows());
                int col = toMatIndex(yRelMe + offsetY, contractCoeff, m.cols());

                double xRelHim = absX - hisX;
                double yRelHim = absY - hisY;

                boolean forward = DataShaper.almostEq(
                    DataShaper.truncateRad(a),
                    DataShaper.truncateRad(Math.atan2(yRelHim, xRelHim))
                );

                double t = upscaleTime * DataShaper.getScaledTime(
                                    DataShaper.linearTimeToReach(
                                        Math.hypot(xRelHim, yRelHim),
                                        (forward ? v : -v)));
                m.set(row, col,
                    Math.max(Math.pow(expC, -t), m.get(row, col)));
            }

            if(-offsetX <= xRelMe - ceilOffset && xRelMe - ceilOffset < offsetX) {
                double yRelMe = absY - myY;

                int row = toMatIndex(xRelMe + offsetX - ceilOffset, contractCoeff, m.rows());
                int col = toMatIndex(yRelMe + offsetY, contractCoeff, m.cols());

                double xRelHim = absX - hisX;
                double yRelHim = absY - hisY;

                boolean forward = DataShaper.almostEq(
                    DataShaper.truncateRad(a),
                    DataShaper.truncateRad(Math.atan2(yRelHim, xRelHim))
                );

                double t = upscaleTime * DataShaper.getScaledTime(
                                    DataShaper.linearTimeToReach(
                                        Math.hypot(xRelHim, yRelHim),
                                        (forward ? v : -v)));
                m.set(row, col,
                    Math.max(Math.pow(expC, -t), m.get(row, col)));
            }
        }
        double timeToStop = Math.ceil(v / DataShaper.MAX_D);

        for(int i = 0; i < timeToStop; i++) {
                v = Math.max(0, v - DataShaper.MAX_D);
                hisX = hisX + v * Math.cos(a);
                hisY = hisY + v * Math.sin(a);
        }

        for(int r = 0; r < m.rows(); r++) {
            for(int c = 0; c < m.cols(); c++) {
                double xRelHim = r * contractCoeff - offsetX + myX - hisX;
                double yRelHim = c * contractCoeff - offsetY + myY - hisY;

                double aToTurn = Math.tan(yRelHim / yRelHim) - a;
                aToTurn = Math.atan(Math.tan(aToTurn));

                double t = aToTurn / DataShaper.BODY_TURN_V;
                t += DataShaper.linearTimeToReach(
                    Math.hypot(xRelHim, yRelHim),
                    0
                );

                t = upscaleTime * DataShaper.getScaledTime(t);

                m.set(r, c,
                    Math.max(Math.pow(expC, -t), m.get(r, c)));

            }
        }

        return m;
    }

    public static Matrix fillTimesToShoot(Robot robot, Matrix matrix, int contractCoeff, double bulletEnergy) {
        matrix.inplaceScale(0);

        double offsetX = ((double)matrix.rows() / 2d) * contractCoeff;
        double offsetY = ((double)matrix.cols() / 2d) * contractCoeff;

        for(int r = 0; r < matrix.rows(); r++) {
            for(int c = 0; c < matrix.cols(); c++) {
                double x = r * contractCoeff - offsetX + robot.getX();
                double y = c * contractCoeff - offsetY + robot.getY();

                double targetA = DataShaper.getAngleToPoint(robot, x, y);
                double toTurn = Math.abs(DataShaper.truncateRad(targetA - DataShaper.getGunHeadingFromX(robot)));

                double t = Math.ceil(toTurn / DataShaper.GUN_TURN_V);
                t += (Math.hypot(x, y) / DataShaper.getBulletSpeed(bulletEnergy));

                t = DataShaper.getScaledTime(t);
                matrix.set(r, c, t);

            }
        }
        return matrix;
    }

    public static Matrix fillTrace(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap,
        String name, Matrix matrix,  int contractCoeff, double timeLimit) {

        matrix.inplaceScale(0);

        if(! observationsMap.containsKey(name)) {
            return matrix;
        }

        LinkedList<Matrix> obsList = observationsMap.get(name);
        Iterator<Matrix> observationsIt = obsList.descendingIterator();

        int index = 0;
        while (observationsIt.hasNext()) {
            Matrix obs = observationsIt.next();

            double t = robot.getTime() - obs.get(2, 0);
            t = DataShaper.getScaledTime(t);
            if(t > timeLimit) {
                return matrix;
            }

            int offsetX = (matrix.rows() / 2);
            int offsetY = (matrix.cols() / 2);

            if(Math.abs(obs.get(0, 0) - robot.getX()) <  offsetX * contractCoeff
                && Math.abs(obs.get(1, 0) - robot.getY()) <  offsetY * contractCoeff) {

                int r = toMatIndex((int)(obs.get(0, 0) - robot.getX()) +
                    offsetX * contractCoeff, contractCoeff, matrix.rows());
                int c = toMatIndex((int)(obs.get(1, 0) - robot.getY()) +
                    offsetY * contractCoeff, contractCoeff, matrix.cols());

                matrix.set(r, c, Math.max(matrix.get(r, c), Math.exp(-t)));
            }
        }
        return matrix;
    }

    public static Matrix fillDataDecisionToShoot(Robot robot, HashMap<String, LinkedList<Matrix>> observationsMap,
                                    ScannedRobotEvent e, Matrix decisionInput) {

        decisionInput.inplaceScale(0);

        fillMatrixCircleT(robot, observationsMap, decisionInput, 0, 16);
        fillMatrixCircleD(robot, observationsMap, decisionInput, 16, 16);
        fillMyScaledPosition(robot, decisionInput, 32);
        fillHisScaledPosition(robot, decisionInput, e, 34);

        fillAngleToTurn(robot, e,  decisionInput, 36);
        fillDistance(robot, e, decisionInput, 37);
        fillNumberOfObservations(robot, observationsMap,  e.getName(), decisionInput, 38);

        fillRandom(decisionInput, 39);
        return decisionInput;
    }

    public static Matrix fillAdditionalInputToShoot(Robot robot, ScannedRobotEvent e, Matrix out, int start) {
        fillAngleToTurn(robot, e, out, start);
        return fillRandom(out, start + 1);
    }


    public static void addMovedPoint(Matrix dp ,
            double[] mkabParallel, double[] mkabPerp) {



        double[] mkabParallelNew = DataShaper.updateNormalGamma(mkabParallel, dp.get(0, 0));
        double[] mkabPerpNew = DataShaper.updateNormalGamma(mkabPerp, dp.get(1, 0));

        for(int i = 0; i < 4; i++) {
            mkabParallel[i] = mkabParallelNew[i];
            mkabPerp[i]     = mkabPerpNew[i];
        }
    }
}
