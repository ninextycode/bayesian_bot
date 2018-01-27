package bayes.additional;
import robocode.*;

import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.ListIterator;

public class HistoryFunctions {
    public static double getHistoryX(Matrix h) {
        return h.get(0, 0);
    }

    public static double getHistoryY(Matrix h) {
        return h.get(1, 0);
    }

    public static Matrix getHistoryXY(Matrix h) {
        return new Matrix(2, 1, new double[]{ h.get(0, 0), h.get(1, 0)});
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

    public static double getHistoryTimeSincePrevious(Matrix h) {
        return h.get(6, 0);
    }

    public static boolean isStopMode(Matrix oldData) {
         //stopping or stoped
        return HistoryFunctions.getHistoryDV(oldData) < 0 ||
                    HistoryFunctions.getHistoryVelocity(oldData) == 0;
    }



    public static Matrix writeMatrixOfObservation(Robot r, ScannedRobotEvent event,
        HashMap<String, LinkedList<Matrix>> observationsMap) {

		Matrix pos = EventDataShaper.getAbsPosition(r, event);

		Matrix data = new Matrix(7, 1, new double[] {
			pos.get(0, 0),  //0
			pos.get(1, 0),  //1
			event.getTime(),    //2

			event.getVelocity(),//3
			EventDataShaper.getHeadingRadiansFromX(event), //4
			0 , //5 acceleration / change in velocity
			100000  //6 time change since last observation of the enemy
                  //initially large, to make first data point meaningless
		});

		if(observationsMap.containsKey(event.getName())) {

			LinkedList<Matrix> obsList = observationsMap.get(event.getName());

			if(obsList.size() > 0) {
				Matrix obsLast = obsList.getLast();
				double dv = HistoryFunctions.calculateDV(
								HistoryFunctions.getHistoryVelocity(data),
								HistoryFunctions.getHistoryVelocity(obsLast));
				data.set(5, 0, dv);

				double dt = HistoryFunctions.getHistoryTime(data) - HistoryFunctions.getHistoryTime(obsLast);
				data.set(6, 0, dt);

				obsList.addLast(data);
			}

		} else {
			LinkedList<Matrix> obsList = new LinkedList<Matrix>();
			obsList.addLast(data);
			observationsMap.put(event.getName(), obsList);
		}
		return data;
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
}
