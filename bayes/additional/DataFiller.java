package bayes.additional;

import robocode.*;
import java.util.LinkedList;
import java.util.ListIterator;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;


public class DataFiller {
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
