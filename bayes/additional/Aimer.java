package bayes.additional;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;
import java.util.LinkedList;

public class Aimer {
	private static double[] mkabMoveParallelPrior = new double[] {
		0.7, 10, 3, 1
	};

	private static double[] mkabMovePerpPrior = new double[] {
		0, 10, 3, 1
	};

	private static double[] mkabStopParallelPrior = new double[] {
		0, 10, 3, 1
	};

	private static double[] mkabStopPerpPrior = new double[] {
		0, 10, 3, 1
	};

    private static HashMap<String, BayesianNormal> modelsInMovePrallel =
                                new HashMap<String, BayesianNormal>();

    private static HashMap<String, BayesianNormal> modelsInMovePerp =
                                new HashMap<String, BayesianNormal>();

    private static HashMap<String, BayesianNormal> modelsAfterStopPrarllel =
                                        new HashMap<String, BayesianNormal>();

    private static HashMap<String, BayesianNormal> modelsAfterStopPerp =
                                        new HashMap<String, BayesianNormal>();

    public static final long PREDICT_TIME_LIMIT = 45;
    public static final long REVERSE_VEL_TIME = 12;
	public static final long OBSERVATIONS_NEEDED = 10;

	public boolean wantToShoot(Matrix data, int numberOfObs) {
		return HistoryFunctions.getHistoryTimeSincePrevious(data) <= Aimer.PREDICT_TIME_LIMIT
		 	&& numberOfObs >= OBSERVATIONS_NEEDED;
	}

	public static double maxShootDistance(double energy) {
		return PREDICT_TIME_LIMIT * DataShaper.getBulletSpeed(energy);
	}

    public void updateParameters(LinkedList<Matrix> observationsList,
        String name) {
		preparePriorIfNecessary(name);

		Iterator<Matrix> it = observationsList.descendingIterator();
    	Matrix lastObservationData = it.next();

		while(it.hasNext()) {
			Matrix oldData = it.next();
			double timeSince =
                HistoryFunctions.getHistoryTime(lastObservationData)
                 - HistoryFunctions.getHistoryTime(oldData);

             // only consider observations which are not very old, and
             // which are not far away apart in time from the one before
			if(
				timeSince > PREDICT_TIME_LIMIT
	            || HistoryFunctions.getHistoryTimeSincePrevious(oldData) > REVERSE_VEL_TIME) {

				return;
			}

			double vOld = DataShaper.simplifiedV(HistoryFunctions.getHistoryVelocity(oldData));
			double aOld = HistoryFunctions.getHistoryHeading(oldData);

            Matrix dPos = HistoryFunctions.getHistoryXY(lastObservationData).add(
                HistoryFunctions.getHistoryXY(oldData).inplaceScale(-1)
            );

			Matrix dPosTransformed = CoordinatesTransform.toVelocityCoordinares(
				vOld, aOld,
				(long)timeSince, dPos
			);

            boolean stopped = HistoryFunctions.isStopMode(oldData);

            BayesianNormal parallelModel = getParallelModel(name, stopped);
            BayesianNormal perpModel = getParallelModel(name, stopped);

			parallelModel.update(dPosTransformed.get(0, 0));
			perpModel.update(dPosTransformed.get(0, 1));
		}
	}

    private void preparePriorIfNecessary(String name) {
        if(! modelsInMovePrallel.containsKey(name)) {
            fillPrior(name);
        }
    }

    private void fillPrior(String name) {
        HashMap[] modelsMaps = new HashMap[] {
            modelsInMovePrallel, modelsInMovePerp, modelsAfterStopPrarllel, modelsAfterStopPerp
        };

        double[][] priorParameters = new double[][]{
            mkabMoveParallelPrior, mkabMovePerpPrior, mkabStopParallelPrior, mkabStopPerpPrior
        };

        for(int i = 0; i < priorParameters.length; i++) {
            modelsMaps[i].put(name, new BayesianNormal(priorParameters[i][0], priorParameters[i][1],
                                priorParameters[i][2], priorParameters[i][3]));
        }
    }

    public Aimer() {
    }

    public Matrix suggestEnemyPoitionChange(Matrix data, String name, long time) {

        Matrix targetPosition = HistoryFunctions.getHistoryXY(data);

        double alpha = HistoryFunctions.getHistoryHeading(data);

        double simplifiedV = DataShaper.simplifiedV(event.getVelocity());


        boolean stopped = HistoryFunctions.isStopMode(data);

        BayesianNormal parallelModel = getParallelModel(name, stopped);
        BayesianNormal perpModel = getParallelModel(name, stopped);

        Matrix targetInVCoordinates = new Matrix(2, 1, new double[] {
            parallelModel.sample(),
            perpModel.sample()
        });




        guessedPositionChange = CoordiantesTransform.fromVelocityCoordinares(simplifiedV,
                                                        alpha, bulletFlyTime, targetInVCoordinates);

        return guessedPositionChange;
    }

    private BayesianNormal getParallelModel(String name, boolean stopped) {
        if (stopped) {
            return this.modelsAfterStopPrarllel.get(name);
        } else {
            return this.modelsInMovePrallel.get(name);
        }
    }

    private BayesianNormal getPerpModel(String name, boolean stopped) {
        if (stopped) {
            return this.modelsAfterStopPerp.get(name);
        } else {
            return this.modelsInMovePerp.get(name);
        }
    }
}
