package bayes; // replace XXXXXXX with your University ID number

import robocode.*;
import bayes.additional.*;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;

import java.lang.Runnable;
import java.lang.Thread;
import java.util.Arrays;

// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html


public class BayesBot extends Robot	{


    private boolean lastWasTurn = true;
    private boolean lastWasMeaningfull = true;

	private HashMap<String, LinkedList<Matrix>> observationsMap =
									new HashMap<String, LinkedList<Matrix>>();

	private static HashMap<String, BayesianNormal> mkabParallelMove =
								new HashMap<String, BayesianNormal>();

	private static HashMap<String, BayesianNormal> mkabPerpMove =
										new HashMap<String, BayesianNormal>();

	private static HashMap<String, BayesianNormal> mkabParallelStop =
								new HashMap<String, BayesianNormal>();

	private static HashMap<String, BayesianNormal> mkabPerpStop =
										new HashMap<String, BayesianNormal>();


	private double[] mkabParallelPriorMove = new double[] {
		0.7, 10, 3, 1
	};

	private double[] mkabPerpPriorMove = new double[] {
		0, 10, 3, 1
	};


	private double[] mkabParallelPriorStop = new double[] {
		0, 10, 3, 1
	};

	private double[] mkabPerpPriorStop = new double[] {
		0, 10, 3, 1
	};


	private ScannedRobotEvent lastObservation, lastTargetEvent;
	private HitByBulletEvent lastHitEvent;


	private void clearToCheck(String name) {
		if(nameToCheckFor.equals(name)) {
			forceClearToCheck();
		}
	}

	private void clearToCheck() {
		toCheck = new ArrayList<Matrix>();
	}


	private void check() {
		if(insideEvent() || lock) {
			return;
		}

		if(toCheck == null || toCheck.size() == 0) {
			return;
		}
		ArrayList<Matrix> toCheckTemp = toCheck; //to prevent recursion
		toCheck = new ArrayList<Matrix>();

		for(int i = 0; i < toCheckTemp.size(); i++) {
			Matrix check = toCheckTemp.get(i);
			RobotActions.rotateGunAtPoint(this, check.get(0, 0), check.get(1, 0));
		}
	}

    @Override
    public void fire(double energy) {
        if(lastWasTurn) {
            super.fire(energy);
        }
        return;
    }
	@Override
	public void turnLeft(double angleInDegrees) {
		check();
        if(DataShaper.almostEq(angleInDegrees, limit)) {
            return;
        }
		super.turnLeft(angleInDegrees);
	}

	@Override
	public void turnGunLeft(double angleInDegrees) {
		check();
        if(DataShaper.almostEq(angleInDegrees, limit)) {
            return;
        }
        System.out.println("turnGunLeft");
        lastWasTurn=true;
		super.turnGunLeft(angleInDegrees);
        lastWasTurn=true;
	}

	@Override
	public void turnRadarLeft(double angleInDegrees) {
		check();
		super.turnRadarLeft(angleInDegrees);
	}



	@Override
	public void ahead(double distance) {
		check();
		super.ahead(distance);
	}


	private void initialise() {
		wallPadding = 100;
	}

	public void run() {
		initialise();
		while(true) {
			nextAction();
		}
	}



	private void nextAction() {
		if(! isNearWall()) {
			goToClosestWall();
		} else {
			scan();
			patrol();
		}
	}

	private boolean isNearWall() {
		return isNearWall(DataShaper.getMyPosition(this));
	}

	private boolean isNearWall(Matrix point) {
		return isNearWall(0, point) ||
				isNearWall(1, point) ||
			    isNearWall(2, point) ||
				isNearWall(3, point);
	}

	private boolean isNearWall(int wallIndex) {
		return isNearWall(wallIndex, DataShaper.getMyPosition(this));
	}

	private boolean isNearWall(int wallIndex, Matrix point) {
		wallIndex = wallIndex % 4;
		switch (wallIndex) {
			case 0:
				return (getBattleFieldWidth() - point.get(0,0)) <= wallPadding;
			case 1:
				return (getBattleFieldHeight() - point.get(1,0)) <= wallPadding;
			case 2:
				return point.get(0,0) <= wallPadding;
			case 3:
				return point.get(1,0) <= wallPadding;
		}
		return false;
	}

	private void goToClosestWall() {
		int closertWall = getClosestWall() ;
		double[] target = new double[2];

		switch (closertWall) {
			case 0:
				target[0] = getBattleFieldWidth() - wallPadding / 2;
				target[1] = getY();
			break;
			case 1:
				target[0] = getX();
				target[1] =  getBattleFieldHeight() - wallPadding / 2;
			break;
			case 2:
				target[0] = wallPadding / 2;
				target[1] = getY();
			break;
			case 3:
				target[0] = getX();
				target[1] = wallPadding / 2;
			break;
		}

		RobotActions.moveAt(this, target[0], target[1]);
	}


	private int getClosestWall() {
		double[] distances = new double[] {
			getBattleFieldWidth() - getX(),
			getBattleFieldHeight() - getY(),
			getX(),
			getY()
		};

		int minD = 0;
		for(int i = 0; i < distances.length; i++) {
			if(distances[i] < distances[minD]) {
				minD = i;
			}
		}
		return minD;
	}


	private void patrol() {
        lookAround();
		//patrol(true);
	}

	private void patrol(boolean adjustGun) {


		int wall = getClosestWall();
		double[] target = new double[2];
		switch (wall) {
			case 0:
				target[0] = getBattleFieldWidth() - wallPadding / 2;
				target[1] = Math.min(getY() + patrolStep,
										getBattleFieldHeight() - wallPadding/2 + 1);

				if (target[1] >= (getBattleFieldHeight() - wallPadding)) {
					target[1] = getBattleFieldHeight() - wallPadding/2 + 1;
				}
			break;
			case 1:
				target[0] = Math.max(getX() - patrolStep, wallPadding/2 - 1);
				target[1] = getBattleFieldHeight() - wallPadding / 2;
				if (target[0] <= wallPadding) {
					target[0] = wallPadding/2 - 1;
				}
			break;
			case 2:
				target[0] = wallPadding / 2;
				target[1] = Math.max(getY() - patrolStep, wallPadding/2 - 1);
				if (target[1] <= wallPadding) {
					target[1] = wallPadding/2 - 1;
				}
			break;
			case 3:
				target[0] = Math.min(getX() + patrolStep,
										getBattleFieldWidth() - wallPadding/2 + 1);
				if (target[0] >= (getBattleFieldWidth() - wallPadding)) {
					target[0] = getBattleFieldWidth() - wallPadding/2 + 1;
				}
				target[1] = wallPadding / 2;
			break;
		}

		RobotActions.moveAtFaceForward(this, target[0], target[1]);

		lookAround();
	}

	private void lookAround() {
		for(int i = 0; i < 4; i++) {
			RobotActions.rotateGunRad(this, Math.PI / 2);
		}
	}



	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
        insideOnScannedRobot = true;

		Matrix data = writeDataPoint(event);

        lastObservation = event;
		lastOnScannedRobotTime = getTime();

		if(isDuel()) {
			if(event.getName().equals(lastTargetEvent.getName())) {
				double de = event.getEnergy() - lastTargetEvent.getEnergy();
				if(0 > de && de >= -3) {
					mustEvade = true;
				}
			}
		}

       updateParameters(event.getName(), data);

       if(!lastWasTurn) {
           Matrix position = new Matrix(2, 1, new double[] {
               DataShaper.getHistoryX(data),
               DataShaper.getHistoryY(data)
           });

           System.out.println("checkAroundPoint");
           checkAroundPoint(position, DataShaper.getHistoryHeading(data));
       }


        if(mustEvade && !evading && !justEvaded) {
            evading = true;
			setAdjustGunForRobotTurn(true);
			standSideToBearing(DataShaper.getBearingRadiansFromX(this, event));
			setAdjustGunForRobotTurn(false);
			smartRandomAhead(50, 80);
			justEvaded = true;
            mustEvade = false;
            evading = false;
		}

        if(lastWasTurn) {
        	justEvaded = false;
            maybeShoot(event, data);
        }






		DataFiller.clearFromOldObservations(this, observationsMap, timeLimit);

		insideOnScannedRobot = false;
	}

	private Matrix writeDataPoint(ScannedRobotEvent event) {


		Matrix pos = DataShaper.getAbsPosition(this, event);

		Matrix data = new Matrix(7, 1, new double[] {
			pos.get(0, 0),  //0
			pos.get(1, 0),  //1
			event.getTime(),    //2

			event.getVelocity(),//3
			DataShaper.getHeadingRadiansFromX(event), //4
			0 , //5 d*abs(v)
			1000  //6 dt, initially large, to make first data point meaningless

		});

		if(observationsMap.containsKey(event.getName())) {

			LinkedList<Matrix> obsList = observationsMap.get(event.getName());

			if(obsList.size() > 0) {
				Matrix obsLast = obsList.getLast();
				double dv = DataShaper.calculateDV(
								DataShaper.getHistoryVelocity(data),
								DataShaper.getHistoryVelocity(obsLast));
				data.set(5, 0, dv);

				double dt = DataShaper.getHistoryTime(data) - DataShaper.getHistoryTime(obsLast);
				data.set(6, 0, dt);

				obsList.addLast(data);
			}

		} else {
			LinkedList<Matrix> obsList = new LinkedList<Matrix>();
			obsList.addLast(data);
			observationsMap.put(event.getName(), obsList);
		}
		return data;
	};

	private void updateParameters(String name, Matrix data) {
		preparePriorIfNecessary(name);

		LinkedList<Matrix> observationsList = observationsMap.get(name);
		Iterator<Matrix> it = observationsList.descendingIterator();
		while(it.hasNext()) {
			Matrix oldData = it.next();
			long t = lastOnScannedRobotTime - (long)oldData.get(2, 0);
			if(t > maxT) {
				return;
			}
			if(t > minT ) {
				if(DataShaper.getHistoryDT(oldData) > informationTimeLimit) {
					//velocity change was meaningless
					return;
				}

				boolean stopped = DataShaper.isStopMode(oldData);

				double vOld = DataShaper.simplifiedV(DataShaper.getHistoryVelocity(oldData));
				double aOld = DataShaper.getHistoryHeading(oldData);

				double dx = data.get(0, 0) - oldData.get(0, 0);
				double dy = data.get(1, 0) - oldData.get(1, 0);


				Matrix dp = DataShaper.toVelocityCoordinares(
					vOld, aOld,
					t,
					new Matrix(2, 1, new double[] {dx, dy})
				);

				HashMap<String, double[]> mkabParallel;
				HashMap<String, double[]> mkabPerp;

				if (stopped) {
					mkabParallel = this.mkabParallelStop;
					mkabPerp = this.mkabPerpStop;
				} else {
					mkabParallel = this.mkabParallelMove;
					mkabPerp = this.mkabPerpMove;
				}

				DataFiller.addMovedPoint(
					dp,
					mkabParallel.get(name),
					mkabPerp.get(name)
				);


				double[] paramsParallel  = DataShaper.mlNormalParams(mkabParallel.get(name));
				double[] paramsPerp = DataShaper.mlNormalParams(mkabPerp.get(name));
			}
		}
	}

	private void preparePriorIfNecessary(String name) {
		if(! mkabPerpMove.containsKey(name)) {
			mkabPerpMove.put(name, new double[] {
				mkabPerpPriorMove[0], mkabPerpPriorStop[1],
				mkabPerpPriorMove[2], mkabPerpPriorStop[3]
			});

			mkabParallelMove.put(name, new double[] {
				mkabParallelPriorMove[0], mkabParallelPriorMove[1],
				mkabParallelPriorMove[2], mkabParallelPriorMove[3]
			});
		}

		if(! mkabPerpStop.containsKey(name)) {
			mkabPerpStop.put(name, new double[] {
				mkabPerpPriorStop[0], mkabPerpPriorStop[1],
				mkabPerpPriorStop[2], mkabPerpPriorStop[3]
			});

			mkabParallelStop.put(name, new double[] {
				mkabParallelPriorStop[0], mkabParallelPriorStop[1],
				mkabParallelPriorStop[2], mkabParallelPriorStop[3]
			});
		}
	}



	public void maybeShoot(ScannedRobotEvent event, Matrix data) {
		if(event.getDistance() > tryShootDistance) {
			return;
		}

		double a = DataShaper.getBearingRadiansFromX(this, event);
		LinkedList<Matrix> observationsList = observationsMap.get(event.getName());

		if(DataShaper.getHistoryDT(data) > informationTimeLimit ||
                observationsList.size() < necessaryObservations) {
			forceClearToCheck();
			Matrix position = new Matrix(2, 1, new double[] {
				DataShaper.getHistoryX(data),
				DataShaper.getHistoryY(data)
			});
			checkAroundPoint(position, DataShaper.getHistoryHeading(data));
			return;
		}

		double energy = DataShaper.getEnergyFromDistance(event.getDistance());

		shootConsideringSpeed(event, data, energy);
	}



	public void shootConsideringSpeed(ScannedRobotEvent event, Matrix data, double energy) {
		lastTargetPosition = DataShaper.getAbsPosition(this, event);
		lastTargetEvent = event;

		Matrix targetPosition = DataShaper.getAbsPosition(this, event);
		double alpha = DataShaper.getHeadingRadiansFromX(event);

		double simplifiedV = DataShaper.simplifiedV(event.getVelocity());

		double distance = Math.hypot(
            DataShaper.getHistoryX(data) - getX(),
            DataShaper.getHistoryY(data) - getY()
        );

		HashMap<String, double[]> mkabParallel;
		HashMap<String, double[]> mkabPerp;

		boolean stopped = DataShaper.isStopMode(data);
		if (stopped) {
			mkabParallel = this.mkabParallelStop;
			mkabPerp = this.mkabPerpStop;
		} else {
			mkabParallel = this.mkabParallelMove;
			mkabPerp = this.mkabPerpMove;
		}

		double[] paramsParallel  = DataShaper.mlNormalParams(mkabParallel.get(event.getName()));
		double[] paramsPerp = DataShaper.mlNormalParams(mkabPerp.get(event.getName()));

		Matrix guessedPositionChange;
		long bulletFlyTime;
		do {
			Matrix targetInVCoordinates = new Matrix(2, 1, new double[] {
				DataShaper.getNormal(paramsParallel[0], paramsParallel[1]),
				DataShaper.getNormal(paramsPerp[0], paramsPerp[1])
			});

			bulletFlyTime = (long)Math.ceil(distance / DataShaper.getBulletSpeed(energy) + 2);

			guessedPositionChange = DataShaper.fromVelocityCoordinares(simplifiedV,
															alpha, bulletFlyTime, targetInVCoordinates);

		} while(
			DataShaper.length(guessedPositionChange) >= bulletFlyTime * DataShaper.MAX_V);



		Matrix guessInAbsCoordinates = guessedPositionChange.add(targetPosition);

		System.out.println("targetPosition " + targetPosition.toStringFull());


		Matrix targetRelative = targetPosition.add(DataShaper.getMyPosition(this).scale(-1));

		lock = true;
        RobotActions.shootAt(this, guessInAbsCoordinates);
		lock = false;

		scan();

		checkAroundPoint(targetPosition, DataShaper.getHistoryHeading(data));

	}

	private boolean isDuel() {
		if(lastTargetEvent == null || lastHitEvent == null) {
			return false;
		}
		return lastHitEvent.getName().equals(lastTargetEvent.getName());
	}

	@Override
 	public void onHitRobot(HitRobotEvent event) {
		if(lock)
			return;

		insideOnHitRobot = true;
		stop();

		double a = DataShaper.getBearingRadiansFromX(this, event);

		lock = true;
		RobotActions.rotateGunRadAt(this, a);
		fire(3);
		lock = false;

		scan();
		insideOnHitRobot = false;
	}


	@Override
	public void onHitByBullet(HitByBulletEvent event) {
		HitByBulletEvent eventBeforeThis = lastHitEvent;
		lastHitEvent = event;

		if(lock)
			return;

		insideOnHitByBullet = true;


		if(lastTargetEvent == null || lastTargetEvent.getDistance() > importantTargetDistance) {
			double a = DataShaper.getBearingRadiansFromX(this, event);
			//checkAround(a, event.getName());
		}

		insideOnHitByBullet = false;
	}

	private void checkAround(double alpha, String name) {
		forceClearToCheck();

		Matrix myPosition = DataShaper.getMyPosition(this);

		double aShift = DataShaper.GUN_TURN_V / 4;
		Matrix check = myPosition.add(
			new Matrix(2, 1, new double[] {
				100 * Math.cos(alpha),
				100 * Math.sin(alpha),
			})
		);

		Matrix check0 = myPosition.add(
			new Matrix(2, 1, new double[] {
				100 * Math.cos(alpha + aShift),
				100 * Math.sin(alpha + aShift),
			})
		);

		Matrix check1 = myPosition.add(
			new Matrix(2, 1, new double[] {
				100 * Math.cos(alpha - aShift),
				100 * Math.sin(alpha - aShift),
			})
		);

		addToCheck(check);
		addToCheck(check0);
		addToCheck(check1);
	}

	private void checkAroundPoint(Matrix position, double heading) {
		forceClearToCheck();

		Matrix vel = new Matrix(2, 1, new double[] {
			DataShaper.MAX_V * Math.cos(heading),
			DataShaper.MAX_V * Math.sin(heading)
		});

		Matrix check  = position;
		Matrix check0 = position.add(vel.inplaceScale(2));
		Matrix check1 = position.add(vel.inplaceScale(-1));
		Matrix check2 = position.add(vel.inplaceScale(4));
		Matrix check3 = position.add(vel.inplaceScale(-2));

		addToCheck(check);
		addToCheck(check0);
		addToCheck(check1);
		addToCheck(check2);
		addToCheck(check3);
	}

	private void runFromBullets(HitByBulletEvent event) {
		System.out.println("runFromBullets");
		forceClearToCheck();
		standSideToBearing(DataShaper.getBearingRadiansFromX(this, event));


		smartRandomAhead(80, 200);
	}

	private void standSideToBearing(double bearing) {
		double alpha =
			DataShaper.truncateRad(DataShaper.getHeadingRadiansFromX(this) - bearing);

		alpha = Math.atan(Math.tan(alpha));
		if(Math.abs(alpha) < Math.PI / 6) {
			RobotActions.turnRad(this, Math.signum(alpha) * (Math.PI / 2 - Math.abs(alpha)));
		}
	}


	private void smartRandomAhead(double mind, double maxd) {
		double alpha = DataShaper.getHeadingRadiansFromX(this);

		Matrix myPosition = DataShaper.getMyPosition(this);

		Matrix p0, p1, target;

		int index = 0;
		do {
			if(index > 10) {
				RobotActions.turnRad(this, Math.PI / 4);
				index = 0;
			}
			double d = DataShaper.random(mind, maxd);
			p0 = myPosition.add(
				new Matrix(2, 1, new double[] {
					d * Math.cos(alpha),
					d * Math.sin(alpha),
				})
			);
			p1 = myPosition.add(
				new Matrix(2, 1, new double[] {
					- d * Math.cos(alpha),
					- d * Math.sin(alpha),
				})
			);

			index++;
		} while(isInCorner(p0) && (isInCorner(p1)));

		if(isInCorner(p0)) {
			target = p1;
		} else if(isInCorner(p1)) {
			target = p0;
		} else {
			if(Math.random() > 0.5) {
				target = p0;
			} else {
				target = p1;
			}
		}
		RobotActions.moveAt(this, target);
	}


	private boolean isInCorner(Matrix p) {
		for(int i = 0; i < 4; i++) {
			if(isNearWall(i, p) && isNearWall(i+1, p)) {
				return true;
			}
		}
		return false;
	}
}
