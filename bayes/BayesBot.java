package bayes;

import robocode.*;
import bayes.additional.*;
import java.util.LinkedList;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.HashMap;
import java.util.Map;



public class BayesBot extends Robot	{
	private HashMap<String, LinkedList<Matrix>> observationsMap =
									new HashMap<String, LinkedList<Matrix>>();

    public static Aimer aimer = new Aimer();




	private ScannedRobotEvent lastObservation, lastTarget;
	private HitByBulletEvent lastHitEvent;

	private boolean insideOnScannedRobot, insideOnHitByBullet, insideOnHitRobot;
	private ArrayList<Matrix> toCheck = new ArrayList<Matrix>();

	private double wallPadding;

	private void clearToCheck() {
		toCheck = new ArrayList<Matrix>();
	}

	private void addToCheck(Matrix m) {
		toCheck.add(m);
	}

	public boolean insideEvent() {
		return insideOnScannedRobot || insideOnHitByBullet || insideOnHitRobot;
	}

	private void check() {
		if(insideEvent()) { //only check if not working on events
			return;
		}

		if(toCheck == null || toCheck.size() == 0) {
			return;
		}
		ArrayList<Matrix> toCheckTemp = toCheck; //to prevent recursion stack owerflow
		toCheck = new ArrayList<Matrix>();

		for(int i = 0; i < toCheckTemp.size(); i++) {
			Matrix check = toCheckTemp.get(i);
			RobotActions.rotateGunAtPoint(this, check.get(0, 0), check.get(1, 0));
		}
	}

    @Override
    public void fire(double energy) {
        super.fire(energy);
        return;
    }

    //if robot scans an evemy while moving, enemy's position canot be correctly calculated
    //this flag indicates if the last robot's move was rotation insteat of motion
    boolean lastWasStopped = true;

    boolean lock;
    //a flag to be raised during aiming/shooting,
    //to prevent new events interrupting shooting

	@Override
	public void turnLeft(double angleInDegrees) {
		check();
        if(DataShaper.almostZero(angleInDegrees)) {
            return;
        }
        lastWasStopped=true;
		super.turnLeft(angleInDegrees);
	}

	@Override
	public void turnGunLeft(double angleInDegrees) {
		check();
        if(DataShaper.almostZero(angleInDegrees)) {
            return;
        }
        lastWasStopped=true;
		super.turnGunLeft(angleInDegrees);
	}

	@Override
	public void turnRadarLeft(double angleInDegrees) {
		check();
        if(DataShaper.almostZero(angleInDegrees)) {
            return;
        }
        lastWasStopped=true;
		super.turnRadarLeft(angleInDegrees);
	}

	@Override
	public void ahead(double distance) {
		check();
        lastWasStopped=false;
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
		if(!isNearWall()) {
			goToClosestWall();
		} else {
			scan();
			//patrol();
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
        lastObservation = event;

		Matrix data = DataFiller.writeDataPoint(event, observationsMap);
        Aimer.updateParameters(observationsMap.get(event.getName()), data);

        if(lock) {
            insideOnScannedRobot = false;
            return;
        }

        //at least check arount where we saw robot if last action was moving
        //ignoring the fact that position is not exactly correct
        if(!lastWasStopped) {
           Matrix position = HistoryFunctions.getHistoryXY(data);
           checkAroundPoint(position, HistoryFunctions.getHistoryHeading(data));
        } else {
           maybeShoot(event, data);
        }

		DataFiller.clearFromOldObservations(this, observationsMap, timeLimit);
		insideOnScannedRobot = false;
	}


	public void maybeShoot(ScannedRobotEvent event, Matrix data) {
		if(event.getDistance() > tryShootDistance) {
			return;
		}

		double a = DataShaper.getBearingRadiansFromX(this, event);
		LinkedList<Matrix> observationsList = observationsMap.get(event.getName());

		if(HistoryFunctions.getHistoryDT(data) > informationTimeLimit ||
                observationsList.size() < necessaryObservations) {
			Matrix position = HistoryFunctions.getHistoryXY(data);
			checkAroundPoint(position, HistoryFunctions.getHistoryHeading(data));
			return;
		}

		double energy = DataShaper.getEnergyFromDistance(event.getDistance());

		shootConsideringSpeed(event, data, energy);
	}



	public void shootConsideringSpeed(ScannedRobotEvent event, Matrix data, double energy) {

        lastTargetPosition = HistoryFunctions.getHistoryXY(data);
        lastTargetEvent = event;


		Matrix guessedPositionChange;
        double distance = Math.hypot(
            HistoryFunctions.getHistoryX(data) - getX(),
            HistoryFunctions.getHistoryY(data) - getY()
        );
        long bulletFlyTime =
            (long)Math.ceil(
                distance / DataShaper.getBulletSpeed(energy) + 2);

		do {

		} while( //reject unrealistic positions
			DataShaper.length(guessedPositionChange)
                >= bulletFlyTime * DataShaper.MAX_V);



		Matrix guessInAbsCoordinates = guessedPositionChange.add(targetPosition);

		lock = true;
        RobotActions.shootAt(this, guessInAbsCoordinates);
		lock = false;

		scan(); // maybe target is still in the radar ray
		checkAroundPoint(targetPosition, HistoryFunctions.getHistoryHeading(data));

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
		insideOnHitByBullet = true;

		if(lock) {
            insideOnHitByBullet = false;
			return;
        }

		if(lastTargetEvent == null || lastTargetEvent.getDistance() > importantTargetDistance) {
			double a = DataShaper.getBearingRadiansFromX(this, event);
			//checkAround(a, event.getName());
		}

		insideOnHitByBullet = false;
	}

	private void checkAround(double alpha, String name) {
		clearToCheck();

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

	private void checkAroundPoint(Matrix position, double enemyHeadinh) {
		clearToCheck();

		Matrix vel = new Matrix(2, 1, new double[] {
			DataShaper.MAX_V * Math.cos(enemyHeadinh),
			DataShaper.MAX_V * Math.sin(enemyHeadinh)
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
		clearToCheck();
		standSideToBearing(DataShaper.getBearingRadiansFromX(this, event));


		smartRandomAhead(80, 200);
	}

	private void standFlankToBearing(double bearing) {
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
