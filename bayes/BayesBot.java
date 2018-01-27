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

	private Matrix lastTargetPosition;
	private ScannedRobotEvent lastObservation, lastTargetEvent;
	private HitByBulletEvent lastHitEvent;

	private boolean insideOnScannedRobot, insideOnHitByBullet, insideOnHitRobot;
	private ArrayList<Matrix> toCheck = new ArrayList<Matrix>();

	private double wallPadding;
	private long historyTimeLimit = 6000;
	private double battleFieldWidth, battleFieldHeight;


	private void clearToCheck() {
		this.toCheck = new ArrayList<Matrix>();
	}

	private void addToCheck(Matrix m) {
		toCheck.add(m);
	}

	public boolean insideEvent() {
		return insideOnScannedRobot || insideOnHitByBullet || insideOnHitRobot;
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


	private void check() {
		if(insideEvent()) { //only check if not working on events
			return;
		}

		//to prevent recursion stack owerflow; without update of toCheck the next call to check will return
		if(toCheck == null || toCheck.size() == 0) {
			return;
		}
		ArrayList<Matrix> toCheckTemp = toCheck;
		toCheck = new ArrayList<Matrix>();

		for(int i = 0; i < toCheckTemp.size(); i++) {
			Matrix check = toCheckTemp.get(i);

			long lastObsTime0 = lastObservation.getTime();
			RobotActions.rotateGunAtPoint(this, check.get(0, 0), check.get(1, 0));
			long lastObsTime1 = lastObservation.getTime();
			if(lastObsTime0 != lastObsTime1) { //we found new target, no nee to scan more
				check();  //there may be new  pointds to check from last ScannedRobotEvent
				break;
			}
		}
	}

	private void initialise() {
		battleFieldWidth = getBattleFieldWidth();
		battleFieldHeight = getBattleFieldHeight();
	}

	public void run() {
		initialise();
		while(true) {
			nextAction();
		}
	}



	private void nextAction() {
		if(!isInCenter()) {
			goToCenter();
		} else {
			lookAround();
		}
	}


	private void goToCenter() {
		RobotActions.moveAt(this, getCenter());
	}

	private boolean isInCenter() {
		return isInCenter(DataShaper.getMyPosition(this));
	}

	private boolean isInCenter(Matrix p) {
		Matrix center = getCenter();
		double dist = DataShaper.hypot(center.add(p.scale(-1)));
		return dist < 300;
	}

	private Matrix getCenter() {
		return new Matrix(2, 1, new double[] {battleFieldWidth / 2, battleFieldHeight / 2});
	}

	private boolean patrollingUp;

	private void lookAround() {
		for(int i = 0; i < 4; i++) {
			RobotActions.rotateGunRad(this, Math.PI / 2);
		}
	}



	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
        insideOnScannedRobot = true;
		ScannedRobotEvent eventBefore = lastObservation;
        lastObservation = event;

		Matrix data = HistoryFunctions.writeMatrixOfObservation(this, event, observationsMap);
        aimer.updateParameters(observationsMap.get(event.getName()), event.getName());


		if(lock) {
            insideOnScannedRobot = false;
            return;
        }

 		Matrix position = HistoryFunctions.getHistoryXY(data);
		double a = EventDataShaper.getBearingRadiansFromX(this, event);
		checkAroundPoint(position, HistoryFunctions.getHistoryHeading(data));


		if(RobotActions.standFlankToBearing(this, a)) {
			insideOnScannedRobot = false;
			return;
		}

		double energyDiff = eventBefore.getEnergy() - event.getEnergy();
		if(0 < energyDiff && energyDiff <= 3) {
			runFromBullets(EventDataShaper.getBearingRadiansFromX(this, event));
			insideOnScannedRobot = false;
			return;
		}


        //at least check arount where we saw robot if last action was moving
        //ignoring the fact that position is not exactly correct
        if(lastWasStopped) {
          	maybeShoot(event, data);
		}


		HistoryFunctions.clearFromOldObservations(this, observationsMap, historyTimeLimit);
		insideOnScannedRobot = false;
	}

	private void maybeShoot(ScannedRobotEvent event, Matrix data) {
		double distance = event.getDistance();
		double energy = DataShaper.getEnergyFromDistance(this, distance);

		double a = EventDataShaper.getBearingRadiansFromX(this, event);
		LinkedList<Matrix> observationsList = observationsMap.get(event.getName());

		if(! aimer.wantToShoot(data, observationsList.size(), energy, distance)) {
			return;
		}
		shootConsideringSpeed(event, data, energy);
	}


	public void shootConsideringSpeed(ScannedRobotEvent event, Matrix data, double energy) {

        Matrix targetPosition = HistoryFunctions.getHistoryXY(data);
		lastTargetPosition = targetPosition.copy();
        lastTargetEvent = event;

        double distance = DataShaper.hypot(
            HistoryFunctions.getHistoryXY(data).add(
				DataShaper.getMyPosition(this).inplaceScale(-1)
			)
        );

        long bulletFlyTime =
            (long)Math.ceil(
                distance / DataShaper.getBulletSpeed(energy) + 2);

		Matrix guessedPositionChange, guessInAbsCoordinates;
		String name = event.getName();
		int i = 0;
		boolean dontShoot = false;
		do {

			guessedPositionChange = aimer.suggestEnemyPoitionChange(data, name , bulletFlyTime);
			guessInAbsCoordinates = guessedPositionChange.add(lastTargetPosition);
			if(i > 1000) {
				dontShoot = true;
				break;
			}
			i++;
		} while(DataShaper.hypot(guessedPositionChange) >= DataShaper.MAX_V * bulletFlyTime
					|| !isOnField(guessInAbsCoordinates)); // our guess shouldnt be impossibly far avay and out of the battlefield

		if(! dontShoot) {
			lock = true;
        	RobotActions.shootAt(this, guessInAbsCoordinates, energy);
			lock = false;
		} else {
			System.out.println("Decided not to shoot, cannot sample a proper guess");
		}

		scan(); // maybe target is still in the radar ray
		checkAroundPoint(targetPosition, HistoryFunctions.getHistoryHeading(data));

	}


	@Override
 	public void onHitRobot(HitRobotEvent event) {
		if(lock)
			return;

		insideOnHitRobot = true;
		stop();

		double a = EventDataShaper.getBearingRadiansFromX(this, event);

		lock = true;
		RobotActions.rotateGunRadAt(this, a);
		fire(3);
		lock = false;

		scan();
		insideOnHitRobot = false;
	}

	private void checkAroundAlpha(double alpha) {
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
		Matrix check0 = position.add(vel.inplaceScale(4));
		Matrix check1 = position.add(vel.inplaceScale(-2));
		Matrix check2 = position.add(vel.inplaceScale(20));
		Matrix check3 = position.add(vel.inplaceScale(-10));

		addToCheck(check);
		addToCheck(check0);
		addToCheck(check1);
		addToCheck(check2);
		addToCheck(check3);
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

		if(!isInCenter()) {
			insideOnHitByBullet = false;
			return;
		}

		double a = EventDataShaper.getBearingRadiansFromX(this, event);
		checkAroundAlpha(a);

		insideOnHitByBullet = false;
	}

	private void runFromBullets(double hitBearing) {
		RobotActions.standFlankToBearing(this, hitBearing);
		smartRandomAhead(100, 120);
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
		} while(!isOnField(p0) && !isOnField(p1));

		if(!isOnField(p0)) {
			target = p1;
		} else if(!isOnField(p1)) {
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

	private boolean isOnField(Matrix p) {
		return 0 < p.get(0, 0) && p.get(0, 0) < battleFieldWidth &&
				0 < p.get(1, 0) && p.get(1, 0) < battleFieldHeight;
	}
}
