# Overview of classes

BayesBot - main class, contains the logic for event listeners, scanning and bullet evading
Calls functions from RobotActions and Aimer class to perform its actions

Aimer - class which updates parameters of Normal distributions based on observations,
and to sample a possible enemy's move

BayesianNormal - implementation of a model for a normal variable with unknown mean and covariance

CoordinamesTransform - functions to transform form standard coordianrtes to otrthoigonal coordinartes with
X axis being (velocity * expected_bullet_fly_time).

DataShaper - high-level functions returning information about BayesBot, for example, get an angle to a point.

EventDataShaper - similar to DataShaper, but functions return information related to events,
for example position of an enemy from ScannedRobotEvent

History functions - information about old ScannedRobotEvents is stored as a list of Matrices, and this class
manages reading/writing to a Matrix representing one ScannedRobotEvent.

Matrix - class for a matrix (mathematics)

Mapper, Reducer - interfaces to implement custom map/redice operations on matrices

RobotActions - class with functions implementing high-level actions, like look at/move to a certain point
