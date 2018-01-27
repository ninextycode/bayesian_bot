# bayesian_bot
Robocode Robot subclass which implements Bayesian modelling of enemies position for aiming.

The change in enemy's position after time t is modelled as a 2D uncorrelated Normal variable.
The variance and the mean of this 2D Normal are unknown and are estimated using the Bayes theorem
Read more at https://en.wikipedia.org/wiki/Conjugate_prior

Sampled variable represents a point in a 2d plane with x-axis being the vector of enemy's velocity,
scaled by the time necessary for a bullet to reach the enemy.
