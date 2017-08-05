/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  cout << "initializing particles filter" << endl;

  if (!is_initialized) {

    // initialize number of particles
    num_particles = 10;

    cout << "number of particles set" << endl;

    // create normal distributions with means based on GPS coordinates
    default_random_engine gen;
    normal_distribution<double> dist_x(x, std[0]);    
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    // initialize particles by sampling the distributions
    for (int i = 0; i < num_particles; ++i) {

      double sample_x, sample_y, sample_theta;
      
      sample_x = dist_x(gen);
      sample_y = dist_y(gen);
      sample_theta = dist_theta(gen);

      std::vector<int> associations;
      std::vector<double> sense_x;
      std::vector<double> sense_y;
      
      Particle p = {
	i, // particle id
	sample_x,
	sample_y,
	sample_theta,
	1, // weight
	associations, // associations
        sense_x, // sense_x
	sense_y // sense_y
      };

      particles.push_back(p);
      weights.push_back(1);
    }

    is_initialized = true;

  }

  cout << "particle filter initalized" << endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  // distributions for random noise
  // add random noise
  default_random_engine gen;
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);


  for (int i = 0; i < num_particles; ++i) {

    // add measurements
    if (fabs(yaw_rate) > 0.001) {

      particles[i].x = 
	particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));

      particles[i].y = 
	particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - sin(particles[i].theta + yaw_rate * delta_t));

      particles[i].theta = particles[i].theta + yaw_rate * delta_t;
    } else {
      particles[i].x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      particles[i].y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      // theta does not change
    }

    // add random noise
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);

    cout << particles[i].x << endl;

  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  for (int i = 0; i < observations.size(); ++i) {

    double min_distance = 100.0; // MAX_DISTANCE
    int closest_landmark = -1;

    for (int j = 0; j < predicted.size(); ++j) {
      
      double dist_x = predicted[j].x - observations[i].x;
      double dist_y = predicted[j].y - observations[i].y;

      double distance = sqrt(pow(dist_x,2) + pow(dist_y,2));

      if (distance < min_distance) { closest_landmark = predicted[j].id; }

    } // end for (observations)

  } // end for (predicted)

  

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  std::vector<LandmarkObs> world_observations;

  for (int i = 0; i < particles.size(); ++i) {


    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;

    for (int j = 0; j < observations.size(); ++j) {

      // transform observations in vehicle coordinates into world coordinates    

      // decompose vehicle coordinates
      double x_x = observations[j].x * cos(particles[i].theta);
      double x_y = observations[j].x * sin(particles[i].theta);

      double y_x = observations[j].y * cos(M_PI/2 - particles[i].theta);
      double y_y = observations[j].y * sin(M_PI/2 - particles[i].theta);

      // add components to particle coordinates to get world coordinates
      double w_x = particles[i].x + x_x + y_x;
      double w_y = particles[i].y + x_y + y_y;

      // find the closest landmark to the observation
      double min_distance = 100.0; // MAX_DISTANCE
      int closest_landmark_id = -1;
      std::vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;

      for (int z = 0; z < landmarks.size(); ++z) {
	double distance = dist(landmarks[z].x_f,landmarks[z].y_f,w_x, w_y); 

	if (distance < min_distance && distance <= sensor_range) { 
	  closest_landmark_id = landmarks[z].id_i; 
	  min_distance = distance;
	}
      }

      // associate measurement to closest landmark
      associations.push_back(closest_landmark_id);
      sense_x.push_back(w_x);
      sense_y.push_back(w_y);

    } // end observations
    
    // store associations for all measurements
    particles[i] = this->SetAssociations(particles[i],associations, sense_x, sense_y);

    // compute multivariate probability per measurement and multiply all probabilities 
    // together to determine weight

    std::vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
    double ONE_OVER_2PI_STD = 1.0 / (2*M_PI * std_landmark[0] * std_landmark[1]);

    for (int j = 0; j < particles[i].associations.size(); ++j) {
      
      // compute multivariate normal distribution
      double mu_x = landmarks[particles[i].associations[j]].x_f;
      double mu_y = landmarks[particles[i].associations[j]].y_f;
      double x = particles[i].sense_x[j];
      double y = particles[i].sense_y[j];
      double std_x = std_landmark[0];
      double std_y = std_landmark[1];

      double exp_x = (x-mu_x)/std_x;
      double exp_y = (y-mu_y)/std_y;

      double prob = ONE_OVER_2PI_STD*exp(-0.5*(exp_x*exp_x + exp_y*exp_y));

      // multiply probabilities together to generate weight
      particles[i].weight *= prob;
      
    }

    // store weight in weights array (to be used in resample computation)
    weights[i] = particles[i].weight;
    
  } // end particles

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  // implement wheel-based resampling algorithm

  default_random_engine gen;
  discrete_distribution<int> dist(0,num_particles);
  int index = dist(gen);

  double wmax = *max_element(weights.begin(), weights.end());
  double beta = 0;

  for (int i = 0; i < weights.size(); ++i) {
    uniform_real_distribution<double> beta_dist(0,2*wmax);
    beta += beta_dist(gen);

    while (weights[index] < beta) {
      beta -= weights[index];
      index = (index + 1) % weights.size();
    }

    // replace current particle with sampled particle
    particles[i] = particles[index];
  }
  


}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
