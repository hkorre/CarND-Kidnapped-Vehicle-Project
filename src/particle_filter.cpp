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

#include "particle_filter.h"


#define NUM_PARTICLES  (100)
#define START_DIST     (1000)

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
  //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
  // Add random Gaussian noise to each particle.
  // NOTE: Consult particle_filter.h for more information about this method (and others in this file).

  num_particles = NUM_PARTICLES;

  std::default_random_engine gen;
  
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];	 
  
  std::normal_distribution<double> dist_x(x, std_x);
  std::normal_distribution<double> dist_y(y, std_y);
  std::normal_distribution<double> dist_theta(theta, std_theta);

  for (int i=0; i<num_particles; i++) {
    struct Particle new_particle = {
      i,                //id
      dist_x(gen),      //x
      dist_y(gen),      //y
      dist_theta(gen),  //theta
      1                 //weight
    };
    particles.push_back(new_particle);
  }

  is_initialized = true;

  std::cout << "ParticleFilter initialized." << std::endl;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // TODO: Add measurements to each particle and add random Gaussian noise.
  // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
  //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
  //  http://www.cplusplus.com/reference/random/default_random_engine/

  std::default_random_engine gen;
  
  double std_x = std_pos[0];
  double std_y = std_pos[1];
  double std_theta = std_pos[2];	 

  std::normal_distribution<double> dist_x(0, std_x);
  std::normal_distribution<double> dist_y(0, std_y);
  std::normal_distribution<double> dist_theta(0, std_theta);

  for (auto it=particles.begin(); it!=particles.end(); ++it) {
    if(yaw_rate==0) {
      it->x = it->x + velocity*delta_t*cos(it->theta) + dist_x(gen);
      it->y = it->y + velocity*delta_t*sin(it->theta) + dist_y(gen);
      //it->theta = it->theta
    } else {
      it->x += (velocity/yaw_rate)*(sin(it->theta + yaw_rate*delta_t) - sin(it->theta)) + dist_x(gen);
      it->y += (velocity/yaw_rate)*(cos(it->theta) - cos(it->theta + yaw_rate*delta_t)) + dist_x(gen);
      it->theta += yaw_rate*delta_t;
    }
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
  // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
  //   observed measurement to this particular landmark.
  // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
  //   implement this method and use it as a helper during the updateWeights phase.

  for (auto it_obs=observations.begin(); it_obs!=observations.end(); ++it_obs) {
    double min_dist_sq = START_DIST;
    double min_id = 0;

    for (auto it_pred=predicted.begin(); it_pred!=predicted.end(); ++it_pred) {
      double dist_sq = dist(it_obs->x, it_obs->y, it_pred->x, it_pred->y);
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        min_id = it_pred->id;
      }
    }

    it_obs->id = min_id;
  }
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
  //   3.33. 
  //   http://planning.cs.uiuc.edu/node99.html

  // 1. Make list of all landmarks within sensor range of particle, call this `predicted_lm`


  // 2. Convert all observations from local to global frame, call this `transformed_obs`


  // 3. Perform `dataAssociation`. This will put the index of the `predicted_lm` nearest 
  //    to each `transformed_obs` in the `id` field of the `transformed_obs` element.

  // 4. Loop through all the `transformed_obs`. Use the saved index in the `id` to find 
  //    the associated landmark and compute the gaussian. 

  // 5. Multiply all the gaussian values together to get total probability of particle (the weight).



  //TODO: What to do with sensor_range?

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

  double range_sq = sensor_range*sensor_range;



  for (auto it_par=particles.begin(); it_par!=particles.end(); ++it_par) {

    // find landmarks in range of particle that sensor can reach...
    std::vector<LandmarkObs> predicted;
    for (auto it_map=map_landmarks.landmark_list.begin(); it_map!=map_landmarks.landmark_list.end(); ++it_map) {
      double dist_sq = dist(it_map->x_f, it_map->y_f, it_par->x, it_par->y);
      if (dist_sq < range_sq) {
        struct LandmarkObs new_predicted = {
          it_map->id_i,  //id
          it_map->x_f,   //x
          it_map->y_f    //y
        };
        predicted.push_back(new_predicted);
      }
    }

    // transform each observations from vehicle frame to the map frame...
    for (auto it_obs=observations.begin(); it_obs!=observations.end(); ++it_obs) {
      double x = it_obs->x;
      double y = it_obs->y;
      double theta = it_par->theta;

      //x_obsInMap = x_obsInCar*cos(theta_carInMap) - y_obsInCar*sin(theta_carInMap) + x_carInmap
      it_obs->x = it_obs->x*cos(it_par->theta) - it_obs->y*sin(it_par->theta) + it_par->x;

      //y_obsInMap = x_obsInCar*sin(theta_carInMap) + y_obsInCar*cos(theta_carInMap) + y_carInmap
      it_obs->y = it_obs->x*sin(it_par->theta)  + it_obs->y*cos(it_par->theta) + it_par->y;
    }

    // associate each transformed observation with a land mark identifier
    //   use dataAssociation()
    dataAssociation(predicted, observations);

    // calculate the particle's final weight

  }

  
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
