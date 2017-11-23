/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <sstream>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 100;
    normal_distribution<double> dist_x(x, std[0]), dist_y(y, std[1]), dist_theta(theta, std[2]);
    particles.clear();
    weights.clear();

    for (int i = 0; i < num_particles; ++i) {
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        particles.push_back(p);
    }

    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/

    normal_distribution<double> dist_x(0, std_pos[0]), dist_y(0, std_pos[1]), dist_theta(0, std_pos[2]);
    double velo_del = velocity * delta_t, velo_yaw = velocity / yaw_rate, yaw_del = yaw_rate * delta_t;

    for (Particle &p : particles) {

        if (fabs(yaw_rate) < 0.00001) {
            p.x += velo_del * cos(p.theta);
            p.y += velo_del * sin(p.theta);
        } else {
            p.x += velo_yaw * (sin(p.theta + yaw_del) - sin(p.theta));
            p.y += velo_yaw * (cos(p.theta) - cos(p.theta + yaw_del));
            p.theta += yaw_del;
        }

        p.x += dist_x(gen);
        p.y += dist_y(gen);
        p.theta += dist_theta(gen);
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.

    double min_distance, distance;
    int map_id;

    for (LandmarkObs &obs : observations) {
        min_distance = numeric_limits<double>::max();
        map_id = -1;

        for (LandmarkObs &pred_obs : predicted) {
            //calculate distance
            distance = dist(obs.x, obs.y, pred_obs.x, pred_obs.y);
            if (distance < min_distance) {
                min_distance = distance;
                map_id = pred_obs.id;
            }
        }

        obs.id = map_id;
    }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
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

    double x_map, y_map, exponent = 0.0,
            gauss_norm = (1 / (2 * M_PI * std_landmark[0] * std_landmark[1])),
            sig_x_two = pow(std_landmark[0], 2),
            sig_y_two = pow(std_landmark[1], 2);


    for (Particle &p : particles) {

        vector<LandmarkObs> predicted_marks, transformed_obs;
        p.associations.clear();
        p.sense_x.clear();
        p.sense_y.clear();

        // Transform vehicle coordinates to map coordinates
        for (LandmarkObs obs : observations) {
            x_map = p.x + (cos(p.theta) * obs.x) - (sin(p.theta) * obs.y);
            y_map = p.y + (sin(p.theta) * obs.x) + (cos(p.theta) * obs.y);
            transformed_obs.push_back(LandmarkObs{obs.id, x_map, y_map});
        }

        // Add predicted marks
        for (Map::single_landmark_s lm : map_landmarks.landmark_list) {
            if (fabs(lm.x_f - p.x) <= sensor_range && fabs(lm.y_f - p.y) <= sensor_range) {
                predicted_marks.push_back(LandmarkObs{lm.id_i, lm.x_f, lm.y_f});
            }
        }

        dataAssociation(predicted_marks, transformed_obs);

        //Re-initialize the weight of the particle, otherwise the error of x and y increases
        p.weight = 1.0;
        //Multivariate-Gaussian Probability
        for (LandmarkObs trans_obs : transformed_obs) {

            for (LandmarkObs pred_lm : predicted_marks) {
                if (pred_lm.id == trans_obs.id) {
                    exponent = (pow(pred_lm.x - trans_obs.x, 2) / (2 * sig_x_two)) +
                               (pow(pred_lm.y - trans_obs.y, 2) / (2 * sig_y_two));
                }
            }
            p.weight *= gauss_norm * exp(-exponent);

            // Set associations
            p.associations.push_back(trans_obs.id);
            p.sense_x.push_back(trans_obs.x);
            p.sense_y.push_back(trans_obs.y);
        }
    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    weights.clear();
    for (Particle p : particles) {
        weights.push_back(p.weight);
    }

    vector<Particle> new_particles;
    uniform_int_distribution<int> idx_distribution(0, particles.size() - 1);
    uniform_real_distribution<double> rand_distribution(0.0, 0.1);
    double beta = 0.0, mw = *max_element(weights.begin(), weights.end());
    int idx = idx_distribution(gen);

    // Resampling wheel
    for (int i = 0; i < particles.size(); ++i) {
        beta += rand_distribution(gen) * 2.0 * mw;
        while (weights[idx] < beta) {
            beta -= weights[idx];
            idx = (idx + 1) % particles.size();
        }
        new_particles.push_back(particles[idx]);
    }

    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
                                         const std::vector<double> &sense_x, const std::vector<double> &sense_y) {
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
    vector<int> v = best.associations;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best) {
    vector<double> v = best.sense_x;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best) {
    vector<double> v = best.sense_y;
    stringstream ss;
    copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length() - 1);  // get rid of the trailing space
    return s;
}
