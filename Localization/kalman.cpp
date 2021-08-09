#include <iostream>
#include <math.h>
#include <tuple>
#include "Core"
#include "LU"

using namespace std;
using namespace Eigen;

tuple<double, double> measurement_update(double prior_mean, double prior_var, double measurement_mean, double measurement_var)
{
    double new_mean, new_var;
    new_mean = (measurement_var*prior_mean + prior_var*measurement_mean) / (prior_var+measurement_var);
    new_var =  1 / (1/prior_var + 1/measurement_var);
    return make_tuple(new_mean, new_var);
}

tuple<double, double> state_prediction(double prior_mean, double prior_var, double motion_mean, double motion_var)
{
    double new_mean, new_var;
    new_mean = prior_mean+motion_mean;
    new_var =  prior_var+motion_var;
    return make_tuple(new_mean, new_var);
}

tuple<MatrixXf, MatrixXf> kalman_filter_with_position_measurement(MatrixXf posterior_state, MatrixXf state_covariance_prediction, MatrixXf process_noise_uncertainty, MatrixXf state_transition_function, MatrixXf measurement_function, MatrixXf measurement_noise_uncertainty, MatrixXf identity_matrix)
{
    for (int n = 0; n < sizeof(measurements) / sizeof(measurements[0]); n++) {
        // Measurement Update
        MatrixXf state_of_observation(1, 1);
        state_of_observation << measurements[n];
        
        MatrixXf measurement_residual(1, 1);
        measurement_residual << state_of_observation-(measurement_function*posterior_state);
        
        MatrixXf S(1, 1);
        S << measurement_function*state_covariance_prediction*H.transpose() + measurement_noise_uncertainty;
        
        // Calculate Kalman Gain
        MatrixXf kalman_gain(2, 1);
        kalman_gain << state_covariance_prediction * measurement_function.transpose() * S.inverse();
        
        // Calculate Posterior and Covariance
        posterior_state << posterior_state+(kalman_gain*measurement_residual);
        state_covariance_prediction << (identity_matrix-(kalman_gain*measurement_function))*state_covariance_prediction;
        
        // State Prediction
        posterior_state << (state_transition_function*posterior_state) + process_noise_uncertainty;
        state_covariance_prediction << state_transition_function*state_covariance_prediction*state_transition_function.transpose();
    }

    return make_tuple(posterior_state, state_covariance_prediction);
}

int main()
{
    double new_mean, new_var;
    // test measurement update
    tie(new_mean, new_var) = measurement_update(10, 8, 13, 2);
    std::cout << "Measurement Update: " << (make_tuple(new_mean, new_var)==make_tuple(12.4, 1.6)) << "\n";

    // test state prediction
    tie(new_mean, new_var) = state_prediction(10, 4, 12, 4);
    std::cout << "State Prediction: " << (make_tuple(new_mean, new_var)==make_tuple(22, 8)) << "\n";

    // test kalman filter
    //Measurements and measurement variance
    double measurements[5] = { 5, 6, 7, 9, 10 };
    double measurement_sig = 4;
    
    //Motions and motion variance
    double motion[5] = { 1, 1, 2, 1, 1 };
    double motion_sig = 2;
    
    //Initial state
    double mu = 0;
    double sig = 1000;
    
    // Loop through all the measurments
    for (int t=0; t<sizeof(measurements)/sizeof(measurements[0]); t++){
        // Apply a measurment update
        tie(mu, sig) = measurement_update(mu, sig, measurements[t], measurement_sig);
        // Apply a state prediction
        tie(mu, sig) = state_prediction(mu, sig, motion[t], motion_sig);
    }
    std::cout << "kalman filter Prediction: " << (make_tuple(round(mu), round(sig))==make_tuple(11, 4)) << "\n";

    // test multi-dimensional kalman filter
    float measurements[3] = { 1, 2, 3 };
    MatrixXf x(2, 1);// Initial state (location and velocity) 
    x << 0,
    	 0; 
    MatrixXf P(2, 2);//Initial Uncertainty
    P << 100, 0, 
    	 0, 100; 
    MatrixXf u(2, 1);// External Motion
    u << 0,
    	 0; 
    MatrixXf F(2, 2);//Next State Function
    F << 1, 1,
    	 0, 1; 
    MatrixXf H(1, 2);//Measurement Function
    H << 1,
    	 0; 
    MatrixXf R(1, 1); //Measurement Uncertainty
    R << 1;
    MatrixXf I(2, 2);// Identity Matrix
    I << 1, 0,
    	 0, 1; 

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
    MatrixXf x_result(2, 1);
    x_result << 3.99664,
                0.999984;
    MatrixXf p_result(2, 2) ;
    p_result << 2.31904, 0.99176,
                0.99176, 0.495058;
    std::cout << "multidimensional kalman filter Prediction: " << (make_tuple(round(x), round(P))==make_tuple(x_result, p_result)) << "\n";
    return 0;
}