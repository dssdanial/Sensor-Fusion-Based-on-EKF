# Sensor-Fusion-Based-on-EKF
## 1-Sensor Fusion 
In this Project we have derived and implemented a multisensory data fusion algorithm to estimate the pose of the robot through multiple onboard sensors (wheel encoders, gyroscope, accelerometer, magnetometer, and electronic compass). To correct the orientation of the mobile robot electronic compass is used. The confidence of the electronic compass measurements is judged by the tri-axial magnetometer. The methodology of the data fusion framework is as follows.
First of all, the entire error model for wheel encoders is derived which is used to estimate the errors in linear and angular velocity of the mobile robot. The error model for gyroscope and accelerometer are also derived with error perturbation method to estimate the errors in linear and angular velocities. Later the indirect Kalman filter is applied on the error model to estimates the errors in the pose and parameters. The estimated state vector is feedback to compensate the parameters and pose calculations. The final mobile robot pose calculation is outside the Kalman filter recursive estimation loop therefore comparatively low iteration frequency of the filter is required, because the dynamics of the robot is outside the filtering loop. 
The error state model of the mobile robot to estimate robot’s pose compose of motion and measurement model. The measurement and observation model are derived using perturbation in encoder, gyroscope and accelerometer measurements and parameters. Each sensors error model is briefly described as follows: 


## 1-1- Encoder Velocity Error Model: 
The real and ideal velocity equations for robot are as follows 

![image](https://user-images.githubusercontent.com/32397445/153770147-c9d3f2a4-9fc2-4a5a-8074-80ceec1ce71c.png)

The velocity scale factor errors ( SL , SR) and wheel distance error ( SD ) are assumed to very slow time invariant, therefore

![image](https://user-images.githubusercontent.com/32397445/153770163-18848c35-f445-4ebc-8e8a-bfd0c98ce8ef.png)

## 1-2- Accelerometer Velocity Error Model:
The accelerometer error model for real and ideal linear velocities along with the scale ( Sax , Say ) and bias factors ( Bax , Bay ) are as follows:

![image](https://user-images.githubusercontent.com/32397445/153770189-2fcdf13e-9adb-44ab-9a91-bd0714d3dbb3.png)

## 1-3- Gyroscope Error Model: 
The gyroscope error model for real and ideal angular velocities along with the scale ( Sgz ) and bias factor ( Bgz ) are as follows:
![image](https://user-images.githubusercontent.com/32397445/153770219-5d575e2f-aefd-43ff-a70d-be99f9c98e94.png)

## 1-4- Compass Angle Error Model: 
The compass error model for actual and ideal azimuth angle along with the bias factor (Bc) are as follows:

![image](https://user-images.githubusercontent.com/32397445/153770246-31986113-bf2b-4df7-a0af-a663c1d0f923.png)

The final measurement and observation model equations used for data fusion in state space model form are as follows:

![image](https://user-images.githubusercontent.com/32397445/153770266-4cf6f640-1393-4fd3-b9bd-cda0fbc795e5.png)

where

![image](https://user-images.githubusercontent.com/32397445/153770278-64c7bb59-88c7-4140-838e-50eee8a1678c.png)
![image](https://user-images.githubusercontent.com/32397445/153770279-86dc965f-c912-4363-bf1f-31aaaecd8a20.png)

By defining the all parameters, we can extract the state space equations, which are needed for Kalman filtering algorithm.
Finally, by obtaining our sensor error models, we can use Kalman Filter to estimate them and correct them in next step. The block diagram of sensor fusion based on Kalman Filter estimation is illustrated as follows.


## 2-Kalman Filter algorithm
The Kalman filter is a recursive estimator. This means that only the estimated state from the previous time step and the current measurement are needed to compute the estimate for the current state. In contrast to batch estimation techniques, no history of observations and/or estimates is required. In what follows, the notation X ̂_(n|m) represents the estimate of X at time n given observations up to and including at time m ≤ n.
The state of the filter is represented by two variables:
X ̂_(k|k-1) is the a posteriori state estimate at time k given observations up to and including at time k;
P_(k|k) is the a posteriori error covariance matrix (a measure of the estimated accuracy of the state estimate).
The Kalman filter can be written as a single equation, however it is most often conceptualized as two distinct phases: "Predict" and "Update". 
The predict phase uses the state estimate from the previous time step to produce an estimate of the state at the current timestep. This predicted state estimate is also known as the a priori state estimate because, although it is an estimate of the state at the current time step, it does not include observation information from the current timestep. In the update phase, the current a priori prediction is combined with current observation information to refine the state estimate. This improved estimate is termed the a posteriori state estimate.
Typically, the two phases alternate, with the prediction advancing the state until the next scheduled observation, and the update incorporating the observation. However, this is not necessary; if an observation is unavailable for some reason, the update may be skipped and multiple prediction steps performed. Likewise, if multiple independent observations are available at the same time, multiple update steps may be performed (typically with different observation matrices Hk).

## Step 1: Prediction

![image](https://user-images.githubusercontent.com/32397445/153770387-29bb45bf-f82b-451f-978b-c7e51e868248.png)

## Step 2: Update
![image](https://user-images.githubusercontent.com/32397445/153770405-cf634b6a-0eca-4120-b7a5-3808388854a6.png)

The formula for the updated (a posteriori) estimate covariance above is valid for the optimal Kk gain that minimizes the residual error, in which form it is most widely used in applications.
Since in this reference initial parameters are not defined accurately, we have set them by experience, which are implemented in Matlab software.


## Results
we assume a kind of circle path for robot. In this case, we prepared all sensor data as offline data involved Gyroscope, Accelerometer, Magnetometer and Encoder data. In each iteration, we read the loaded data involve sensors data and desired velocity, position and attitude. For instance, Gyroscope sensor measured the angular velocities of in motion vehicle. Similarly, Accelerometer sensor measured accelerations in three dimensions such X, Y and Z axis.  It is notable that in real world implementation, we need to transform IMU or other sensors data from body frame to inertial frame, which is a unique frame for mathematic calculation of robot kinematics.
Furthermore, in this simulation we got path-planning data from MATLAB such as Position, Velocity and Attitudes, which means that in this method we have known the desired path and its specifications that make us needless to path planning calculations. These data can be prepared in real world by a GPS/Camera sensor or integrated of them. 

![image](https://user-images.githubusercontent.com/32397445/153770480-bc71bef1-6935-437f-8192-0370835f850b.png)

![image](https://user-images.githubusercontent.com/32397445/153770502-24a35471-69ba-402a-a1db-03eeb6678d79.png)


Figures show the XY-trajectory tracking of our mobile robot, which uses multi-sensors fusion that are implemented based on Kalman filter. 

