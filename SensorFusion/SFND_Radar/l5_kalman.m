
state = [x;vx;y;vy];

% Instantiate a kalman filter
filter = 
    trackingKF('MotionModel', 
                '2D Constant Velocity', 
                'State', 
                state, 
                'MeasurementModel', 
                measurementModel, 
                'StateCovariance', 
                stateCovrariance, 
                'MeasurementNoise', 
                measurementNoise);