classdef cEKF < cObserver
    properties 
        stateEqn            % Dynamic model - f(x,u)
        outputEqn           % Measurement model - f(x,u)
        xjacobian           % Jacobian matrix - df/fx
        hjacobian           % Jacobian matrix - dh/fx
        P                   % Covariance matrix 
        x                   % State vector
        z                   % Measurement vector
        Q                   % Process noise
        R                   % Measurement noise
        yk                  % Innovation
        dk                  % Residual
    end

    methods 

        function ekf = cEKF(stateEqn,outputEqn,xjacobian,hjacobian, Q, R)
            %  ExtendedKalmanFilter Constructor
            %  Construct an EKF given the state equation, measurement matrix,
            %  state Jacobian, and the proccess and sensors noises.
            ekf.stateEqn = stateEqn;
            ekf.outputEqn = outputEqn;
            ekf.xjacobian = xjacobian;
            ekf.hjacobian = hjacobian;
            ekf.Q = Q;
            ekf.R = R;
        end		

        function initialize(ekf,x0,u0, P0)
            ekf.x = x0;
            ekf.u = u0;  
            ekf.P = P0; 
        end

         function prediction(ekf, u) 
            % Prediction Step
            Fkx = ekf.xjacobian(ekf.x, ekf.u);               
            xkp1 = ekf.stateEqn(ekf.x, ekf.u);            
            Pkp1 = Fkx * ekf.P * Fkx' + ekf.Q;

            ekf.P = Pkp1;
            ekf.x = xkp1;                 

            % Update inputs
            ekf.u = u;
         end

         function measurementUpdate(ekf, z, memoryX, memoryP)
            xkp1 = memoryX;
            Pkp1 = memoryP;             
            Hk = ekf.hjacobian(xkp1, ekf.u);       

            ekf.yk = z - (Hk * xkp1);
            Sk = (Hk * Pkp1 * Hk') + ekf.R;			
            Kk = (Pkp1 * Hk') / Sk;
            xkp1 = xkp1 + (Kk * ekf.yk);
            Pkp1 = (eye(length(ekf.x)) - Kk * Hk) * Pkp1;
            
            ekf.dk = z - (Hk * xkp1);
            
            ekf.x = xkp1;
            ekf.P = Pkp1;
         end

        function stateEstimate = getStateEstimate(ekf)
            stateEstimate.mean = ekf.x;
            stateEstimate.covariance = ekf.P;
            stateEstimate.residual = ekf.dk;
            
        end
    end
end
