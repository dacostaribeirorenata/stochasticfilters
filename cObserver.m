classdef cObserver < handle
    properties
        initialized;  % Flag describing whether observer is initialized
        u;            % Input at time t
        pnoise;       % Process noise
        mnoise;       % Measurement noise
    end
    
    methods (Abstract)
        
        % Given initial state and inputs, initialize observer
        initialize(obj,x,u)
        
        % Perform an prediction step given new time, inputs, and outputs
        prediction (obj,u)       
        
        % Perform an measurement update step if 
        measurementUpdate (obj,z)        
        
        % Get state estimate
        getStateEstimate(obj)
    end
    
end