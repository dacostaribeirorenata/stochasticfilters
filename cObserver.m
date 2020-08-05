classdef cObserver < handle
    properties
        initialized;  % Flag describing whether observer is initialized
        u;            % Input at time t
    end
    
    methods (Abstract)
        
        % Given initial state and inputs, initialize observer
        initialize(obj,x,u)       
        
        % Given the input 'u' at the current time-step, predict the x and P
        prediction (obj,u)     
 
        % Given the measurement 'z' at the current time-step, correct the x and P
        measurementUpdate (obj,z)        
        
        % Get state current estimate
        getStateEstimate(obj)
    end    
end
