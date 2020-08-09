classdef Autolycus_C < rl.env.MATLABEnvironment
    %AUTOLYCUS: Template for defining custom environment in MATLAB.    
    
    %% Properties (set properties' attributes accordingly)
    properties
        % Specify and initialize environment's necessary properties    
        % Acceleration due to gravity in m/s^2
        
        %%%%NEEDS CHANGING%%%
        % Max Force the input can apply
        MaxForce = 1
               
        % Sample time
        Ts = 0.02
        
        % Bounds At which To Fail Episode
        X_Threshold = 100
        Y_Threshold = 10
        Z_Threshold = 15
        
        % Distance at which to fail the episode
        %DisplacementThreshold = 2.4
        
        % Reward each time step the cart-pole is balanced
        %RewardForNotFalling = 1
        
        % Penalty when the cart-pole fails to balance
        %PenaltyForFalling = -10 
    end
    
    properties
        % Initialize system state [u,v,w,p,q,r,x,y,z,phi,theta,psi]'
        State = zeros(12,1)
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = Autolycus_C()
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([16 1]);
            ObservationInfo.Name = 'AUV States';
            ObservationInfo.Description = 'u,v,w,p,q,r,x,y,z,phi,theta,psi';
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([4 1], ...
                           'LowerLimit', [-1; -1; -1; -1], ...
                           'UpperLimit', [1; 1; 1; 1]);
            ActionInfo.Description = 'INDEX: T_Stbd,T_Port,T_aft and T_sten';
            ActionInfo.Name = 'AUV Actions';
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            
            % Initialize property values and pre-compute necessary values
            %updateActionInfo(this);
        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
            
            % Get action
            Force = getForce(this,Action);            
            
            % Unpack state vector
            u =this.State(1);
            v= this.State(2);
            w =this.State(3);
            p =this.State(4);
            q =this.State(5);
            r =this.State(6);
            x =this.State(7);
            y =this.State(8);
            z =this.State(9);
            phi = this.State(10);
            theta = this.State(11);
            psi = this.State(12);
            
            
            Tstbd = Action(1);
            Taft = Action(2);
            Tport = Action(3);
            Tfore = Action(4);
            
            
            % Solve Dynamics Via ODE45
            X0 = double([u,v,w,p,q,r,x,y,z,phi,theta,psi,Tstbd, Taft, Tport, Tfore]);
            dt = 0.025;
            tspan = [0:dt:0.1];
            
            [t,X] = ode45('Dynamics',tspan,X0);
            Observation = X(end,:)';
            
            
            
            % Update system states
            this.State = Observation;
            
            %disp([Observation(7) Observation(8) Observation(9) Action'])
            
                       
            %Add Noise
            %Observation(7) = Observation(7)+ normrnd(0,0.01);
            %Observation(8) = Observation(8)+ normrnd(0,0.01);
            %Observation(9) = Observation(9)+ normrnd(0,0.01);
            
            
            % Check terminal condition
            X = Observation(7);
            Y = Observation(8);
            Z = Observation(9);
            
            %disp([X Y Z Action])

            IsDone = abs(X) > this.X_Threshold || abs(Y) > this.Y_Threshold|| abs(Z) > this.Z_Threshold;
            if IsDone ==1
                disp('AUV out of Bounds')
            end
            %this.IsDone = IsDone;
            
            % Get reward
            Reward = getReward(this);
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            %notifyEnvUpdated(this);
        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
            
            InitialObservation = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
            this.State = InitialObservation;
            
            % (optional) use notifyEnvUpdated to signal that the 
            % environment has been updated (e.g. to update visualization)
            notifyEnvUpdated(this);
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Helper methods to create the environment
        % Discrete force 1 or 2
        function force = getForce(~,action)
            force = action;           
        end
        % Reward function
        function Reward = getReward(this)
            Target_Pos = [10 0 10];
            if ~this.IsDone
                Reward =  sqrt(Target_Pos(1)^2 + Target_Pos(2)^2 + Target_Pos(3)^2  ) - ...
                    sqrt((this.State(7)-Target_Pos(1))^2+(this.State(8)-Target_Pos(2))^2+(this.State(9)-Target_Pos(3))^2);
                if this.State(8) > 0.25 && this.State(8) < 0.25
                    Reward = Reward - (100*this.State(8))^2;
                end
                else
                Reward = - sqrt((this.State(7)-Target_Pos(1))^2+(this.State(8)-Target_Pos(2))^2+(this.State(9)-Target_Pos(3))^2)^2 ;
            end
        end
        
        % (optional) Visualization method
        function plot(this)
            % Initiate the visualization
            
            % Update the visualization
            envUpdatedCallback(this)
        end
        
    end
    
    methods (Access = protected)
        % (optional) update visualization everytime the environment is updated 
        % (notifyEnvUpdated is called)
        function envUpdatedCallback(this)
        end
    end
end
