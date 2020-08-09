%Simulate Agent in the Enviroment - (Agent must be loaded into MATLAB
%workspace)

%For DDPG Enviroment
env = Autolycus_D;

%For DQN Enviroment
%env = Autolycus_D;

simOpts = rlSimulationOptions;
simOpts.MaxSteps = 2000;

%Ouput Results.
results = sim(env,saved_agent,simOpts);
States_Ts = results.Observation.AUVStates;
