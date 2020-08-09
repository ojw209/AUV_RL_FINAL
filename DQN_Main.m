%Load In Autolcus Enviroment
env = Autolycus_D;
validateEnvironment(env)


%DQN Agent Options:
%% Deep Neural Network Options:
% create a critic network to be used as underlying approximator

statePath = [
    imageInputLayer([16 1 1], 'Normalization', 'none', 'Name', 'state')
    fullyConnectedLayer(256, 'Name', 'CriticStateFC1')
    fullyConnectedLayer(256, 'Name', 'CriticStateFC2')
    reluLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(128, 'Name', 'CriticStateFC3')];
actionPath = [
    imageInputLayer([1 1 1], 'Normalization', 'none', 'Name', 'action')
    fullyConnectedLayer(128, 'Name', 'CriticActionFC1')];
commonPath = [
    additionLayer(2,'Name', 'add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1, 'Name', 'output')];
criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = addLayers(criticNetwork, commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC3','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');


%LEARN RATE CHANGED FROM 0.001 to 0.01
criticOptions = rlRepresentationOptions('LearnRate',0.001,'GradientThreshold',1);

obsInfo = getObservationInfo(env);
actInfo = getActionInfo(env);
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'state'},'Action',{'action'},criticOptions);

%Need to try smaller buffer.
%https://www.reddit.com/r/MachineLearning/comments/52r4wu/weird_catastrophic_forgetting_in_cartpole/
agentOptions = rlDQNAgentOptions(...
    'TargetUpdateMethod',"Smoothing", ...
    'TargetSmoothFactor',1e-3, ...
    'TargetUpdateFrequency',1, ...
    'ExperienceBufferLength',100000,...
    'UseDoubleDQN',true,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',128);

%% Epsilon Greedy Settings
agentOptions.EpsilonGreedyExploration.Epsilon = 0.95;
agentOptions.EpsilonGreedyExploration.EpsilonMin = 0.1;
agentOptions.EpsilonGreedyExploration.EpsilonDecay = 0.01;


agent = rlDQNAgent(critic,agentOptions);

%pool = parpool(4)

%% Training Options
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',1000,...
    'MaxStepsPerEpisode',1500,...
    'ScoreAveragingWindowLength',50,...
    'Verbose',false,...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',16000,...
    'SaveAgentCriteria','EpisodeReward',...
    'SaveAgentValue',10000) %,...
%'UseParallel' ,true) ;
%TrainingOptions.ParallelizationOptions.Mode = "async";


%Running Training
trainingStats = train(agent,env,trainingOptions);


