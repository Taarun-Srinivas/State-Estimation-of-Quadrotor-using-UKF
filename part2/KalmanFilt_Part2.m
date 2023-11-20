clear; % Clear variables
datasetNum = 9; % CHANGE THIS VARIABLE TO CHANGE DATASET_NUM
[sampledData, sampledVicon, sampledTime] = init(datasetNum);
Z = sampledVicon(7:9,:);%all the measurements that you need for the update
% Set initial condition
uPrev = vertcat(sampledVicon(1:9,1),zeros(6,1)); % Copy the Vicon Initial state
covarPrev = eye(15); % Covariance constant
savedStates = zeros(15, length(sampledTime)); %J ust for saving state his.
prevTime = 0; %last time step in real time
%write your code here calling the pred_step.m and upd_step.m functions
for i = 1:length(sampledTime)

    if (i == 1)
        dt = sampledTime(1);
        [covarEst, uEst] = pred_step(uPrev, covarPrev, sampledData(1).omg, sampledData(1).acc, dt);
        [uCurr, covar_curr] = upd_step(Z(:,1), covarEst, uEst);

    else
        dt = sampledTime(i) - sampledTime(i-1);
        [covarEst, uEst] = pred_step(uCurr, covar_curr, sampledData(i).omg, sampledData(i).acc, dt);
        [uCurr, covar_curr] = upd_step( Z(:,i), covarEst, uEst);
    end

    uPrev = uCurr;
    covarPrev = covar_curr;
    savedStates(:,i) = uCurr;

end

plotData(savedStates, sampledTime, sampledVicon, 1, datasetNum);