clear all;
hold on;
format long
% Set Matlab path to include OpenSim libraries
opensimroot = ['C:\opensim_build\opensim_install\'];
addpath([opensimroot 'bin'], [opensimroot 'sdk\lib']);

% Set Windows System path to include OpenSim libraries
setenv('PATH', [[opensimroot 'bin'] ';' [opensimroot 'sdk\lib'] ';' getenv('PATH')]);

import org.opensim.modeling.*     
osimModel=Model('simple23.osim');
%osimModel=Model('gait10dof18musc.osim');
% osimModel=Model('gait10dof18musc_Prescribed.osim'); 
% osimModel.setUseVisualizer(true);
time_vector = xlsread('gait_time_vector.xlsx');
%ZMP = xlsread('ZMP.xlsx');
myState = osimModel.initSystem();                    
coordSet = osimModel.getCoordinateSet();
muscleSet = osimModel.getMuscles();
nMuscles = muscleSet.getSize();
nCoordinates = coordSet.getSize();
coordSet = osimModel.updCoordinateSet();
state = osimModel.initSystem(); 
multibodySubsystem = osimModel.getMultibodySystem();
matterSubsystem = osimModel.getMatterSubsystem();
bodySet = osimModel.getBodySet();
nBodies = bodySet.getSize();
for m = 1:nMuscles
        myForce = muscleSet.get(m-1);
        muscleNames(m) = myForce.getName();     
end

for c = 1:nCoordinates
        myCoordinate = coordSet.get(c-1);
        coordNames(c) = myCoordinate.getName();
end
%osimModel.computeStateVariableDerivatives(state);

for b = 1:nBodies
        myBody = bodySet.get(b-1);
        myBody.getName();
        bodyNames(b) = myBody.getName();
        bodyMassProperties(b) = myBody.getMassProperties();
        bodyMasses(b) = myBody.getMass;
        massCenter = Vec3();
        myBody.getMassCenter();
        bodyMassCenters(b) = massCenter;
end


%}

% Load model states (including muscle states)
statesStorage = Storage('simple23_states_modified.sto');
%statesStorage = Storage('walk_subject_states_modified.sto');
nTimeFrames = statesStorage.getSize;
timeArray = ArrayDouble();
statesStorage.getTimeColumn(timeArray);
initialTimeIndex0 = statesStorage.findIndex(statesStorage.getFirstTime); % Note: C++ arrays use index starting at 0
finalTimeIndex0 = statesStorage.findIndex(statesStorage.getLastTime); % Note: C++ arrays use index starting at 0
labelsArray = statesStorage.getColumnLabels;
nStates = labelsArray.getSize-1; % -1 to remove time from size count

currentTimeFrame = 1
cla;
frameCountActual = frameCountActual +1;
statesStorage.getTime(currentTimeFrame, state.updTime);
currentTime = state.getTime;
osimModel.setAllControllersEnabled(true);

disp('model states were updated to data; verify this:');
%disp(state.getY); %check state to verify changes
disp(osimModel.getStateVariableValues(state));

  for currentStateVariable = 1 :statesStorage.getStateVector(currentTimeFrame).getSize()
     osimModel.setStateVariableValue(state,labelsArray.get(currentStateVariable),statesStorage.getStateVector(currentTimeFrame).getData.get(currentStateVariable-1))
 end

osimModel.assemble(state);

osimModel.computeStateVariableDerivatives(state);
%disp(state.getY); %check state to verify changes
disp(osimModel.getStateVariableValues(state));
% Compute the moment arm matrix for all (Cases 1-3) torque computations
muscleSet = osimModel.getMuscles();
for c = 1:nCoordinates
    for m = 1:nMuscles
        thismuscle = muscleSet.get(muscleNames(m));
        myCoordinate = coordSet.get(c-1);
        Moment_Arm = thismuscle.computeMomentArm(state, myCoordinate);
        Momentarms(c,m) = Moment_Arm;
    end  
end

%osimModel.getMatterSubsystem().multiplyByFrameJacobian(state,9,osimModel.getBodySet().get(9).getMassCenter(),torquesCase3,wrenchCase3);
% Lock the model coordinates for 0 velocity
for c = 1:nCoordinates
    originalLockedFlag(c) = osimModel.getCoordinateSet().get(c-1).getLocked(state);
    osimModel.getCoordinateSet().get(c-1).setLocked(state, true);
end

disp('model Us are set to zero; verify this:');
%disp(state.getU); %check state to verify changes
disp(osimModel.getStateVariableValues(state));

% Set the model U's to 0 for Case 2
for currentStateVariable = 1 :statesStorage.getStateVector(currentTimeFrame).getSize()
        currentStateName = char(labelsArray.get(currentStateVariable));
        if(strcmp(currentStateName(end-1:end), 'ed'))
        osimModel.setStateVariableValue(state, currentStateName, 0.0);
        end
        osimModel.setStateVariableValue(state,currentStateName,statesStorage.getStateVector(currentTimeFrame).getData.get(currentStateVariable-1))
end

% osimModel.setPropertiesFromState(state);
osimModel.assemble(state);
% osimModel.equilibrateMuscles(state);
osimModel.computeStateVariableDerivatives(state);
%disp(state.getU); %check state to verify changes
disp(osimModel.getStateVariableValues(state));