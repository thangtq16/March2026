%% NR Cell Performance with Downlink MU-MIMO
%% Custom Path Library
custom_lib_path_5g = fullfile(pwd, '5g');
custom_lib_path_wn = fullfile(pwd, 'wirelessnetwork');

addpath(genpath(custom_lib_path_5g));
addpath(genpath(custom_lib_path_wn));

rehash toolboxcache;

%% Initialize Simulation
wirelessnetworkSupportPackageCheck

rng("default")
numFrameSimulation = 2;
networkSimulator = wirelessNetworkSimulator.init;

duplexType = "TDD";

%% Configure gNB Node
gNBPosition = [0 0 30];
gNB = nrGNB(Position=gNBPosition, TransmitPower=60, SubcarrierSpacing=30000, ...
    CarrierFrequency=3.7e9, ChannelBandwidth=100e6, NumTransmitAntennas=32, NumReceiveAntennas=32, ...
    DuplexMode=duplexType, ReceiveGain=11, SRSPeriodicityUE=5, NumResourceBlocks=273);

csiMeasurementSignalDLType = "CSI-RS";

%% Configure MU-MIMO and Scheduler
muMIMOConfiguration = struct( ...
    MaxNumUsersPaired=4, MaxNumLayers=16, MinNumRBs=1, ...
    MinCQI=7, SemiOrthogonalityFactor=0.5);

muLinkAdaptationConfigDL = struct(InitialOffset = 0);

allocationType = 0;
configureScheduler(gNB, Scheduler="ProportionalFair", ResourceAllocationType=allocationType, MaxNumUsersPerTTI=4, ...
    MUMIMOConfigDL=muMIMOConfiguration, CSIMeasurementSignalDL=csiMeasurementSignalDLType, LinkAdaptationConfigDL=muLinkAdaptationConfigDL);

%% Define UE Positions and Deployment
numUEs = 10;
nearFieldLimit = 10;
farFieldLimit = 100;
ueRelPosition = [rand(numUEs, 1)*(farFieldLimit-nearFieldLimit)+nearFieldLimit (rand(numUEs, 1)-0.5)*120 zeros(numUEs, 1)];
% ueRelPosition = [ones(numUEs, 1)*1000 (ones(numUEs, 1)-0.5)*120 zeros(numUEs, 1)];
[xPos, yPos, zPos] = sph2cart(deg2rad(ueRelPosition(:, 2)), deg2rad(ueRelPosition(:, 3)), ...
    ueRelPosition(:, 1));

uePositions = [xPos yPos zPos-30] + gNBPosition;
ueNames = "UE-" + (1:size(uePositions, 1));

figure('Name','UE and gNB Positions','NumberTitle','off');
tiledlayout(1,2)

nexttile
plot(uePositions(:,1), uePositions(:,2), 'bo', 'MarkerFaceColor','b')
hold on
plot(gNBPosition(1), gNBPosition(2), 'r^', 'MarkerSize',10, 'MarkerFaceColor','r')
text(uePositions(:,1)+5, uePositions(:,2)+5, ueNames, 'FontSize',8)
xlabel('X (m)'); ylabel('Y (m)');
title('Top-down (X-Y) positions');
axis equal
grid on
legend('UEs','gNB','Location','best')

nexttile
scatter3(uePositions(:,1), uePositions(:,2), uePositions(:,3), 50, 'b', 'filled')
hold on
scatter3(gNBPosition(1), gNBPosition(2), gNBPosition(3), 100, 'r', 'filled','^')
text(uePositions(:,1)+5, uePositions(:,2)+5, uePositions(:,3)+2, ueNames, 'FontSize',8)
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D positions (including elevation)');
axis vis3d
grid on
view(45,25)
legend('UEs','gNB','Location','best')

%% Create UE Nodes and Connectivity
UEs = nrUE(Name=ueNames, Position=uePositions, ReceiveGain=0, NumTransmitAntennas=4, NumReceiveAntennas=4);

connectUE(gNB, UEs, FullBufferTraffic="DL", CSIReportPeriodicity=10)

addNodes(networkSimulator, gNB)
addNodes(networkSimulator, UEs)

%% Channel Modeling
channelConfig = struct(DelayProfile="CDL-D", DelaySpread=450e-9, MaximumDopplerShift=5);
channels = hNRCreateCDLChannels(channelConfig, gNB, UEs);

customChannelModel  = hNRCustomChannelModel(channels);
addChannelModel(networkSimulator, @customChannelModel.applyChannelModel)

%% Logging and Metrics Setup
enableTraces = true;

if enableTraces
    simSchedulingLogger = helperNRSchedulingLogger(numFrameSimulation, gNB, UEs);
    simPhyLogger = helperNRPhyLogger(numFrameSimulation, gNB, UEs);
end

numMetricPlotUpdates = 2000;

metricsVisualizer = helperNRMetricsVisualizer(gNB, UEs, RefreshRate=numMetricPlotUpdates, ...
    PlotSchedulerMetrics=true, PlotPhyMetrics=false, PlotCDFMetrics=true, LinkDirection=0);

simulationLogFile = "simulationLogs";

%% Run Simulation
simulationTime = numFrameSimulation*1e-2;
run(networkSimulator, simulationTime);

%% Results Analysis
displayPerformanceIndicators(metricsVisualizer)

if enableTraces
    simulationLogs = cell(1, 1);
    if gNB.DuplexMode == "FDD"
        logInfo = struct(DLTimeStepLogs=[], ULTimeStepLogs=[], ...
            SchedulingAssignmentLogs=[], PhyReceptionLogs=[]);
        [logInfo.DLTimeStepLogs, logInfo.ULTimeStepLogs] = getSchedulingLogs(simSchedulingLogger);
    else
        logInfo = struct(TimeStepLogs=[], SchedulingAssignmentLogs=[], PhyReceptionLogs=[]);
        logInfo.TimeStepLogs = getSchedulingLogs(simSchedulingLogger);
    end
    logInfo.SchedulingAssignmentLogs = getGrantLogs(simSchedulingLogger);
    logInfo.PhyReceptionLogs = getReceptionLogs(simPhyLogger);
    simulationLogs{1} = logInfo;
    save(simulationLogFile, "simulationLogs")
end

%% User Pairing Statistics
if enableTraces
    avgNumUEsPerRB = calculateAvgUEsPerRBDL(logInfo, gNB.NumResourceBlocks, allocationType, duplexType);

    figure;
    theme("light")
    histogram(avgNumUEsPerRB, 'BinWidth', 0.1);
    title('Distribution of Average Number of UEs per RB in DL Slots');
    xlabel('Average Number of UEs per RB');
    ylabel('Number of Occurrence');
    grid on;

    [pairCounts, pairValues] = countPairedUEsPerRBDL(logInfo, gNB.NumResourceBlocks, ...
        allocationType, duplexType, muMIMOConfiguration.MaxNumUsersPaired);
    pairingTable = table(pairValues', pairCounts', 'VariableNames', {'NumUEs', 'NumRBs'});
    disp(pairingTable)

    slotSummary = summarizeMuMimoPerSlot(logInfo, gNB.NumResourceBlocks, allocationType, ...
        duplexType, muMIMOConfiguration.MaxNumUsersPaired);
    disp(slotSummary)

    figure;
    theme("light")
    bar(pairValues, pairCounts);
    title('Count of RBs by Paired UE Count (DL Slots)');
    xlabel('Paired UE Count');
    ylabel('Number of RBs');
    xticks(pairValues);
    grid on;
end

%% Local Functions
function avgUEsPerRB = calculateAvgUEsPerRBDL(logInfo, numResourceBlocks, ratType, duplexMode)
if strcmp(duplexMode, 'TDD')
    timeStepLogs = logInfo.TimeStepLogs;
    freqAllocations = timeStepLogs(:, 5);
elseif strcmp(duplexMode, 'FDD')
    timeStepLogs = logInfo.DLTimeStepLogs;
    freqAllocations = timeStepLogs(:, 4);
end

numOfSlots = size(timeStepLogs, 1) - 1;

if ~ratType
    numRBG = size(freqAllocations{2}, 2);
    P = ceil(numResourceBlocks / numRBG);
    numRBsPerRBG = P * ones(1, numRBG);
    remainder = mod(numResourceBlocks, P);
    if remainder > 0
        numRBsPerRBG(end) = remainder;
    end
end

avgUEsPerRB = zeros(1, numOfSlots);

for slotIdx = 1:numOfSlots
    if strcmp(duplexMode, 'TDD')
        slotType = timeStepLogs{slotIdx + 1, 4};
        if ~strcmp(slotType, 'DL')
            continue;
        end
    end

    freqAllocation = freqAllocations{slotIdx + 1};

    if ~ratType
        totalUniqueUEs = sum(arrayfun(@(rbgIdx) nnz(freqAllocation(:, rbgIdx) > 0) * ...
            numRBsPerRBG(rbgIdx), 1:length(numRBsPerRBG)));
        avgUEsPerRB(slotIdx) = totalUniqueUEs / numResourceBlocks;
    else
        ueRBUsage = zeros(1, numResourceBlocks);
        for ueIdx = 1:size(freqAllocation, 1)
            startRB = freqAllocation(ueIdx, 1);
            numContiguousRBs = freqAllocation(ueIdx, 2);
            ueRBUsage(startRB + 1:(startRB + numContiguousRBs)) = ...
                ueRBUsage(startRB + 1:(startRB + numContiguousRBs)) + 1;
        end
        avgUEsPerRB(slotIdx) = mean(ueRBUsage(ueRBUsage > 0));
    end
end
avgUEsPerRB = avgUEsPerRB(avgUEsPerRB > 0);
end

function [pairCounts, pairValues] = countPairedUEsPerRBDL(logInfo, numResourceBlocks, ...
    ratType, duplexMode, maxPairs)
if strcmp(duplexMode, 'TDD')
    timeStepLogs = logInfo.TimeStepLogs;
    freqAllocations = timeStepLogs(:, 5);
elseif strcmp(duplexMode, 'FDD')
    timeStepLogs = logInfo.DLTimeStepLogs;
    freqAllocations = timeStepLogs(:, 4);
end
numOfSlots = size(timeStepLogs, 1) - 1;
pairCounts = zeros(1, maxPairs);
if ~ratType
    numRBG = size(freqAllocations{2}, 2);
    P = ceil(numResourceBlocks / numRBG);
    numRBsPerRBG = P * ones(1, numRBG);
    remainder = mod(numResourceBlocks, P);
    if remainder > 0
        numRBsPerRBG(end) = remainder;
    end
end
for slotIdx = 1:numOfSlots
    if strcmp(duplexMode, 'TDD')
        slotType = timeStepLogs{slotIdx + 1, 4};
        if ~strcmp(slotType, 'DL')
            continue;
        end
    end
    freqAllocation = freqAllocations{slotIdx + 1};
    if isempty(freqAllocation)
        continue;
    end
    if ~ratType
        for rbgIdx = 1:size(freqAllocation, 2)
            ueCount = nnz(freqAllocation(:, rbgIdx) > 0);
            if ueCount == 0
                continue;
            end
            if ueCount > numel(pairCounts)
                pairCounts(ueCount) = 0;
            end
            pairCounts(ueCount) = pairCounts(ueCount) + numRBsPerRBG(rbgIdx);
        end
    else
        ueRBUsage = zeros(1, numResourceBlocks);
        for ueIdx = 1:size(freqAllocation, 1)
            startRB = freqAllocation(ueIdx, 1);
            numContiguousRBs = freqAllocation(ueIdx, 2);
            if numContiguousRBs <= 0
                continue;
            end
            ueRBUsage(startRB + 1:(startRB + numContiguousRBs)) = ...
                ueRBUsage(startRB + 1:(startRB + numContiguousRBs)) + 1;
        end
        for rbIdx = 1:numResourceBlocks
            ueCount = ueRBUsage(rbIdx);
            if ueCount == 0
                continue;
            end
            if ueCount > numel(pairCounts)
                pairCounts(ueCount) = 0;
            end
            pairCounts(ueCount) = pairCounts(ueCount) + 1;
        end
    end
end
pairValues = 1:numel(pairCounts);
end

function slotSummary = summarizeMuMimoPerSlot(logInfo, numResourceBlocks, ratType, ...
    duplexMode, maxPairs)
if strcmp(duplexMode, 'TDD')
    timeStepLogs = logInfo.TimeStepLogs;
    freqAllocations = timeStepLogs(:, 5);
else
    timeStepLogs = logInfo.DLTimeStepLogs;
    freqAllocations = timeStepLogs(:, 4);
end
headers = timeStepLogs(1, :);
frameCol = find(strcmp(headers, 'Frame'), 1);
slotCol = find(strcmp(headers, 'Slot'), 1);
typeCol = find(strcmp(headers, 'Type'), 1);
numOfSlots = size(timeStepLogs, 1) - 1;
if ~ratType
    numRBG = size(freqAllocations{2}, 2);
    P = ceil(numResourceBlocks / numRBG);
    numRBsPerRBG = P * ones(1, numRBG);
    remainder = mod(numResourceBlocks, P);
    if remainder > 0
        numRBsPerRBG(end) = remainder;
    end
end
frames = zeros(numOfSlots, 1);
slots = zeros(numOfSlots, 1);
slotTypes = strings(numOfSlots, 1);
hasMu = false(numOfSlots, 1);
maxUePerRb = zeros(numOfSlots, 1);
numMuRbs = zeros(numOfSlots, 1);
maxMuLayers = zeros(numOfSlots, 1);
maxLayersUsed = zeros(numOfSlots, 1);
keepIdx = false(numOfSlots, 1);
for slotIdx = 1:numOfSlots
    if strcmp(duplexMode, 'TDD') && ~isempty(typeCol)
        slotType = timeStepLogs{slotIdx + 1, typeCol};
        if ~strcmp(slotType, 'DL')
            continue;
        end
        slotTypes(slotIdx) = slotType;
    end
    freqAllocation = freqAllocations{slotIdx + 1};
    if isempty(freqAllocation)
        uePerRb = zeros(1, numResourceBlocks);
    elseif ~ratType
        ueCountsRbg = sum(freqAllocation > 0, 1);
        uePerRb = repelem(ueCountsRbg, numRBsPerRBG);
        uePerRb = uePerRb(1:numResourceBlocks);
    else
        uePerRb = zeros(1, numResourceBlocks);
        for ueIdx = 1:size(freqAllocation, 1)
            startRB = freqAllocation(ueIdx, 1);
            numContiguousRBs = freqAllocation(ueIdx, 2);
            if numContiguousRBs <= 0
                continue;
            end
            uePerRb(startRB + 1:(startRB + numContiguousRBs)) = ...
                uePerRb(startRB + 1:(startRB + numContiguousRBs)) + 1;
        end
    end
    maxUe = max(uePerRb);
    muRbs = sum(uePerRb >= 2);
    frameVal = timeStepLogs{slotIdx + 1, frameCol};
    slotVal = timeStepLogs{slotIdx + 1, slotCol};
    frames(slotIdx) = frameVal;
    slots(slotIdx) = slotVal;
    maxUePerRb(slotIdx) = min(maxUe, maxPairs);
    hasMu(slotIdx) = maxUe >= 2;
    numMuRbs(slotIdx) = muRbs;
    maxLayersUsed(slotIdx) = maxLayersUsedInSlot(logInfo, frameVal, slotVal);
    maxMuLayers(slotIdx) = maxMuMimoLayersInSlot(logInfo, numResourceBlocks, ...
        ratType, frameVal, slotVal, numRBsPerRBG);
    keepIdx(slotIdx) = true;
end
slotSummary = table(frames(keepIdx), slots(keepIdx), slotTypes(keepIdx), ...
    hasMu(keepIdx), maxUePerRb(keepIdx), numMuRbs(keepIdx), ...
    maxMuLayers(keepIdx), maxLayersUsed(keepIdx), 'VariableNames', ...
    {'Frame', 'Slot', 'SlotType', 'HasMuMimo', 'MaxUesPerRb', 'NumMuRbs', ...
    'MaxMuMimoLayers', 'MaxLayersUsed'});
end

function maxLayersUsed = maxLayersUsedInSlot(logInfo, frameVal, slotVal)
grantLogs = logInfo.SchedulingAssignmentLogs;
if isempty(grantLogs)
    maxLayersUsed = 0;
    return;
end
headers = grantLogs(1, :);
frameCol = find(strcmp(headers, 'Frame'), 1);
slotCol = find(strcmp(headers, 'Slot'), 1);
grantTypeCol = find(strcmp(headers, 'Grant Type'), 1);
numLayersCol = find(strcmp(headers, 'NumLayers'), 1);

if isempty(frameCol) || isempty(slotCol) || isempty(grantTypeCol) || isempty(numLayersCol)
    maxLayersUsed = 0;
    return;
end

grantRows = grantLogs(2:end, :);
isDl = strcmp(grantRows(:, grantTypeCol), 'DL');
isFrame = cell2mat(grantRows(:, frameCol)) == frameVal;
isSlot = cell2mat(grantRows(:, slotCol)) == slotVal;
slotMask = isDl & isFrame & isSlot;
if ~any(slotMask)
    maxLayersUsed = 0;
    return;
end

layerCells = grantRows(slotMask, numLayersCol);
layerVals = zeros(numel(layerCells), 1);
for idx = 1:numel(layerCells)
    val = layerCells{idx};
    if isempty(val) || ~isnumeric(val)
        layerVals(idx) = NaN;
    else
        layerVals(idx) = val;
    end
end
layerVals = layerVals(~isnan(layerVals) & layerVals > 0);
if isempty(layerVals)
    maxLayersUsed = 0;
else
    maxLayersUsed = max(layerVals);
end
end

function maxMuLayers = maxMuMimoLayersInSlot(logInfo, numResourceBlocks, ratType, ...
    frameVal, slotVal, numRBsPerRBG)
grantLogs = logInfo.SchedulingAssignmentLogs;
if isempty(grantLogs)
    maxMuLayers = 0;
    return;
end
headers = grantLogs(1, :);
frameCol = find(strcmp(headers, 'Frame'), 1);
slotCol = find(strcmp(headers, 'Slot'), 1);
grantTypeCol = find(strcmp(headers, 'Grant Type'), 1);
numLayersCol = find(strcmp(headers, 'NumLayers'), 1);
freqAllocCol = find(strcmp(headers, 'Frequency Allocation'), 1);

grantRows = grantLogs(2:end, :);
isDl = strcmp(grantRows(:, grantTypeCol), 'DL');
isFrame = cell2mat(grantRows(:, frameCol)) == frameVal;
isSlot = cell2mat(grantRows(:, slotCol)) == slotVal;
slotMask = isDl & isFrame & isSlot;
if ~any(slotMask)
    maxMuLayers = 0;
    return;
end
slotGrants = grantRows(slotMask, :);
rbUeCount = zeros(1, numResourceBlocks);
rbLayerSum = zeros(1, numResourceBlocks);
for rowIdx = 1:size(slotGrants, 1)
    freqAllocation = slotGrants{rowIdx, freqAllocCol};
    numLayers = slotGrants{rowIdx, numLayersCol};
    if isempty(freqAllocation) || numLayers <= 0
        continue;
    end
    if ~ratType
        for rbgIdx = 1:numel(freqAllocation)
            if freqAllocation(rbgIdx) <= 0
                continue;
            end
            rbStart = sum(numRBsPerRBG(1:rbgIdx-1)) + 1;
            rbEnd = min(rbStart + numRBsPerRBG(rbgIdx) - 1, numResourceBlocks);
            rbUeCount(rbStart:rbEnd) = rbUeCount(rbStart:rbEnd) + 1;
            rbLayerSum(rbStart:rbEnd) = rbLayerSum(rbStart:rbEnd) + numLayers;
        end
    else
        startRB = freqAllocation(1);
        numContiguousRBs = freqAllocation(2);
        rbStart = startRB + 1;
        rbEnd = min(rbStart + numContiguousRBs - 1, numResourceBlocks);
        rbUeCount(rbStart:rbEnd) = rbUeCount(rbStart:rbEnd) + 1;
        rbLayerSum(rbStart:rbEnd) = rbLayerSum(rbStart:rbEnd) + numLayers;
    end
end
muMask = rbUeCount >= 2;
if any(muMask)
    maxMuLayers = max(rbLayerSum(muMask));
else
    maxMuLayers = 0;
end
end