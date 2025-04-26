%% Create Taipei Metro NavGraph

%Define the names and the location(states) of the stations
names = {
    'Taipei Main Station';%1
    'Zhongshan';
    'NTU Hospital';
    'Chiang Kai-Shek Memorial Hall';
    'Dongmen';%5
    'Songjiang Nanjing';
    'Beimen';
    'Ximen';
    'Xiaonanmen';
    'Guting';%10
    'Longshan Temple';
    'Shandao Temple';
    'Zhongxiao Xinsheng';
    'Xingtian Temple';
    'Shuanglian';%15
    'Mingquan W. Rd.'
    'Zhongshan Elementary School'
    'Daqiaotou'
    'Nanjing Fuxing'
    'Zhongxiao Fuxing'%20
    'Zhongshan Junior High School'
    'Daan'
    'Daan Park'
};

states = [
     0,  0;    %1
     0,  1;
     0, -0.5;
     0, -1;
     1, -1;    %5
     1,  1;
    -1,  0.5;
    -1, -0.5;
    -1, -1;
     0.5, -1.5;%10
    -2, -0.5;
     0.5, 0;
     1, 0;
     1, 1.7;
     0, 1.7;   %15
     0, 2;
     0.7, 2;
     -1, 2;
     2, 1;
     2, 0;     %20
     2, 1.8;
     2, -1;
     1.5, -1;

];

%Create the links between each of stations
links = [
    1, 2;   % Taipei Main Station -> Zhongshan
    1, 3;   % Taipei Main Station -> NTU Hospital
    3, 4;   % NTU Hospital -> Chiang Kai-Shek Memorial Hall
    4, 5;   % Chiang Kai-Shek -> Dongmen
    6, 2;   % Songjiang Nanjing -> Zhongshan
    2, 7;   % Zhongshan -> Beimen
    7, 8;   % Beimen -> Ximen
    8, 9;   % Ximen -> Xiaonanmen
    9, 4;   % Xiaonanmen -> Chiang Kai-Shek Memorial Hall
    4, 10;  % Chiang Kai-Shek -> Guting
    1, 8;   % Taipei Main Station -> Ximen
    8, 11;  % Ximen -> Longshan Temple
    1, 12;  % Taipei Main Station -> Shandao Temple
    12, 13; % Shandao Temple -> Zhongxiao Xinsheng
    14, 6;  % Xingtian Temple -> Songjiang Nanjing
    6, 13;  % Songjiang Nanjing -> Zhongxiao Xinsheng
    13, 5;  % Zhongxiao Xinsheng -> Dongmen
    5, 10;  % Dongmen -> Guting
    2, 15;  % Zhongshan -> Shuanglian
    15, 16; % Shuanglian -> Mingquan W. Rd.
    16, 17; % Mingquan W. Rd. -> Zhongshan Elementary School
    17, 14; % Zhongshan Elementary School -> Xingtian Temple
    16, 18; % Mingquan W. Rd. -> Daqiaotou 
    19, 6;  % Nanjing Fuxing -> Shongjiang Nanjing
    20, 19; % Zhongxiao Fuxing -> Nanjing Fuxing
    20, 13; % Zhongxiao Fuxing -> Zhongxiao Xinsheng
    21, 19; % Zhongshan Junior High School -> Nanjing Fuxing
    22, 20; % Daan -> Zhongshao Fuxing
    22, 23; % Daan -> Daan Park
    23, 5;  % Daan Park -> Dongmen
];
%Intercommunicate the station to station
links = [links; links(:,[2, 1])];

%Calculate the weights
weights = vecnorm(states(links(1:end/2,1),:) - states(links(1:end/2,2),:), 2, 2);
weights = [weights; weights];

%Generate navGraph
G = navGraph(states, links, Weight=weights, Name=names);

%% Utilize A* planner
% Create a graph-based A* path planner
planner = plannerAStar(G);
planner2 = copy(planner)

planner.HeuristicCostFcn = @(state1,state2) ...
   sum(abs(state1-state2),2)/100;
% Set the start and the goal station
start = "Xiaonanmen";
goal = "Xingtian Temple";

[pathOutput,solutionInfo] = plan(planner,start,goal);
%% Show the figure
figure
h = show(G);
set(h,XData=G.States.StateVector(:,1), ...
      YData=G.States.StateVector(:,2))
pathStateIDs = solutionInfo.PathStateIDs;
highlight(h,pathStateIDs,EdgeColor="#FFFF00",LineWidth=4)
highlight(h,pathStateIDs(1),NodeColor="#FF0000",MarkerSize=5)
highlight(h,pathStateIDs(end),NodeColor="#0000FF",MarkerSize=5)