clc;
clear;
close all;
% Simulink.data.dictionary.closeAll

disp("AP参数初始化");

%% 加载路径
addpath('./mcTrajectoryGenerator');
addpath('./commonFunction');
addpath('./routePlan');

%% 加载数据字典中的枚举类型
dictPath_c = which('apDataDictionary.sldd');

if isempty(dictPath_c)
    error('找不到数据字典 apDataDictionary.sldd，请检查当前目录或MATLAB路径。');
end

apDataDictObj = Simulink.data.dictionary.open(dictPath_c);

%% 加载参数
autopilotParams;
APTest;