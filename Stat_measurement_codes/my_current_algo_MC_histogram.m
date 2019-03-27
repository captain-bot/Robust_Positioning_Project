clc
clear
close all

% Load the joint angle file
load("empirical_file.mat", "solIKs_array", "success_count_array");

% Number of bins
num_buck = 8;

% joint limits
jl_min = [-1.7016 -2.147 -3.0541 -0.05 -3.059 -1.5707 -3.059];
jl_max = [1.7016 1.047 3.0541 2.618 3.059 2.094 3.059];

for i = 1:length(jl_min)
    figure(i)
    edges = linspace(jl_min(i), jl_max(i), num_buck);
    histogram(solIKs_array(:, i), edges);
    grid on
%     x_write = strcat('\theta_', num2str(i), '[rad]')
    xlabel(strcat('\theta_', num2str(i), '[rad]'))
    ylabel('frequency')
end