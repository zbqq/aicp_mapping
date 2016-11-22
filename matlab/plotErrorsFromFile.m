close all;
clear all;

% Read from file
file = fopen('../python/errorsFile.txt','r'); % Open .txt file for reading
%file = fopen(fullfile('../','matlab/','errorsFile.txt','r'));
data = [];
correction_flag = [];
i = 1;
j = 1;
while ~feof(file)
    line = fgets(file); % Read line by line
    data(i,:) = sscanf(line,'%d\t%f\t%f\t%f\t%d'); % sscanf can read only numeric data
    % Set up matrix to plot correction flag
    if (data(i,5) == 1)
        correction_flag(j,:) = [data(i,2), data(i,3), data(i,4)];
        j = j+1;
    end
    i = i+1;
end
fclose(file);

%%

% Plot 3D translation errors
figure;

hPlotTop = axes('Units','normalized','Position',[0.1 0.58 0.72 .38]);
plot(hPlotTop, data(:,2), data(:,3), 'r', 'LineWidth', 2), xlabel('Time [seconds]'), ylabel('3D Transl. Error [m]');
hold on;
plot(correction_flag(:,1), correction_flag(:,2), 'ko', 'LineWidth', 2);
grid on;

hPlotBot = axes('Units','normalized','Position',[0.1 0.1 0.72 .38]);
plot(hPlotBot, data(:,2), data(:,4), 'r', 'LineWidth', 2), xlabel('Time [seconds]'), ylabel('3D Rot. Error [deg]');
hold on;
plot(correction_flag(:,1), correction_flag(:,3), 'ko', 'LineWidth', 2);
grid on;

% Boxplots for quantiles 25, 50, 75, 95
q_t = quantile(data(:,3),[.25 .50 .75 .95]);
q_t = q_t';

hBoxTop = axes('Units','normalized','Position',[.87 0.56 0.09 .4]);
box = boxplot(hBoxTop, data(:,3),'labels',{'Q'});

% Modify upper whisker's length
set(box(1,:),{'Ydata'},num2cell(q_t(end-1:end,:),1)');
% Modify lower whisker's length
%set(box(2,:),{'Ydata'},num2cell(q_t(2:-1:1,:),1)');
% Modify upper whisker's endbar
set(box(3,:),{'Ydata'},num2cell(q_t([end end],:),1)');
% Modify lower whisker's endbar
set(box(4,:),{'Visible'},{'off'});
% Modify body
%set(box(5,:),{'Ydata'},num2cell(q_t([2 3 3 2 2],:),1)');
% Median? (nothing specified, just invisible)
set(box(6,:),{'Visible'},{'on'});
set(box(6,:),'LineWidth', 2);
% Outliers? (set the old ones to off and draw new ones)
set(box(7,:),{'Visible'},{'off'});

set(gca, 'YAxisLocation', 'right');
ax.XTickLabelRotation=90;
grid on;

q_r = quantile(data(:,4),[.25 .50 .75 .95]);
q_r = q_r';

hBoxBot = axes('Units','normalized','Position',[.843 0.085 0.09 .4]);
box = boxplot(hBoxBot, data(:,4),'labels',{'Q'});

% Modify upper whisker's length
set(box(1,:),{'Ydata'},num2cell(q_r(end-1:end,:),1)');
% Modify lower whisker's length
%set(box(2,:),{'Ydata'},num2cell(q_r(2:-1:1,:),1)');
% Modify upper whisker's endbar
set(box(3,:),{'Ydata'},num2cell(q_r([end end],:),1)');
% Modify lower whisker's endbar
set(box(4,:),{'Visible'},{'off'});
% Modify body
%set(box(5,:),{'Ydata'},num2cell(q_r([2 3 3 2 2],:),1)');
% Median? (nothing specified, just invisible)
set(box(6,:),{'Visible'},{'on'});
set(box(6,:),'LineWidth', 2);
% Outliers? (set the old ones to off and draw new ones)
set(box(7,:),{'Visible'},{'off'});

set(gca, 'YAxisLocation', 'right');
ax.XTickLabelRotation=90;
grid on;