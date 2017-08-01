M = dlmread('example_degeneracy_matrix.txt');
lambdas = dlmread('example_degeneracy_eigenvalues.txt');

% Sort row wrt to the eigenvalues
M = [lambdas, M]
M = sortrows(M, 1)

% Get eigenvalues < than threshold
threshold = 0.0001;
lambda_small = [];
lambda_small = M(M(:,1) < threshold)

% Plot matrix as in "On Degeneracy of Optimization-based State Estimation Problems", J. Zhang, 2016 
figure;
imagesc(M(:,2:end));
set(gca,'YTickLabel',['v1'; 'v2'; 'v3'; 'v4'; 'v5'; 'v6'],'XTickLabel',['R'; 'P'; 'Y'; 'F'; 'L'; 'U'],'XAxisLocation','top');
colormap(flipud(gray(256)));
colorbar;
hold on
y_line = (size(lambda_small,1)+0.5);
plot([0.5 6.5],[y_line y_line],'r','LineWidth',3);
drawnow
hold off
