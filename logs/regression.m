function [ coeffs, R2 ] = regression( filename )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

data = import_function(filename);

commands = zeros(1, size(data, 1)*(size(data, 2) - 1));
speeds = zeros(1, size(data, 1)*(size(data, 2) - 1));

for i = 1:size(data, 1)
    c = data(i, 1);
    s = data(i, 2:end);
    index = (i-1)*size(s, 2)+1;
    commands(index:(index+size(s,2)-1)) = repmat(c, size(s));
    speeds(index:(index+size(s,2)-1)) = s;  
end

% We want a function to predict speed from command
z = 1:size(speeds, 2);

% linear function
fit = @(a) a(1)*commands(z) + a(2);

% sum of square error
f_error = @(a) sum((fit(a) - speeds(z)).^2);

options = optimset('MaxFunEvals', 1000000, 'MaxIter', 1000000);

% find coeffs a(1) and a(2) to minimize square error
coeffs = fminsearch(f_error, [1, 0], options);
RSS = f_error(coeffs);
TSS = sum((speeds - mean(speeds)).^2);
R2 = 1-(RSS/TSS);

figure()
hold on
plot(commands, speeds, 'bo')
plot(commands, fit(coeffs), 'r-')
xlabel('motor command')
ylabel('observed speed, rad/s')
title(['Motor: ', filename(18:end), ...
    ', Fit: s = ', num2str(coeffs(1)), 'c + ', num2str(coeffs(2)), ...
    ', R^2: ', num2str(R2)])

end

