% check_environment.m
% Check MATLAB environment for single link inverted pendulum project

clear; clc;

disp('Checking MATLAB environment...');
disp('--------------------------------');

disp('Current project folder:');
disp(pwd);

disp(' ');
disp('MATLAB version:');
disp(version);

installed_products = ver;
product_names = string({installed_products.Name});

required_products = [
    "MATLAB"
    "Simulink"
    "Control System Toolbox"
    "Simscape"
    "Simscape Multibody"
    ];

disp(' ');
disp('Required products:');

for i = 1:length(required_products)
    product = required_products(i);

    if any(product_names == product)
        fprintf('[OK] %s is installed.\n', product);
    else
        fprintf('[MISSING] %s is not installed.\n', product);
    end
end

disp(' ');
disp('Checking important functions:');

if exist('lqr', 'file') == 2
    disp('[OK] lqr() is available.');
else
    disp('[MISSING] lqr() is not available.');
end

if exist('ss', 'file') == 2
    disp('[OK] ss() is available.');
else
    disp('[MISSING] ss() is not available.');
end

if exist('ctrb', 'file') == 2
    disp('[OK] ctrb() is available.');
else
    disp('[MISSING] ctrb() is not available.');
end

if exist('simulink', 'file') == 2
    disp('[OK] Simulink is available.');
else
    disp('[MISSING] Simulink is not available.');
end

disp(' ');
disp('Environment check completed.');