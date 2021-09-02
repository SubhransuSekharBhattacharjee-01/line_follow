close all
clear
clc
scale = 5.3e-3;
base = .159;
% Calibrate the scale parameter and wheel track of the robot
addpath("../simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor_square.jpg");
% Start by placing your robot at the start of the line
new_state = [1;1;0];
pb.place([1;1],0);
% Create a window to visualise the robot camera
figure;
camAxes = axes();
% Follow the line in a loop
while true
     tic;
    img = pb.getImage();
    img = img(end-50:end, 1:end/3);
    gray_img = im2gray(img);
    bin_img = ~imbinarize(gray_img, 0.25);
     % Check the dot is visible anywhere
    if ~any(bin_img)
        break
    end
    % Find the centre of mass of the dot pixels
    [r, c] = find(bin_img == 1);
    centre_of_mass = [mean(c), mean(r)];
    centre_of_mass = (centre_of_mass - [200, 100]) ./ [200, 100];
    % Find the centre of the line to follow
    line_centre = centre_of_mass; % replace with correct value
    % If you have reached the end of the line, you need to stop by breaking
    % the loop.
    end_of_line = false;
    if (end_of_line)
        break;
    end
    % If x is negative, spin left. If x is positive, spin right
    q = -0.5*centre_of_mass(1);
    % Drive forward as soon as the dot is roughly in view
    if abs(centre_of_mass(1)) > 0.1
        u = 0.0;
    else
        u = .25;
    end
    [wl,wr] = inverse_kinematics(u,q);
    s = 5;
    t = 10;
    if abs(wl) < s && wl < 0
        wl = -t;
    elseif abs(wl) < s && wl > 0
        wl = t;
    end
    
    if abs(wr) < s && wr < 0
        wr = -t;
    elseif abs(wr) < s && wr > 0
        wr = t;
    end 
    pb.setVelocity(wl,wr);
     dt = toc;
     a = scale*(wl+wr)/2;
     b = (scale/base)*(wr-wl);
     A = pb.measure;
     z1 = A(1);
     z2 = A(2);
     z3 = new_state(3);
     new_state = [z1; z2; z3];
     new_state = integrate_kinematics(new_state, dt, a, b);
     x = new_state(1);
    y = new_state(2);
     plot(x,y,'r*', 'MarkerSize',5);     
    hold on     
     figure(2);     
     f = getframe;
end
hold off
pb.saveTrail();



