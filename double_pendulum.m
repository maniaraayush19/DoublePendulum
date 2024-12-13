function double_pendulum_model
    % Parameters
    l1 = 1.5; l2 = 1; % Lengths (m)
    m1 = 1, m2 = 0.5; % Masses (kg)
    g = 9.81; % Gravitational acceleration (m/s^2)
    theta1 = pi/4; %setting angles for the masses to the normal line
    theta2 = pi/4; 
    omega1 = 0; %setting initial angular velocities
    omega2 = 0;
    initial_conditions = [theta1, theta2, omega1,omega2];
    tspan = [0 60]; % time range of simulation 
    
    % Solve the system
    [t, y] = ode45(@(t, y) equations_of_motion(t, y, l1, l2, m1, m2, g), tspan, initial_conditions);
    %ode45 helps to solve system of differential equations. 
    % equation of motion (__,___,___) is function, tspan is second parameter (time), and initial_conditions is last parameter
  
   
    % Visualize the results
    animate_double_pendulum_with_trajectory(t, y, l1, l2);
end

function dydt = equations_of_motion(~, y, l1, l2, m1, m2, g) % represents function of motion
    % Unpack variables
    theta1 = y(1); omega1 = y(2); theta2 = y(3); omega2 = y(4);
    delta = theta2 - theta1;

    % Equations of motion (understand these)
    den1 = l1 * (2*m1 + m2 - m2*cos(2*delta));
    dydt(1) = omega1;
    dydt(2) = (-g*(2*m1 + m2)*sin(theta1) - m2*g*sin(theta1-2*theta2) ...
              - 2*sin(delta)*m2*(omega2^2*l2 + omega1^2*l1*cos(delta))) / den1;

    den2 = l2 * (2*m1 + m2 - m2*cos(2*delta));
    dydt(3) = omega2;
    dydt(4) = (2*sin(delta)*(omega1^2*l1*(m1+m2) + g*(m1+m2)*cos(theta1) ...
              + omega2^2*l2*m2*cos(delta))) / den2;

    dydt = dydt(:); % Return column vector
end

function animate_double_pendulum(t, y, l1, l2) %this function is not getting called
    theta1 = y(:, 1);
    theta2 = y(:, 3);
    x1 = l1 * sin(theta1);
    y1 = -l1 * cos(theta1);
    x2 = x1 + l2 * sin(theta2);
    y2 = y1 - l2 * cos(theta2);

    figure;
    for i = 1:length(t)
        plot([0 x1(i) x2(i)], [0 y1(i) y2(i)], '-o', 'LineWidth', 2);
        axis equal;
        axis([-l1-l2 l1+l2 -l1-l2 l1+l2]);
        pause(0.05);
    end
end

function animate_double_pendulum_with_trajectory(t, y, l1, l2)
    % Extract angles and compute positions of the pendulum masses
    theta1 = y(:, 1);
    theta2 = y(:, 3);
    x1 = l1 * sin(theta1);
    y1 = -l1 * cos(theta1);
    x2 = x1 + l2 * sin(theta2);
    y2 = y1 - l2 * cos(theta2);

    % Initialize the figure
    figure;
    hold on;
    
    % Initialize pendulum rods and masses
    h_pendulum = plot([0, x1(1), x2(1)], [0, y1(1), y2(1)], '-o', 'LineWidth', 2); 
    % Initialize trajectory as a dotted line
    h_trajectory = plot(x2(1), y2(1), ':r', 'LineWidth', 1.5); 
    h_trajectory2 = plot(x1(1), y1(1), ':b', 'LineWidth', 1.5); 

    axis equal;
    axis([-l1-l2, l1+l2, -l1-l2, l1+l2]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('Double Pendulum Animation with Trajectory');

    % Speed control: time scaling factor
    playback_speed = 0.05; % Adjust this value to slow down or speed up

    % Animation loop
    for i = 1:length(t)
        % Update pendulum positions
        set(h_pendulum, 'XData', [0, x1(i), x2(i)], 'YData', [0, y1(i), y2(i)]);
        
        % Update trajectory (all past positions of m2 up to current time)
        set(h_trajectory, 'XData', x2(1:i), 'YData', y2(1:i)); 
        set(h_trajectory2, 'XData', x1(1:i), 'YData', y1(1:i)); 
        
        % Refresh the plot
        drawnow;

        % Pause to control playback speed
        if i > 1
            pause(playback_speed * (t(i) - t(i-1))); % Scales with the time step
        end
    end
end
