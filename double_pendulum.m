function double_pendulum_model
    % Parameters
    l1 = 1.5; % Lengths (m)
    l2 = 0.5; 
    m1 = 1; % Masses (kg)
    m2 = 0.5; 
    g = 9.81; % Gravitational acceleration (m/s^2)
    theta1 = pi/3; %setting angles for the masses to the normal line
    theta2 = pi/3; 
    omega1 = 0; %setting initial angular velocities
    omega2 = 0;
    initial_conditions = [theta1, theta2, omega1,omega2];
    tspan = [0 45]; % time range of simulation 
    
    % Solve the system
    [t, y] = ode45(@(t, y) equations_of_motion(t, y, l1, l2, m1, m2, g), tspan, initial_conditions);
    %ode45 helps to solve system of differential equations. 
    % equation of motion (__,___,___) is function, tspan is second parameter (time), and initial_conditions is last parameter
  
   
    % Visualize the results
    animate_double_pendulum_with_trajectory(t, y, l1, l2);

    %solve_system()




   
end

function dydt = equations_of_motion(~, y, l1, l2, m1, m2, g) % represents function of motion
    % assign variables
    theta1 = y(1); 
    omega1 = y(2); 
    theta2 = y(3); 
    omega2 = y(4);
    delta = theta2 - theta1;

    % Equations of motion (understand these)
    dydt(1) = omega1;
    dydt(2) = -(g*m1*sin(theta1) + g*m2*sin(theta1) + l2*m2*omega2^2*sin(theta1 - theta2) - g*m2*cos(theta1 - theta2)*sin(theta2) + ...
        l1*m2*omega1^2*cos(theta1 - theta2)*sin(theta1 - theta2))/(l1*(m1 + m2 - m2*cos(theta1 - theta2)^2));



    %our way 
    % -(g*m1*sin(theta1) + g*m2*sin(theta1) + l2*m2*omega2^2*sin(theta1 - theta2) - g*m2*cos(theta1 - theta2)*sin(theta2) + ...
       % l1*m2*omega1^2*cos(theta1 - theta2)*sin(theta1 - theta2))/(l1*(m1 + m2 - m2*cos(theta1 - theta2)^2));
    
    %chat gpt's way 
    % (-g*(2*m1 + m2)*sin(theta1) - m2*g*sin(theta1-2*theta2) ...
              %- 2*sin(delta)*m2*(omega2^2*l2 + omega1^2*l1*cos(delta))) / (l1 * (2*m1 + m2 - m2*cos(2*delta)));

    dydt(3) = omega2;
    dydt(4) = (l1*m1*omega1^2*sin(theta1 - theta2) - g*m2*sin(theta2) - g*m1*sin(theta2) + l1*m2*omega1^2*sin(theta1 - theta2) + ...
        g*m1*cos(theta1 - theta2)*sin(theta1) + g*m2*cos(theta1 - theta2)*sin(theta1) + l2*m2*omega2^2*cos(theta1 - theta2)*sin(theta1 - theta2))/(l2*(m1 + m2 - m2*cos(theta1 - theta2)^2));

    
    %our way
    % (l1*m1*omega1^2*sin(theta1 - theta2) - g*m2*sin(theta2) - g*m1*sin(theta2) + l1*m2*omega1^2*sin(theta1 - theta2) + ...
       % g*m1*cos(theta1 - theta2)*sin(theta1) + g*m2*cos(theta1 - theta2)*sin(theta1) + l2*m2*omega2^2*cos(theta1 - theta2)*sin(theta1 - theta2))/(l2*(m1 + m2 - m2*cos(theta1 - theta2)^2));

    %chat gpt's way
    %(2*sin(delta)*(omega1^2*l1*(m1+m2) + g*(m1+m2)*cos(theta1) ...
     %         + omega2^2*l2*m2*cos(delta))) / (l2 * (2*m1 + m2 - m2*cos(2*delta)));

    dydt = dydt(:); % Return column vector
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

%used to solve for angular accelerations
function solve_system()
   % System of Equations Solver in MATLAB

   % Define the system symbolically
   syms x y m1 m2 l1 omega1 l2 omega2 theta1 theta2 g
    
    % Example equations with symbolic coefficients (modify as needed):
    % a*x + b*y + c*z = d
    % e*x + f*y + g*z = h
    eq1 = ((m1+m2)*l1)*x + (m2*l2*cos(theta2-theta1))*y  == (m2*l2*(omega2)^2*sin(theta2-theta1))-((m1+m2)*g*sin(theta1));
    eq2 = (l1*cos(theta2-theta1))*x + (l2)*y == -(l1*(omega1^2)*sin(theta2-theta1))-(g*sin(theta2));
    
    % Combine equations into a system
    system = [eq1, eq2];
    
    % Solve the system symbolically
    solution = solve(system, [x, y]);
    
    % Display symbolic solutions
    disp('Symbolic Solution:');
    disp(solution);
    % Calculate the halfway point

    fieldValues = struct2cell(solution);

    firstValue = fieldValues{1};
        % Convert to string if necessary
    firstValue = string(firstValue);
    
    % Split into two parts
    midpoint = ceil(strlength(firstValue) / 2);
    parta = extractBetween(firstValue, 1, midpoint);
    partb = extractBetween(firstValue, midpoint+1, strlength(firstValue));
    
    disp(parta); % '10,20'
    disp(partb); % '30,40'

    secondValue = fieldValues{2};
        % Convert to string if necessary
    secondValue = string(secondValue);
    
    % Split into two parts
    midpoint = ceil(strlength(secondValue) / 2);
    part1 = extractBetween(secondValue, 1, midpoint);
    part2 = extractBetween(secondValue, midpoint+1, strlength(secondValue));
    
    disp(part1); % '10,20'
    disp(part2); % '30,40'
    
    
end