% DYNAMISM OF THE ROBOT

% Constantes
DIAM_ROBOT = 54; %mm
RADIUS_WHEEL = 20.7; %mm

PERIOD_REGULATOR = 0.2; %s  
PERIOD_MEASUREMENT = 0.0125; %s
Y_DETECTION = 50; %mm
OFFSET = 25;

MOTOR_SPEED_LIMIT_MARGIN = 1000; % [step/s]
MOTOR_SPEED_LIMIT_MARGIN_RAD_S = 2*pi; % [rad/s]
MOTOR_SPEED_LIMIT_MARGIN_MM_S = 130; % [mm/s]
MOTOR_SPEED_CROIS_RAD_S = 2*pi;
R_ROT_ROB_MIN = (DIAM_ROBOT/2);
MAX_ANGLE_ROT = (MOTOR_SPEED_CROIS_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR) / (R_ROT_ROB_MIN + (DIAM_ROBOT/2));

KP = MAX_ANGLE_ROT / (Y_DETECTION - OFFSET);
KI = 0;
KD = 0;

% Conditions initiales
y = - Y_DETECTION + OFFSET; %mm 
alpha = 0; % [rad] angle avec lequel on arrive contre le mur

% Début programme

for k = 1:30
    beta = (KP * y);
    gamma = beta - alpha; % l'angle que l'on souhaite tourner avec le robot
    % sécurité (saturation de gama)
    if gamma > MAX_ANGLE_ROT 
        gamma = MAX_ANGLE_ROT;
    end
    if gamma < -MAX_ANGLE_ROT 
        gamma = -MAX_ANGLE_ROT;
    end
    if gamma >= 0
        rRotRob = ((MOTOR_SPEED_CROIS_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR) / (gamma)) - (DIAM_ROBOT/2);
    else 
        rRotRob = ((MOTOR_SPEED_CROIS_RAD_S * RADIUS_WHEEL * PERIOD_REGULATOR) / (gamma)) + (DIAM_ROBOT/2);
    end
    
    % enregistrement des mesures pour le plot
    gamma_m(k) = gamma;
    alpha_m(k) = alpha;
    beta_m(k) = beta;
    y_m(k) = y - OFFSET;
    
    % valeurs futures
    y = y + rRotRob * (- cos(alpha) + cos(beta)) + sin(beta) * 4 * (MOTOR_SPEED_CROIS_RAD_S * RADIUS_WHEEL * PERIOD_MEASUREMENT);
    alpha = alpha + gamma;
    
end


%% plot angles
figure(1)
plot([gamma_m' alpha_m' beta_m']);
title('Angles without input angle')
xlabel('Time [s]') 
ylabel('Angles values [rad]')
legend('gamma','alpha','beta');

%% plot y
figure(2)
plot(y_m');
title('Distance Robot-Wall without input angle')
xlabel('Time [s]') 
ylabel('y_{robot} [mm]') 
legend('y_{robot}');

