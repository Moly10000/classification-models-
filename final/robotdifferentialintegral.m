function pose = robotdifferentialintegral(state, t)

    R = 0.034; % Radio en metros de mi robot
    L = 0.21;

    goal_x = -10;
    goal_y = -3;

Kp_linear = 2;

Kp_angular = 2;
Ki_angular = 3;

x = state(1);
y = state(2);
theta = state(3);

pose = zeros(4, 1);

v_desired = 1;
integral_error = 0;

min_integral = -1;  % Ajusta según sea necesario
max_integral = 1;   % Ajusta según sea necesario
tolerancia = 0.1;   % Ajusta según sea necesario

% calulo errores de posicion


    %Calculo de los errores posicionales X, Y, Phi
    x_error = goal_x - x;
    y_error = goal_y - y;
    phi_error = atan2(goal_y - y, goal_x - x);

    %Calculo de los errores de velocidad
    error_linear = sqrt((x_error)^2 + (y_error)^2);
    error_angular = phi_error - theta;

    v_control = Kp_linear * error_linear;

    w_control = Kp_angular * error_angular + Ki_angular * integral_error;
    integral_error = integral_error + error_angular * (.01);
    integral_error = max(min_integral, min(max_integral, integral_error));

    %Cinemática inversa
    V = [v_desired; w_control];
    J = [R/2 R/2; R/L -R/L];
    W = inv(J) * V;
    wR = W(1);
    wL = W(2);

    %Funciones de transferencia de los motores
    TransferL = tf(12.533, [.17525 1]); % Dinamica del motor izquiero
    TransferR = tf(12.533, [.10025 1]); % Dinamica del motor derecho

    %Obtener los coeficientes numericos de la dinamica
    Gs_L = [num_L, den_L] = tfdata(TransferL, 'v');
    Gs_R = [num_R, den_R] = tfdata(TransferR, 'v');

    %Cinemática directa
    v = R/2 * (wR + wL); %linear velocity
    w = R/L * (wR - wL); %angular velocity

    %Funciones de transferencia de los motores
    V_L = v + R/2 * Gs_L * wL;
    V_R = v + R/2 * Gs_R * wR;

    %Cinemática inversa con la dinamica
    W_L = (V_L - v) / (R/2 * Gs_L);
    W_R = (V_R - v) / (R/2 * Gs_R);

    %Actualizando V y W del robot con los valores reales de las ruedas..
    Dv = R/2 * (W_R + W_L);
    Dw = R/L * (W_R - W_L);

    %Matriz de posicion con el valor real del robot, integrando cinematica directa, inversa y dinamica.
    dt = .01;
    thetadot = Dw;
    xdot = Dv * cos(theta);
    ydot = Dv * sin(theta);
    pose = [xdot; ydot; thetadot];
end
