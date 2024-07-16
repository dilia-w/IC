% Par�metros do rob�
L1 = 2; % Comprimento do primeiro elo
L2 = 1.5; % Comprimento do segundo elo

% Definir centro e raio do c�rculo
circle_center_x = 2.5;
circle_center_y = 1;
circle_radius = 1;

% N�mero de pontos para a trajet�ria do cardioid
num_points = 100;
theta_cardioid = linspace(0, 2*pi, num_points);

% Inicializa��o das coordenadas dos elos e do efetuador
x_end_effector = zeros(1, num_points);
y_end_effector = zeros(1, num_points);

% Calcular coordenadas do cardioid com raio menor
cardioid_radius = circle_radius / 2;
x_cardioid = (2 * cardioid_radius * cos(theta_cardioid)) .* (1 - cos(theta_cardioid)) + circle_center_x;
y_cardioid = (2 * cardioid_radius * sin(theta_cardioid)) .* (1 - cos(theta_cardioid)) + circle_center_y;

% Plot do rob� e da trajet�ria do cardioid
figure;

for i = 1:num_points
    % Coordenadas do efetuador para desenhar o cardioid
    x_end_effector(i) = x_cardioid(i);
    y_end_effector(i) = y_cardioid(i);
    
    % Cinem�tica inv para encontrar as coordenadas dos elos
    x_end_effector_temp = x_end_effector(i);
    y_end_effector_temp = y_end_effector(i);
    D = (x_end_effector_temp^2 + y_end_effector_temp^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2 = real(acos(D)); % Garantindo que o �ngulo seja real
    theta1 = real(atan2(y_end_effector_temp, x_end_effector_temp) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)));
    
    x_link1 = L1 * cos(theta1);
    y_link1 = L1 * sin(theta1);
    
    x_link2 = x_link1 + L2 * cos(theta1 + theta2);
    y_link2 = y_link1 + L2 * sin(theta1 + theta2);
    
    % Desenhar o rob� e a trajet�ria do cardioid
    plot(x_cardioid, y_cardioid, 'r--', 'LineWidth', 1.5);
    hold on;
    plot([0, x_link1], [0, y_link1], 'b-', 'LineWidth', 2);
    plot([x_link1, x_link2], [y_link1, y_link2], 'g-', 'LineWidth', 2);
    plot(x_end_effector(1:i), y_end_effector(1:i), 'k.', 'MarkerSize', 10);
    grid on;
    axis equal;
    xlim([circle_center_x - circle_radius - 2, circle_center_x + circle_radius + L1 + L2 - 1]);
    ylim([circle_center_y - circle_radius - 3, circle_center_y + circle_radius + L1 + L2 - 1]);
    title('Rob� planar com dois elos desenhando um cardioid menor');
    xlabel('X');
    ylabel('Y');
    legend('Trajet�ria do cardioid', 'Elo 1', 'Elo 2', 'Efetuador');
    hold off;
    pause(0.05); % Pausa para exibi��o em tempo real
end
