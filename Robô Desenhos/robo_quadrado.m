% Par�metros do rob�
L1 = 2; % Comprimento do primeiro elo
L2 = 1.5; % Comprimento do segundo elo

% Definir os v�rtices do quadrado dentro da �rea de trabalho do rob�
square_vertices = [1 1.5; 1.5 1.5; 1.5 1; 1 1; 1 1.5];

% N�mero de pontos para a trajet�ria do quadrado
num_points = 100;
x_square = interp1(1:5, square_vertices(:,1), linspace(1, 5, num_points));
y_square = interp1(1:5, square_vertices(:,2), linspace(1, 5, num_points));

% Inicializa��o das coordenadas dos elos e do efetuador
x_end_effector = zeros(1, num_points);
y_end_effector = zeros(1, num_points);

% Plot do rob� e da trajet�ria do quadrado
figure;

for i = 1:num_points
    % Coordenadas do efetuador para desenhar o quadrado
    x_end_effector(i) = x_square(i);
    y_end_effector(i) = y_square(i);
    
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
    
    % Desenhar o rob� e a trajet�ria do quadrado
    plot(x_square, y_square, 'r--', 'LineWidth', 1.5);
    hold on;
    plot([0, x_link1], [0, y_link1], 'b-', 'LineWidth', 2);
    plot([x_link1, x_link2], [y_link1, y_link2], 'g-', 'LineWidth', 2);
    plot(x_end_effector(1:i), y_end_effector(1:i), 'k.', 'MarkerSize', 10);
    grid on;
    axis equal;
    xlim([min(x_square) - 3, max(x_square) + L1 + L2 + 1]);
    ylim([min(y_square) - 3, max(y_square) + L1 + L2 + 1]);
    title('Rob� planar com dois elos desenhando um quadrado');
    xlabel('X');
    ylabel('Y');
    legend('Trajet�ria do quadrado', 'Elo 1', 'Elo 2', 'Efetuador');
    hold off;
    pause(0.05); % Pausa para exibi��o em tempo real
end
