% Parâmetros do robô
L1 = 2; % Comprimento do primeiro elo
L2 = 1.5; % Comprimento do segundo elo

% Definir os vértices do quadrado dentro da área de trabalho do robô
square_vertices = [1 1.5; 1.5 1.5; 1.5 1; 1 1; 1 1.5];

% Número de pontos para a trajetória do quadrado
num_points = 100;
x_square = interp1(1:5, square_vertices(:,1), linspace(1, 5, num_points));
y_square = interp1(1:5, square_vertices(:,2), linspace(1, 5, num_points));

% Inicialização das coordenadas dos elos e do efetuador
x_end_effector = zeros(1, num_points);
y_end_effector = zeros(1, num_points);

% Inicialização das variáveis para posição, velocidade e aceleração
theta_t = zeros(1, num_points);
theta_dot_t = zeros(1, num_points);
theta_dot_dot_t = zeros(1, num_points);

% Plot dos gráficos
figure;

for i = 1:num_points
    % Coordenadas do efetuador para desenhar o quadrado
    x_end_effector(i) = x_square(i);
    y_end_effector(i) = y_square(i);
    
    % Verificar se o efetuador está dentro da área de trabalho
    inside_workspace = norm([x_end_effector(i), y_end_effector(i)]) <= L1 + L2;
    
    if inside_workspace
        % Cinemática inv para encontrar as coordenadas dos elos
        x_end_effector_temp = x_end_effector(i);
        y_end_effector_temp = y_end_effector(i);
        D = (x_end_effector_temp^2 + y_end_effector_temp^2 - L1^2 - L2^2) / (2 * L1 * L2);
        theta2 = real(acos(D)); % Garantindo que o ângulo seja real
        theta1 = real(atan2(y_end_effector_temp, x_end_effector_temp) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2)));

        x_link1 = L1 * cos(theta1);
        y_link1 = L1 * sin(theta1);

        x_link2 = x_link1 + L2 * cos(theta1 + theta2);
        y_link2 = y_link1 + L2 * sin(theta1 + theta2);

        % Desenhar o robô e a trajetória do quadrado
        subplot(2, 2, 1);
        plot(x_square, y_square, 'r--', 'LineWidth', 1.5);
        hold on;
        plot([0, x_link1], [0, y_link1], 'b-', 'LineWidth', 2);
        plot([x_link1, x_link2], [y_link1, y_link2], 'g-', 'LineWidth', 2);
        plot(x_end_effector(1:i), y_end_effector(1:i), 'k.', 'MarkerSize', 10);
        grid on;
        axis equal;
        xlim([min(x_square) - 3, max(x_square) + L1 + L2 + 1]);
        ylim([min(y_square) - 3, max(y_square) + L1 + L2 + 1]);
        title('Robô e trajetória do quadrado');
        xlabel('X');
        ylabel('Y');
        legend('Trajetória do quadrado', 'Elo 1', 'Elo 2', 'Efetuador');
        hold off;

        % Cálculo dos ângulos de partida e chegada para cada segmento do quadrado
        if i < num_points
            theta0 = atan2(y_square(i+1) - y_square(i), x_square(i+1) - x_square(i));
            thetaf = atan2(y_square(i+1) - y_square(i), x_square(i+1) - x_square(i));
        else
            theta0 = atan2(y_square(1) - y_square(num_points), x_square(1) - x_square(num_points));
            thetaf = atan2(y_square(1) - y_square(num_points), x_square(1) - x_square(num_points));
        end

        % Aplicar cinemática direta para calcular as coordenadas dos elos
        x1 = L1 * cos(theta0);
        y1 = L1 * sin(theta0);
        x2 = x1 + L2 * cos(theta0);
        y2 = y1 + L2 * sin(theta0);
        x3 = L1 * cos(thetaf);
        y3 = L1 * sin(thetaf);
        x4 = x3 + L2 * cos(thetaf);
        y4 = y3 + L2 * sin(thetaf);

        % Exibir os ângulos de partida e chegada para cada segmento do quadrado
        disp(['Segmento ', num2str(i), ': theta0 = ', num2str(theta0), ', thetaf = ', num2str(thetaf)]);

        % Cálculo da posição, velocidade e aceleração
        tf = 10; % Tempo total para percorrer cada segmento do quadrado (segundos)
        t = linspace(0, tf, num_points);
        a0 = theta0;
        a1 = 0;
        a2 = (3/tf^2) * (thetaf - theta0);
        a3 = -(2/tf^3) * (thetaf - theta0);

        % Calcular theta(t), theta_dot(t) e theta_dot_dot(t)
        theta_t(i) = a0 + a1 * t(i) + a2 * t(i)^2 + a3 * t(i)^3;
        theta_dot_t(i) = a1 + 2 * a2 * t(i) + 3 * a3 * t(i)^2;
        theta_dot_dot_t(i) = 2 * a2 + 6 * a3 * t(i);

        % Gráfico de posição
        subplot(2, 2, 2);
        plot(t(1:i), theta_t(1:i), 'b-', 'LineWidth', 1.5);
        grid on;
        title('Posição vs Tempo');
        xlabel('Tempo (s)');
        ylabel('Posição (rad)');

        % Gráfico de velocidade
        subplot(2, 2, 3);
        plot(t(1:i), theta_dot_t(1:i), 'r-', 'LineWidth', 1.5);
        hold on;
        grid on;
        title('Velocidade vs Tempo');
        xlabel('Tempo (s)');
        ylabel('Velocidade (rad/s)');

        % Gráfico de aceleração
        subplot(2, 2, 4);
        plot(t(1:i), theta_dot_dot_t(1:i), 'g-', 'LineWidth', 1.5);
        hold on;
        grid on;
        title('Aceleração vs Tempo');
        xlabel('Tempo (s)');
        ylabel('Aceleração (rad/s^2)');

        pause(0.05); % Pausa para exibição em tempo realelse
    else
        % Fechar a figura se o efetuador sair da área de trabalho
        close(gcf);
        disp('O efetuador saiu da área de trabalho!');
        break;
    end     
end
