clear; clc; close all;

% ==========================================
% CONFIGURACIÓN GENERAL
% ==========================================
etiquetas_x    = {'Case 1', 'Case 2', 'Case 3', 'Case 4'};
leyendas_pistas = {'1 Pista', '2 Pistas', '3 Pistas'};

colores = [0.2157 0.4706 0.7490;   % azul
           0.3020 0.6863 0.2902;   % verde
           0.8941 0.1020 0.1098];  % rojo

font_size_label  = 13;
font_size_title  = 14;
font_size_legend = 12;
font_size_tick   = 11;

% Datos
matriz_costos = [3710, 210, 0; 2830, 60, 0; 3974, 0, 0; 12870.34, 997.83, 120.60];
matriz_tiempos = [60.00, 60.00, 25.54; 60.00, 60.00, 49.52; 60.00, 21.10, 48.14; 60.00, 60.00, 60.03];
matriz_nodos = [175186, 59686, 21018; 77961, 40876, 28579; 2066485, 14596, 25074; 183388, 19284, 8083];
estado_optimo = [0, 0, 1; 0, 0, 1; 0, 1, 1; 0, 0, 0];

% ==========================================
% GRÁFICO 1: Costo Total
% ==========================================
figure('Name', 'Costo Total', 'Color', 'w', 'Position', [100 500 700 420]);
b1 = bar(matriz_costos);
for k = 1:3, b1(k).FaceColor = colores(k,:); b1(k).EdgeColor = 'none'; end
ax1 = gca; ax1.Color = 'w'; ax1.XColor = 'k'; ax1.YColor = 'k';
ax1.XTickLabel = etiquetas_x; ax1.YGrid = 'on';
title('Costo Total de Penalizaciones', 'FontSize', font_size_title, 'Color', 'k');

lg1 = legend(leyendas_pistas, 'Location', 'northeast');
lg1.FontSize = font_size_legend;
lg1.TextColor = 'w';         
lg1.Color = [0.2 0.2 0.2];    

% ==========================================
% GRÁFICO 2: Tiempo de CPU
% ==========================================
figure('Name', 'Tiempo de CPU', 'Color', 'w', 'Position', [820 500 700 420]);
b2 = bar(matriz_tiempos);
for k = 1:3, b2(k).FaceColor = colores(k,:); b2(k).EdgeColor = 'none'; end
ax2 = gca; ax2.Color = 'w'; ax2.XColor = 'k'; ax2.YColor = 'k';
ax2.XTickLabel = etiquetas_x; ax2.YGrid = 'on'; ylim([0 68]);
title('Tiempo de CPU por Caso', 'FontSize', font_size_title, 'Color', 'k');

lg2 = legend(leyendas_pistas, 'Location', 'northeast');
lg2.TextColor = 'w';          % <--- TEXTO BLANCO
lg2.Color = [0.2 0.2 0.2];    % <--- FONDO OSCURO
yline(60, 'r--', 'Timeout', 'LineWidth', 1.8, 'Color', [0.8 0 0]);

% ==========================================
% GRÁFICO 3: Nodos Explorados
% ==========================================
figure('Name', 'Nodos Explorados', 'Color', 'w', 'Position', [100 50 700 420]);
b3 = bar(matriz_nodos);
for k = 1:3, b3(k).FaceColor = colores(k,:); b3(k).EdgeColor = 'none'; end
ax3 = gca; ax3.Color = 'w'; ax3.XColor = 'k'; ax3.YColor = 'k';
ax3.YScale = 'log'; ax3.XTickLabel = etiquetas_x; ax3.YGrid = 'on';
title('Nodos Explorados (Log)', 'FontSize', font_size_title, 'Color', 'k');

lg3 = legend(leyendas_pistas, 'Location', 'northeast');
lg3.TextColor = 'w';         
lg3.Color = [0.2 0.2 0.2];   

% ==========================================
% GRÁFICO 4: Reducción Porcentual
% ==========================================
reduccion_1a2 = (matriz_costos(:,1) - matriz_costos(:,2)) ./ max(matriz_costos(:,1), 1) * 100;
reduccion_2a3 = (matriz_costos(:,2) - matriz_costos(:,3)) ./ max(matriz_costos(:,2), 1) * 100;
figura4_data = [reduccion_1a2, reduccion_2a3];
figure('Name', 'Reducción de Costo', 'Color', 'w', 'Position', [820 50 700 420]);
colores_reduc = [0.2157 0.4706 0.7490; 0.8941 0.1020 0.1098];
b4 = bar(figura4_data);
for k = 1:2, b4(k).FaceColor = colores_reduc(k,:); b4(k).EdgeColor = 'none'; end
ax4 = gca; ax4.Color = 'w'; ax4.XColor = 'k'; ax4.YColor = 'k';
ax4.XTickLabel = etiquetas_x; ax4.YGrid = 'on'; ylim([0 115]);
title('Reducción Porcentual de Costo', 'FontSize', font_size_title, 'Color', 'k');

lg4 = legend({'1 -> 2 Pistas', '2 -> 3 Pistas'}, 'Location', 'southeast');
lg4.TextColor = 'w';         
lg4.Color = [0.2 0.2 0.2];   

hold on;
offsets4 = [-0.15, 0.15];
for c = 1:n_casos
    for p = 1:2
        xpos = c + offsets4(p); yval = figura4_data(c,p);
        if yval > 1
            text(xpos, yval + 1.5, sprintf('%.1f%%', yval), 'FontSize', 8, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', 'k');
        end
    end
end
hold off;