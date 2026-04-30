clear; clc; close all;

% ==========================================
% CONFIGURACIÓN GENERAL
% ==========================================
etiquetas_x    = {'Case 1', 'Case 2', 'Case 3', 'Case 4'};
leyendas_pistas = {'1 Pista', '2 Pistas', '3 Pistas'};
n_casos = length(etiquetas_x);

colores = [0.2157 0.4706 0.7490;   % azul
           0.3020 0.6863 0.2902;   % verde
           0.8941 0.1020 0.1098];  % rojo

font_size_label  = 13;
font_size_title  = 14;
font_size_legend = 12;
font_size_tick   = 11;

% ==========================================
% DATOS EXPERIMENTALES (MFC + Branch & Bound)
% ==========================================
% Usamos NaN para los casos "Infactibles" o "Colgados", 
% así MATLAB no grafica barras engañosas en cero.

% Costos Totales
matriz_costos = [
    NaN, 154690.00, 154480.00; 
    NaN, 177150.00, 177090.00; 
    NaN,   7750.00,   7750.00; 
    NaN,       NaN,       NaN
];

% Tiempo de CPU (s)
% Al caso 4 (2 y 3 pistas) le asignamos 600s para representar visualmente el Timeout.
matriz_tiempos = [
    0.2548, 2.2351, 0.0115; 
    0.5519, 0.1753, 0.0232; 
    0.0057, 0.0798, 0.1188; 
    486.7286,  600,    600 
];

% Nodos Explorados
matriz_nodos = [
    426,  5615, 16; 
    849,   293, 21; 
      2,    45, 45; 
   9614,   NaN, NaN
];

% ==========================================
% GRÁFICO 1: Costo Total
% ==========================================
figure('Name', 'Costo Total', 'Color', 'w', 'Position', [100 500 700 420]);
b1 = bar(matriz_costos);
for k = 1:3, b1(k).FaceColor = colores(k,:); b1(k).EdgeColor = 'none'; end
ax1 = gca; ax1.Color = 'w'; ax1.XColor = 'k'; ax1.YColor = 'k';
ax1.XTickLabel = etiquetas_x; ax1.YGrid = 'on';
title('Costo Total de Penalizaciones', 'FontSize', font_size_title, 'Color', 'k');
ylabel('Penalización Total');

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
ax2.XTickLabel = etiquetas_x; ax2.YGrid = 'on'; 
ylim([0 650]); % Ajustado para poder ver el Timeout visual
title('Tiempo de CPU por Caso', 'FontSize', font_size_title, 'Color', 'k');
ylabel('Segundos');

lg2 = legend(leyendas_pistas, 'Location', 'northwest');
lg2.TextColor = 'w';          
lg2.Color = [0.2 0.2 0.2];    
yline(600, 'r--', 'Timeout / Colgado', 'LineWidth', 1.8, 'Color', [0.8 0 0], 'LabelHorizontalAlignment','left');

% ==========================================
% GRÁFICO 3: Nodos Explorados
% ==========================================
figure('Name', 'Nodos Explorados', 'Color', 'w', 'Position', [100 50 700 420]);
b3 = bar(matriz_nodos);
for k = 1:3, b3(k).FaceColor = colores(k,:); b3(k).EdgeColor = 'none'; end
ax3 = gca; ax3.Color = 'w'; ax3.XColor = 'k'; ax3.YColor = 'k';
ax3.YScale = 'log'; ax3.XTickLabel = etiquetas_x; ax3.YGrid = 'on';
title('Nodos Explorados (Log)', 'FontSize', font_size_title, 'Color', 'k');
ylabel('Cantidad de Nodos');

lg3 = legend(leyendas_pistas, 'Location', 'northeast');
lg3.TextColor = 'w';         
lg3.Color = [0.2 0.2 0.2];   

% ==========================================
% GRÁFICO 4: Reducción Porcentual
% ==========================================
% Como 1 pista es siempre infactible, la reducción de 1->2 no existe estadísticamente.
reduccion_1a2 = zeros(n_casos, 1);
reduccion_2a3 = zeros(n_casos, 1);

for i = 1:n_casos
    if ~isnan(matriz_costos(i,2)) && ~isnan(matriz_costos(i,3)) && matriz_costos(i,2) > 0
        reduccion_2a3(i) = (matriz_costos(i,2) - matriz_costos(i,3)) / matriz_costos(i,2) * 100;
    end
end

figura4_data = [reduccion_1a2, reduccion_2a3];

figure('Name', 'Reducción de Costo', 'Color', 'w', 'Position', [820 50 700 420]);
colores_reduc = [0.2157 0.4706 0.7490; 0.8941 0.1020 0.1098];
b4 = bar(figura4_data);
for k = 1:2, b4(k).FaceColor = colores_reduc(k,:); b4(k).EdgeColor = 'none'; end
ax4 = gca; ax4.Color = 'w'; ax4.XColor = 'k'; ax4.YColor = 'k';
ax4.XTickLabel = etiquetas_x; ax4.YGrid = 'on'; 
ylim([0 2]); % Eje pequeño porque las mejoras son de decimales (0.13%, 0.03%)
title('Reducción Porcentual de Costo', 'FontSize', font_size_title, 'Color', 'k');
ylabel('Porcentaje (%)');

lg4 = legend({'1 -> 2 Pistas (N/A)', '2 -> 3 Pistas'}, 'Location', 'northeast');
lg4.TextColor = 'w';         
lg4.Color = [0.2 0.2 0.2];   

hold on;
offsets4 = [-0.15, 0.15];
for c = 1:n_casos
    for p = 1:2
        xpos = c + offsets4(p); yval = figura4_data(c,p);
        if yval > 0
            text(xpos, yval + 0.1, sprintf('%.2f%%', yval), 'FontSize', 9, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', 'k');
        end
    end
end
hold off;