%% 01_GenerarWorkspace
% Generacion de workspace alcanzable de una pata de robot hexapodo

clear; clc; close all;

%% ============================================================
% 1. PARAMETROS DEL ROBOT
% =============================================================

L1 = 49.25;     % Coxa
L2 = 101.63;    % Femur
L3 = 127.92;    % Tibia

% Limites articulares (grados)
q1_min = -28;
q1_max = 28;

q2_min = -90;
q2_max = 90;

q3_min = -90;
% Se eligió 0 por considerar que esta extremidad siempre mira hacia abajo
q3_max = 0;

% Resolucion angular
paso = 2;       % grados


%% ============================================================
% 2. GENERACION DE NUBE DE PUNTOS ALCANZABLES
% =============================================================

workspace = [];

for q1 = q1_min:paso:q1_max
    
    q1r = deg2rad(q1);

    for q2 = q2_min:paso:q2_max
        
        q2r = deg2rad(q2);

        for q3 = q3_min:paso:q3_max
            
            q3r = deg2rad(q3);

            % Cinematica directa en plano de la pata
            r = L1 + L2*cos(q2r) + L3*cos(q2r+q3r);

            z = L2*sin(q2r) + L3*sin(q2r+q3r);
            
            x = r*cos(q1r);
            y = r*sin(q1r);


            workspace = [
                workspace;
                x y z
                ];

        end
    end
end


% Eliminar puntos repetidos
workspace = unique(round(workspace,3),'rows');


fprintf("Puntos generados: %d\n",size(workspace,1));


%% ============================================================
% 3. VISUALIZAR NUBE DE PUNTOS
% =============================================================

figure;
scatter3(workspace(:,1),workspace(:,2),workspace(:,3),1,'.');
grid on;
axis equal;
xlabel("X");
ylabel("Y");
zlabel("Z");
title("Workspace completo");

%% ============================================================
% 4. GENERAR CUERPO GEOMETRICO CONTINUO DEL WORKSPACE
% =============================================================

% Crear alphaShape inicialmente sin fijar Alpha
workspaceBody = alphaShape( ...
    workspace(:,1), ...
    workspace(:,2), ...
    workspace(:,3));


% =============================================================
% CALCULAR UNA ESCALA ESPACIAL DE LA NUBE
% =============================================================
%
% La separación entre puntos NO debe interpretarse como huecos
% reales del workspace.
%
% Se estima una distancia característica entre puntos vecinos.
%

DT = delaunayTriangulation(workspace(:,1:3));

E = edges(DT);

P1 = DT.Points(E(:,1),:);
P2 = DT.Points(E(:,2),:);

distancias = sqrt(sum((P1-P2).^2,2));


% Ignorar conexiones largas producidas por la triangulación.
% Nos interesan solamente las conexiones locales.

distanciaLocal = median(distancias);


% =============================================================
% ALPHA DEL CUERPO
% =============================================================
%
% IMPORTANTE:
%
% No usar:
%
% criticalAlpha(...,'all-points')
%
% porque obliga al cuerpo a envolver puntos alejados y termina
% cerrando concavidades reales.
%
% Alpha se fija usando solamente la separación local de puntos.
%

factorAlpha = 2.5;

alphaWorkspace = factorAlpha * distanciaLocal;

workspaceBody.Alpha = alphaWorkspace;


fprintf("Distancia local: %.3f mm\n",distanciaLocal);
fprintf("Alpha utilizado: %.3f mm\n",alphaWorkspace);


% =============================================================
% VISUALIZACION
% =============================================================

figure;

plot(workspaceBody);

axis equal;
grid on;

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

title('Cuerpo continuo del workspace');

view(3);

rotate3d on;


%% ============================================================
% 5. DISCRETIZACION DE ALTURAS
% =============================================================

% Límite físico calculado por la cinemática
z_limite = min(workspace(:,3));

% Primera altura normal múltiplo de 5 dentro del workspace
z_min = ceil(z_limite/5)*5;

% El robot solo trabajará desde el centro hacia abajo. Se calculan mas
% alturas para poder levantar el pie despues
z_max = 50;

% Conservar explícitamente la máxima extensión real
z_levels = [z_min:8:z_max];


poligonos = struct();


%% ============================================================
% 6. GENERAR POLIGONO ALCANZABLE EN CADA ALTURA
% =============================================================

poligonos = struct();

% Resolución espacial utilizada para hacer cada corte.
% 0.5 mm da bastante precisión.
resXY = 0.5;


% ------------------------------------------------------------
% Límites generales del cuerpo
% ------------------------------------------------------------

xmin = min(workspace(:,1));
xmax = max(workspace(:,1));

ymin = min(workspace(:,2));
ymax = max(workspace(:,2));


% ------------------------------------------------------------
% Malla XY utilizada para hacer los cortes
% ------------------------------------------------------------

xGrid = xmin:resXY:xmax;
yGrid = ymin:resXY:ymax;

[Xgrid,Ygrid] = meshgrid(xGrid,yGrid);


% ------------------------------------------------------------
% Recorrer todas las alturas
% ------------------------------------------------------------

contadorPoligono = 1;

for h = 1:length(z_levels)

    Z = z_levels(h);

    fprintf("Generando poligono %d de %d | Z = %.1f\n", ...
        h,length(z_levels),Z);


    % ========================================================
    % 1. HACER CORTE DEL CUERPO EN ESTA ALTURA
    % ========================================================

    Zgrid = Z * ones(size(Xgrid));

    dentro = inShape( ...
        workspaceBody, ...
        Xgrid(:), ...
        Ygrid(:), ...
        Zgrid(:));

    dentro = reshape(dentro,size(Xgrid));


    % Si en esta altura no existe workspace
    if ~any(dentro,'all')
        continue;
    end


    % ========================================================
    % 2. ENCONTRAR UN PUNTO DE LA REGION EXTERIOR
    %
    % Se utiliza el punto alcanzable con mayor X.
    %
    % Esto hace que la búsqueda comience directamente en la
    % zona útil más alejada del robot.
    % ========================================================

    [filas,columnas] = find(dentro);

    valoresX = Xgrid(sub2ind(size(Xgrid),filas,columnas));

    [~,indiceMayorX] = max(valoresX);

    filaInicial = filas(indiceMayorX);
    columnaInicial = columnas(indiceMayorX);


    % ========================================================
    % 3. RECORRER SOLO LA REGION CONECTADA AL PUNTO DE MAYOR X
    %
    % Las demás regiones NO se detectan.
    % La búsqueda nunca entra en ellas.
    % ========================================================

    region = false(size(dentro));

    % Cola para flood-fill
    maxElementos = nnz(dentro);

    colaFila = zeros(maxElementos,1);
    colaColumna = zeros(maxElementos,1);

    inicioCola = 1;
    finCola = 1;

    colaFila(1) = filaInicial;
    colaColumna(1) = columnaInicial;

    region(filaInicial,columnaInicial) = true;


    while inicioCola <= finCola

        f = colaFila(inicioCola);
        c = colaColumna(inicioCola);

        inicioCola = inicioCola + 1;


        % ----------------------------------------------
        % Revisar los 8 vecinos
        % ----------------------------------------------

        for df = -1:1

            for dc = -1:1

                if df == 0 && dc == 0
                    continue;
                end

                fn = f + df;
                cn = c + dc;


                % Comprobar límites
                if fn < 1 || fn > size(dentro,1)
                    continue;
                end

                if cn < 1 || cn > size(dentro,2)
                    continue;
                end


                % El punto debe:
                %
                % 1. pertenecer al cuerpo
                % 2. no haber sido visitado
                % --------------------------------------

                if dentro(fn,cn) && ~region(fn,cn)

                    region(fn,cn) = true;

                    finCola = finCola + 1;

                    colaFila(finCola) = fn;
                    colaColumna(finCola) = cn;

                end

            end

        end

    end


    % ========================================================
    % 4. EXTRAER LOS PUNTOS DE LA REGION SELECCIONADA
    % ========================================================

    Xregion = Xgrid(region);
    Yregion = Ygrid(region);


    % Por seguridad
    if length(Xregion) < 3
        continue;
    end


    % ========================================================
    % 5. OBTENER SOLO EL CONTORNO
    %
    % boundary conserva concavidades, a diferencia de convhull.
    % ========================================================

    factorContraccion = 0.90;

    indicesContorno = boundary( ...
        Xregion, ...
        Yregion, ...
        factorContraccion);


    XY = [
        Xregion(indicesContorno) ...
        Yregion(indicesContorno)
    ];


    % ========================================================
    % 6. CERRAR POLIGONO
    % ========================================================

    if any(XY(1,:) ~= XY(end,:))

        XY = [
            XY
            XY(1,:)
        ];

    end


    % ========================================================
    % 7. GUARDAR
    % ========================================================

    poligonos(contadorPoligono).z = Z;

    poligonos(contadorPoligono).XY = XY;

    contadorPoligono = contadorPoligono + 1;

end


fprintf("Poligonos generados: %d\n",length(poligonos));

%% ============================================================
% 7. GRAFICAR POLIGONOS POR ALTURA (3D)
% =============================================================

figure;
hold on;
grid on;
axis equal;
view(3);

% Colores según la altura
colores = turbo(length(poligonos));

for i = 1:length(poligonos)

    XY = poligonos(i).XY;
    Z = poligonos(i).z;

    % Si el polígono no está cerrado, cerrarlo
    if any(XY(1,:) ~= XY(end,:))
        XY = [XY; XY(1,:)];
    end

    % Dibujar el contorno
    plot3( XY(:,1), ...
           XY(:,2), ...
           Z*ones(size(XY,1),1), ...
           'Color', colores(i,:), ...
           'LineWidth', 2);

    % Rellenar el polígono
    fill3( XY(:,1), ...
           XY(:,2), ...
           Z*ones(size(XY,1),1), ...
           colores(i,:), ...
           'FaceAlpha',0.30,...
           'EdgeColor','none');

end

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');

title('Áreas alcanzables por altura');

colormap(turbo);
cb = colorbar;
cb.Label.String = 'Altura';

caxis([poligonos(1).z poligonos(end).z]);

rotate3d on;
axis vis3d;
box on;

camproj perspective;
camlight headlight;
lighting gouraud;


%% ============================================================
% 8. GUARDAR POLIGONOS
% =============================================================

save("Workspace_Poligonos.mat","poligonos");


fprintf("Archivo Workspace_Poligonos.mat generado\n");

%%
load Workspace_Poligonos.mat

% ============================================================
% CONFIGURACION
% =============================================================

angulo_step = 2;
angulos = 0:angulo_step:178;

paso = 0.5;        % resolución para buscar intersecciones
margen = 0.5;      % evita errores numéricos

% ============================================================
% VARIABLES
% =============================================================

Lineas = struct();

contador = 1;

offsetAnterior = [];
ventanaBusqueda = 15;   % mm

% ============================================================
% RECORRER TODAS LAS ALTURAS
% =============================================================

for h = 1:length(poligonos)

    fprintf("Altura %d de %d\n",h,length(poligonos));

    Z = poligonos(h).z;

    P = poligonos(h).XY;

    % cerrar poligono
    if any(P(1,:)~=P(end,:))
        P=[P;P(1,:)];
    end

    xmin=min(P(:,1));
    xmax=max(P(:,1));

    ymin=min(P(:,2));
    ymax=max(P(:,2));

    % --------------------------------------------------------
    % Para cada direccion
    %--------------------------------------------------------

    for a=1:length(angulos)

        ang=angulos(a);

        theta=deg2rad(ang);

        d=[cos(theta) sin(theta)];

        % vector perpendicular
        n=[-sin(theta) cos(theta)];

        % rango suficiente para recorrer todo el poligono
        diagonal=hypot(xmax-xmin,ymax-ymin);

        if h==1

            offsets = -diagonal:paso:diagonal;
        
        else
        
            if isempty(offsetAnterior)
        
                offsets = -diagonal:paso:diagonal;
        
            else
        
                centro = offsetAnterior(a);
        
                offsets = (centro-ventanaBusqueda):paso:(centro+ventanaBusqueda);
        
                offsets(offsets<-diagonal) = [];
                offsets(offsets> diagonal) = [];
        
            end
        
        end

        mejorLongitud=0;

        mejorP1=[NaN NaN];
        mejorP2=[NaN NaN];

        mejorOffset = NaN;

        toleranciaLongitud = 0.5;   % mm

        primerOffset = NaN;
        ultimoOffset = NaN;

        % ----------------------------------------------------
        % Barrer todas las rectas paralelas
        % ----------------------------------------------------

        for k=1:length(offsets)

            c=offsets(k);

            % punto sobre la recta
            p0=n*c;

            % recorrer la recta
            t=-diagonal*2:paso:diagonal*2;

            x=p0(1)+d(1)*t;
            y=p0(2)+d(2)*t;

            dentro=inpolygon(x,y,P(:,1),P(:,2));

            if ~any(dentro)
                continue
            end

            cambios=diff([0 dentro 0]);

            ini=find(cambios==1);
            fin=find(cambios==-1)-1;

            if isempty(ini)
                continue
            end

            for s=1:length(ini)

                t1=t(ini(s));
                t2=t(fin(s));

                L=t2-t1;

                if L > mejorLongitud + toleranciaLongitud

                    mejorLongitud = L;
                
                    mejorP1 = p0 + d*t1;
                    mejorP2 = p0 + d*t2;
                
                    primerOffset = c;
                    ultimoOffset = c;
                
                elseif abs(L - mejorLongitud) <= toleranciaLongitud
                
                    ultimoOffset = c;
                
                end

            end

        end

        % ----------------------------------------------------
        % Guardar resultado
        % ----------------------------------------------------

        % Colocar la recta en el centro del conjunto de rectas máximas

        mejorOffset = (primerOffset + ultimoOffset)/2;
        
        p0 = n*mejorOffset;
        
        t = -diagonal*2:paso:diagonal*2;
        
        x = p0(1)+d(1)*t;
        y = p0(2)+d(2)*t;
        
        dentro = inpolygon(x,y,P(:,1),P(:,2));
        
        cambios = diff([0 dentro 0]);
        
        ini = find(cambios==1);
        fin = find(cambios==-1)-1;
        
        mejorLongitud = 0;
        
        for s=1:length(ini)
        
            t1 = t(ini(s));
            t2 = t(fin(s));
        
            L = t2-t1;
        
            if L>mejorLongitud
        
                mejorLongitud = L;
        
                mejorP1 = p0+d*t1;
                mejorP2 = p0+d*t2;
        
            end
        
        end
        
        offsetActual(a)=mejorOffset;
        
        Lineas(contador).z=Z;

        Lineas(contador).angulo=ang;

        Lineas(contador).x1=mejorP1(1);
        Lineas(contador).y1=mejorP1(2);

        Lineas(contador).x2=mejorP2(1);
        Lineas(contador).y2=mejorP2(2);

        Lineas(contador).longitud=mejorLongitud;

        contador=contador+1;

    end

    offsetAnterior = offsetActual;
    
    offsetActual = [];

end



fprintf("Lineas calculadas.\n");


N = length(Lineas);

for i = 1:N

    Lineas(contador).z = Lineas(i).z;

    Lineas(contador).angulo = mod(Lineas(i).angulo+180,360);

    Lineas(contador).x1 = Lineas(i).x2;
    Lineas(contador).y1 = Lineas(i).y2;

    Lineas(contador).x2 = Lineas(i).x1;
    Lineas(contador).y2 = Lineas(i).y1;

    Lineas(contador).longitud = Lineas(i).longitud;

    contador = contador+1;

end

%% ============================================================
% GUARDAR MAT
% =============================================================

save("LineasMaximas.mat","Lineas");

%% ============================================================
% EXPORTAR CSV
% =============================================================

N=length(Lineas);

tabla=zeros(N,7);

for i=1:N

    tabla(i,1)=Lineas(i).z;
    tabla(i,2)=Lineas(i).angulo;
    tabla(i,3)=Lineas(i).x1;
    tabla(i,4)=Lineas(i).y1;
    tabla(i,5)=Lineas(i).x2;
    tabla(i,6)=Lineas(i).y2;
    tabla(i,7)=Lineas(i).longitud;

end

writematrix(tabla,"LineasMaximas.csv");

fprintf("CSV generado.\n");
%% ============================================================
% VISUALIZACION 3D
% =============================================================

figure
hold on
grid on
axis equal
view(3)

colormap(parula(length(poligonos)));
colores = parula(length(poligonos));

% ------------------------------------------------------------
% Dibujar los polígonos en su altura correspondiente
% ------------------------------------------------------------

for h = 1:length(poligonos)

    P = poligonos(h).XY;
    Z = poligonos(h).z;

    % Relleno semitransparente
    fill3(P(:,1), ...
          P(:,2), ...
          Z*ones(size(P,1),1), ...
          colores(h,:), ...
          'FaceAlpha',0.25,...
          'EdgeColor',colores(h,:),...
          'LineWidth',1.5);

end

% ------------------------------------------------------------
% Dibujar las líneas máximas
% ------------------------------------------------------------

for i = 1:length(Lineas)

    z = Lineas(i).z;

    plot3([Lineas(i).x1 Lineas(i).x2], ...
          [Lineas(i).y1 Lineas(i).y2], ...
          [z z], ...
          'r', ...
          'LineWidth',1.2);

end

% ------------------------------------------------------------
% Etiquetas
% ------------------------------------------------------------

xlabel('X (mm)')
ylabel('Y (mm)')
zlabel('Z (mm)')

title('Líneas máximas del espacio de trabajo')

axis vis3d
rotate3d on
camlight headlight
lighting gouraud

fprintf("Proceso terminado.\n");