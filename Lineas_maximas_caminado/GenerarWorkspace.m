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
paso = 1.5;       % grados


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
% 4. DISCRETIZACION DE ALTURAS
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
% 5. GENERAR POLIGONOS (CONCAVE HULL)
% =============================================================

tolerancia = 1.3;

contador = 1;

alpha = 18;      % Ajustar entre 12 y 30 según el resultado

for n = 1:length(z_levels)

    zActual = z_levels(n);

    % En el límite físico usamos una capa especialmente fina para
    % no sobreestimar el área alcanzable en máxima extensión.
    if n == 1
        toleranciaActual = 0.05;
    else
        toleranciaActual = tolerancia;
    end
    
    indices = abs(workspace(:,3)-zActual) <= toleranciaActual;

    puntos = workspace(indices,:);

    if size(puntos,1)<4
        continue
    end

    XY = puntos(:,1:2);

    DT = delaunayTriangulation(XY);

    T = DT.ConnectivityList;
    P = DT.Points;

    mantener = false(size(T,1),1);

    for i=1:size(T,1)

        p1 = P(T(i,1),:);
        p2 = P(T(i,2),:);
        p3 = P(T(i,3),:);

        a = norm(p2-p1);
        b = norm(p3-p2);
        c = norm(p1-p3);

        s = (a+b+c)/2;

        A = sqrt(max(s*(s-a)*(s-b)*(s-c),0));

        if A<1e-8
            continue
        end

        R = a*b*c/(4*A);

        if R < alpha
            mantener(i)=true;
        end

    end

    T = T(mantener,:);

    if isempty(T)
        continue
    end

    E = [
        T(:,[1 2]);
        T(:,[2 3]);
        T(:,[3 1])
        ];

    E = sort(E,2);

    [U,~,ic] = unique(E,'rows');

    veces = accumarray(ic,1);

    borde = U(veces==1,:);

    if size(borde,1)<3
        continue
    end

    vecinos = cell(size(P,1),1);

    for i=1:size(borde,1)

        a = borde(i,1);
        b = borde(i,2);

        vecinos{a}(end+1)=b;
        vecinos{b}(end+1)=a;

    end

    visitado = false(size(P,1),1);

    mejorOrden = [];
    mejorLongitud = 0;
    
    for inicio = 1:size(P,1)
    
        if visitado(inicio)
            continue
        end
    
        if isempty(vecinos{inicio})
            continue
        end
    
        anterior = 0;
        actual = inicio;
    
        orden = [];
    
        while true
    
            if visitado(actual)
                break
            end
    
            visitado(actual)=true;
    
            orden(end+1)=actual;
    
            v = vecinos{actual};
    
            if isempty(v)
                break
            elseif length(v)==1
                siguiente=v(1);
            else
    
                if v(1)==anterior
                    siguiente=v(2);
                else
                    siguiente=v(1);
                end
    
            end
    
            anterior=actual;
            actual=siguiente;
    
            if actual==inicio
                break
            end
    
        end
    
        if length(orden)>mejorLongitud
    
            mejorLongitud=length(orden);
            mejorOrden=orden;
    
        end
    
    end
    
    poligonos(contador).z = zActual;
    poligonos(contador).XY = P(mejorOrden,:);

    contador = contador+1;

end

fprintf("Poligonos obtenidos: %d\n",length(poligonos));


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