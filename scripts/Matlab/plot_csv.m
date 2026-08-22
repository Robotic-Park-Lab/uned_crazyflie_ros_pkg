function plot_csv(varargin)
% PLOT_CSV  Grafica uno o varios .csv generados por bag2csv.py (o por
%   bag2csv_crazyflie.py): una figura por fichero, una curva por columna
%   numérica frente a Timestamp (convertido a segundos relativos al
%   inicio del registro).
%
%   Complementa a Crazyflie_Graphs.m: ese script espera variables ya
%   cargadas en el workspace (references, dron_state, ...) generadas por
%   el pipeline Crazyflie_Launch.m + bag2txt_sim.sh + Txt2Mat.m. Este
%   lee directamente los .csv de bag2csv.py, sin ese pipeline intermedio.
%
%   Uso:
%       plot_csv('fichero1.csv')
%       plot_csv('fichero1.csv', 'fichero2.csv')

    if nargin < 1
        error('plot_csv: hay que pasar al menos un fichero .csv');
    end

    for k = 1:nargin
        file = varargin{k};
        data = readtable(file);

        if height(data) == 0
            warning('%s: vacío, se omite', file);
            continue
        end

        t0 = data.Timestamp(1);
        t = (data.Timestamp - t0) / 1e9; % ns -> s

        value_vars = data.Properties.VariableNames(2:end);
        numeric_vars = value_vars(varfun(@isnumeric, data(:, value_vars), ...
            'OutputFormat', 'uniform'));

        if isempty(numeric_vars)
            warning('%s: no tiene columnas numéricas que graficar', file);
            continue
        end

        [~, name, ext] = fileparts(file);
        figure('Name', [name ext]);
        hold on
        for v = 1:numel(numeric_vars)
            plot(t, data.(numeric_vars{v}));
        end
        hold off
        xlabel('Tiempo [s]')
        title([name ext], 'Interpreter', 'none')
        legend(numeric_vars, 'Interpreter', 'none', 'Location', 'best')
        grid minor
    end
end
