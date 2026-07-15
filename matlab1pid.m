% =========================================================
%  SCRIPT MATLAB UNTUK MONITORING DAN ANALISIS PID
%  Data diambil dari Arduino yang sudah pakai PID
%  Konstanta PID dari Ziegler–Nichols
%  Dengan batas waktu percobaan 600 detik (10 menit)
% =========================================================

% === SETUP SERIAL ===
port = 'COM6';      % Ganti sesuai port Arduino
baud = 115200;      % Harus sama dengan Serial.begin di Arduino
s = serial(port,'BaudRate',baud);
fopen(s);

disp('Mulai rekam... Percobaan jalan selama 10 menit...');

% === Variabel penyimpan data ===
t = [];
suhu = [];
setp = [];

% === Setup Grafik Real-Time ===
figure;
xlabel('Waktu (detik)');
ylabel('Suhu (°C)');
title('Uji PID Suhu Inkubator (Real-Time)');
grid on;
hold on;

h_suhu = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
h_setp = plot(NaN, NaN, 'r--', 'LineWidth', 1.5);
legend('Suhu','Setpoint','Location','southwest');

% === Variabel analisis PID ===
rise_time = NaN;
rise_time_done = false;

snapnow;
% === Timer untuk batas waktu percobaan ===
max_time = 3600; % detik (10 menit)
t0 = tic;

try
    while toc(t0) < max_time
        % ==== Baca data serial dari Arduino ====
        try
            data = fgetl(s);
        catch
            continue;
        end

        if isempty(data)
            continue;
        end

        nilai = str2double(strsplit(strtrim(data), ','));

        if length(nilai) == 3 && all(~isnan(nilai))
            t(end+1)    = nilai(1);
            suhu(end+1) = nilai(2);
            setp(end+1) = nilai(3);

            sp = setp(1);
            y  = suhu;
            tt = t - t(1);

            % === Analisis karakteristik respon PID ===
            if length(y) > 10
                if ~rise_time_done
                    if sp > y(1)
                        y10 = y(1) + 0.1*(sp - y(1));
                        y90 = y(1) + 0.9*(sp - y(1));
                        t_rise_start = find(y >= y10, 1);
                        t_rise_end   = find(y >= y90, 1);
                    else
                        y10 = y(1) - 0.1*(y(1) - sp);
                        y90 = y(1) - 0.9*(y(1) - sp);
                        t_rise_start = find(y <= y10, 1);
                        t_rise_end   = find(y <= y90, 1);
                    end
                    if ~isempty(t_rise_start) && ~isempty(t_rise_end)
                        rise_time = tt(t_rise_end) - tt(t_rise_start);
                        rise_time_done = true;
                    end
                end

                % Settling Time (2% band)
                within_band = abs(y - sp) <= 0.02*abs(sp);
                if any(within_band)
                    last_idx = find(~within_band, 1, 'last');
                    if isempty(last_idx)
                        settling_time = tt(end);
                    elseif last_idx < length(tt)
                        settling_time = tt(last_idx+1);
                    else
                        settling_time = tt(end);
                    end
                else
                    settling_time = NaN;
                end

                % Overshoot
                ymax = max(y);
                if sp ~= 0
                    max_overshoot = (ymax - sp) / abs(sp) * 100;
                else
                    max_overshoot = NaN;
                end

                % Steady-State Error
                steady_state_error = abs(sp - y(end));

                % Cetak ke Command Window
                fprintf('Rise Time: %.2f s | Settling Time: %.2f s | Overshoot: %.2f %% | SSE: %.2f °C\n', ...
                    rise_time, settling_time, max_overshoot, steady_state_error);
            else
                settling_time = NaN;
                max_overshoot = NaN;
                steady_state_error = NaN;
            end

            % === Update Grafik Real-Time ===
            set(h_suhu, 'XData', tt, 'YData', suhu);
            set(h_setp, 'XData', tt, 'YData', setp);

            if ~isnan(rise_time)
                info_txt = {
                    ['Rise Time: ' num2str(rise_time,'%.2f') ' s'], ...
                    ['Settling Time: ' num2str(settling_time,'%.2f') ' s'], ...
                    ['Overshoot: ' num2str(max_overshoot,'%.2f') ' %'], ...
                    ['SSE: ' num2str(steady_state_error,'%.2f') ' °C']
                    };
                x_pos = tt(end) - (tt(end)-tt(1))*0.4;
                y_pos = sp + 2;
                delete(findall(gca,'Type','text'));
                text(x_pos, y_pos, info_txt, 'BackgroundColor','w','EdgeColor','k');
            end

            drawnow;
        end
    end
catch
    disp('Rekaman dihentikan manual');
end

% === Simpan data ke file CSV ===
data_tabel = table(t', suhu', setp', 'VariableNames', {'Waktu_dtk','Suhu','Setpoint'});
writetable(data_tabel, 'data_pid.csv');
disp('Data berhasil disimpan di file: data_pid.csv');

% === Tutup serial ===
fclose(s);
delete(s);
clear s;
disp('Percobaan selesai otomatis (10 menit).');