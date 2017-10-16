function plot_motivation(filename, robot_id, task_id)

	csv_file = csvread(filename, 1, 0);
	t = csv_file(:, 1);
	t = 10e-9 * (t - min(t) * ones(size(t)));
	figure;
	title(['Motivation of the ' robot_id '/' task_id ' behaviour set']);
	subplot(6, 1, 1); plot(t, csv_file(:, 2)); grid on; ylabel('Impatience');
	subplot(6, 1, 2); plot(t, csv_file(:, 3)); grid on; ylabel('Acquiescent');
	subplot(6, 1, 3); plot(t, csv_file(:, 4)); grid on; ylabel('Suppressed');
	subplot(6, 1, 4); plot(t, csv_file(:, 5)); grid on; ylabel('Resetted');
	subplot(6, 1, 5); plot(t, csv_file(:, 6)); grid on; ylabel('Applicable');
	subplot(6, 1, 6); plot(t, csv_file(:, 7)); grid on; ylabel('Motivation');
	xlabel('t (s)');

end