clc;
clear all;
close all;
robot_id = {{'robot1'}; {'robot2'}; {'robot3'}};
task_id = {{'wander'}; {'border_protection'}; {'report'}};
for i = 1 : rows(robot_id)
  for j = 1 : rows(task_id)
    try
      plot_motivation(robot_id{i}{1}, task_id{j}{1});
    catch e
      disp(e.message);
    end;
  end;
end;
