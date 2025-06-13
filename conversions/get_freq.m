function freq = get_freq(time)
dt = mean(diff(time));
freq = 1/dt;


