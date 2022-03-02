#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import Data.plotting as plotting
import Data.loading as loading
import matplotlib.pyplot as plt

#%%
# Load Ronny Data
ronny_trials = loading.load_folder('Output/sevgi/finger')
# Clean Ronny Data
ronny_trials = loading.clean_many_dataframes_from_initial_values(ronny_trials)

#%% Plot learning curve
plt.figure()  # create new plot window
plotting.plot_learning_curve(ronny_trials)

#%% Plot pedal presses
plt.figure()  # create new plot window
plotting.plot_pedal_press_counts_per_trial(ronny_trials)
