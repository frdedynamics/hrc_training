#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import matplotlib.pyplot as plt
import data


def plot_learning_curve(dataframes, show_points=True):
    # Calculate the average time for each dataframe
    means = []
    for trial in dataframes:
        # Calculate the average time of this run of the simulation
        means.append(trial[data.ID_TIME].max())
    # Set the X axis to start with 1; As in 1st try, 2nd try, etc.
    x_axis = range(1, len(means)+1)
    # Plot the average simulation times of this user (learning curve)
    if show_points:
        frmt = '-o'
    else:
        frmt = '-'
    plt.plot(x_axis, means, frmt)
    # Set the properties of the plot
    plt.ylabel('Average Simulation Time (seconds)')
    plt.xlabel('Times Tried')
    plt.suptitle('Learning Curve')
    plt.show()


def plot_many_learning_curves(users, show_points=True):
    for user_dataframes in users:
        plot_learning_curve(user_dataframes, show_points)
    # set the ticks X axis ticks to be from 1 to the most amount of trials a user has done
    # just in case not all users have performed the same amount of trials
    max_trials = 0
    for user_dataframes in users:
        max_trials = max(max_trials, len(user_dataframes))
    x_axis_ticks = range(1, max_trials+1)
    plt.xticks(x_axis_ticks)
    
    
def plot_pedal_press_counts_per_trial(dataframes, show_points=True):
    # Calculate the average time for each dataframe
    pedal_counts = []
    for trial in dataframes:
        # Calculate the average time of this run of the simulation
        pedal_counts.append(trial[data.ID_PEDAL].max())
    # Set the X axis to start with 1; As in 1st try, 2nd try, etc.
    x_axis = range(1, len(pedal_counts)+1)
    # Plot the average simulation times of this user (learning curve)
    if show_points:
        frmt = '-o'
    else:
        frmt = '-'
    plt.plot(x_axis, pedal_counts, frmt)
    # Set the properties of the plot
    plt.ylabel('Pedal Press Count')
    plt.xlabel('Times Tried')
    plt.suptitle('Foot Pedal Presses per Trial')
    plt.show()


def plot_many_pedal_press_counts_per_trial(users, show_points=True):
    for user_dataframes in users:
        plot_pedal_press_counts_per_trial(user_dataframes, show_points)
    # set the ticks X axis ticks to be from 1 to the most amount of trials a user has done
    # just in case not all users have performed the same amount of trials
    max_trials = 0
    for user_dataframes in users:
        max_trials = max(max_trials, len(user_dataframes))
    x_axis_ticks = range(1, max_trials+1)
    plt.xticks(x_axis_ticks)
