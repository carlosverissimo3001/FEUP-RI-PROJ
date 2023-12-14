import matplotlib.pyplot as plt
import subprocess
import sys
import numpy as np
import pandas as pd

positionExperimentData = pd.read_csv('positionExperimentData.csv')

# Load your background image
background_image = plt.imread("world.png")

# Create a figure
plt.figure(figsize=(10, 10))

# Display the background image
plt.imshow(background_image, extent=(-4.5, 7, -8, 8))

# Create the heat map
plt.scatter(positionExperimentData.loc[:, 'X'], positionExperimentData.loc[:, 'Y'],
            c=positionExperimentData.loc[:, 'Time'], cmap='viridis', marker='s', s=100, plotnonfinite=True)

# Add a colorbar for reference
plt.colorbar(label='Time Taken')

# Customize your plot as needed

# Show the plot
plt.show()
