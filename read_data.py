import matplotlib.pyplot as plt

# Open the file in read mode
with open('result2.txt', 'r') as f:
    # Initialize empty lists for x and y values
    x_values = []
    y_values = []
    
    # Loop over each line in the file
    for line in f:
        # Split the line into x and y values
        x, y = line.split()
        
        # Convert x and y values to float and append to lists
        x_values.append(float(x))
        y_values.append(float(y))
        
# Print the x and y values
print('X values:', x_values)
print('Y values:', y_values)


# Create a line plot based on the data
plt.scatter(x_values, y_values)

# Add labels and a title to the plot
plt.xlabel('X values')
plt.ylabel('Y values')
plt.title('Sample Plot')

# Display the plot
plt.show()