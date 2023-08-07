def calculate_slope(data):
    slopes = []
    for i in range(1, len(data)):
        x_diff = data[i][0] - data[i - 1][0]
        y_diff = data[i][1] - data[i - 1][1]
        slope = y_diff / x_diff
        slopes.append(slope)
    return slopes
