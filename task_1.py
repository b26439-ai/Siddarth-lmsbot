#task1
def line_position(readings):
    positions = [-2, -1, 0, 1, 2]
    total_reading = sum(readings)
    if total_reading == 0:
        return 'ERROR: Line lost'  
    weighted_sum = sum(r * p for r, p in zip(readings, positions))
    return weighted_sum / total_reading
def limit_motor_command(left_command, right_command, max_magnitude):
    m = max(abs(left_command), abs(right_command))
    if m > max_magnitude:
        scaling_factor = max_magnitude / m
        left_command *= scaling_factor
        right_command *= scaling_factor
        
    return left_command, right_command
#example usage
print('line location example')
# Example 1: Line slightly to the right
readings_1 = [0, 1, 4, 2, 0]
pos_1 =line_position(readings_1)
print(f"Readings: {readings_1}")
print(f"Estimated Line Position: {pos_1}")
# Example 2: All-zero case (line lost)
readings_2 = [0, 0, 0, 0, 0]
pos_2 = line_position(readings_2)
print(f"Readings (All-Zero): {readings_2}")
print(f"Estimated Line Position: {pos_2}")
print('limiting motor command example')
# Example: Commands exceeding the maximum magnitude limit
left_cmd = 250
right_cmd = -300
max_limit = 200
limited_left, limited_right = limit_motor_command(left_cmd, right_cmd, max_limit)
print(f"Original Commands: {left_cmd}, {right_cmd}")
print(f"Max Magnitude Limit {max_limit}")
print(f"Limited Commands : {limited_left},  {limited_right}")