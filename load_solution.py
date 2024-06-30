def load_solution_variables(solution_file_path):
    solution = {}
    delta = 1e-6  # Small delta for numerical errors
    with open(solution_file_path, 'r') as file:
        for line in file:
            # Check if the line contains a space, indicating a possible variable assignment
            if ' ' in line:
                parts = line.split()
                if len(parts) == 2:  # Ensure the line is in the format 'variable_name value'
                    try:
                        variable_name = parts[0]
                        value = float(parts[1])
                        
                        # Check if the variable is active
                        if value > delta:
                            solution[variable_name] = value
                    except ValueError:
                        # If conversion to float fails, skip the line
                        continue
    return solution

import numpy as np

def extract_active_routes(solution, N, Tr):
    # Extract active trucks
    active_trucks = [v for v in Tr if any(key.startswith(f'y[{v}]') and value >= 0.99 for key, value in solution.items())]

    # Extract routes for active trucks
    active_routes_truck = {}
    for truck in active_trucks:
        active_routes_truck[truck] = []
        for node_from in N:
            for node_to in N:
                if node_from != node_to:
                    key = f'x[{truck},{node_from},{node_to}]'
                    if solution.get(key, 0) >= 0.99:
                        timestamp = solution.get(f't[{truck},{node_to}]', 0)
                        weight = solution.get(f'W_dynamic[{truck},{node_from},{node_to}]', 0)
                        active_routes_truck[truck].append((node_from, node_to, timestamp, np.round(weight, 1)))
    
    return active_routes_truck

# Example usage
solution_file_path = '40_40_0.3_copy_solution.sol'
solution = load_solution_variables(solution_file_path)
# Assuming N (set of nodes) and Tr (set of trucks) are defined elsewhere in your code
# Example usage
N = ['N1', 'N2', 'N3']  # Example set of nodes
Tr = ['Tr1', 'Tr2', 'Tr3']  # Example set of trucks
active_routes = extract_active_routes(solution, N, Tr)
print(active_routes)





