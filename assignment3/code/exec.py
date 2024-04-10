# import subprocess

# def execute_and_collect_output(map_file):
#     command = ['./a.out', map_file]
#     process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#     stdout, stderr = process.communicate()
#     output = stdout.decode('utf-8')
#     lines = output.split('\n')
    
#     target_caught = None
#     path_cost = None
    
#     for line in lines:
#         if "target caught" in line:
#             target_caught = int(line.split('=')[1].strip())
#         elif "path cost" in line:
#             path_cost = int(line.split('=')[1].strip())

#     return target_caught, path_cost

# def tabulate_results():
#     results = {}
#     for i in range(1, 10):
#         map_file = f"map{i}.txt"
#         target_caught, path_cost = execute_and_collect_output(map_file)
#         results[map_file] = {'Target Caught': target_caught, 'Path Cost': path_cost}
#     return results

# if __name__ == "__main__":
#     results = tabulate_results()
#     print("Map File\tTarget Caught\tPath Cost")
#     for map_file, data in results.items():
#         target_caught = data['Target Caught']
#         path_cost = data['Path Cost']
#         print(f"{map_file}\t{target_caught}\t\t{path_cost}")

import subprocess
import time

def execute_and_collect_output(map_file):
    start_time = time.time()  # Start timing
    command = ['./a.out', map_file]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    stdout, stderr = process.communicate()
    end_time = time.time()  # End timing
    
    execution_time = end_time - start_time  # Calculate the time taken
    
    output = stdout.decode('utf-8')
    lines = output.split('\n')
    
    target_caught = None
    path_cost = None
    
    for line in lines:
        if "target caught" in line:
            target_caught = int(line.split('=')[1].strip())
        elif "path cost" in line:
            path_cost = int(line.split('=')[1].strip())

    return target_caught, path_cost, execution_time

def tabulate_results():
    results = {}
    for i in range(1, 10):
        map_file = f"map{i}.txt"
        target_caught, path_cost, execution_time = execute_and_collect_output(map_file)
        results[map_file] = {
            'Target Caught': target_caught, 
            'Path Cost': path_cost, 
            'Execution Time': execution_time
        }
    return results

if __name__ == "__main__":
    results = tabulate_results()
    print("Map File\tTarget Caught\tPath Cost\tExecution Time (s)")
    for map_file, data in results.items():
        target_caught = data['Target Caught']
        path_cost = data['Path Cost']
        execution_time = data['Execution Time']
        print(f"{map_file}\t{target_caught}\t\t{path_cost}\t\t{execution_time:.4f}")
