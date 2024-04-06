import subprocess

# Loop through the numbers 1 to 9
for i in range(1, 10):
    # Construct the map filename
    map_file = f"map{i}.txt"
    # Execute the command with the map file
    subprocess.run(["./a.out", map_file])

