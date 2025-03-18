#!/bin/bash


#you can create a batch file (or a shell script) to launch programs on a Raspberry Pi. On Raspberry Pi, the commonly used shell is Bash.
#Note: Ensure that the script has the necessary permissions to execute (chmod +x). Also, be cautious with the paths to Python and the scripts.
#If you're using virtual environments or different Python versions, you might need to specify the full path to the Python interpreter.

#You can then run the script using:

#./launch_programs.sh


#You can create a Bash script on your Raspberry Pi to launch two Python programs with infinite loops. 
#Here's an example script named launch_programs.sh:

# Launch the first Python program in the background
#!/bin/bash
export PYTHONPATH=$PYTHONPATH:/home/sergio/.local/lib/python3.9/site-packages


python /home/sergio/Projects/UTSA/baby_Rowdy/baby_Robot_body/release/baby_rowdy_control.py &


#!/bin/bash



#!/bin/bash


# Give it some time to start
sleep 12

# Launch the second Python program in the background

python  /home/sergio/Projects/UTSA/baby_Rowdy/baby_Robot_body/release/rollie_wheels_control.py &


# Keep the script running so that the programs continue to execute
while true; do
    sleep 1
done




