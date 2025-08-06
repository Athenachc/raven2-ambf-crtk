# **RAVEN-II AMBF CRTK Framework**  
Forked from: [lbyng/raven2-ambf-crtk](https://github.com/lbyng/raven2-ambf-crtk)

## **1. Installation & Setup**  

### **Step 1: Setup RAVEN-II Package**

Ensure that you have the **[RAVEN-II Package](https://github.com/uw-biorobotics/raven2)** installed and running on your system.
```bash
mkdir -p ~/raven2_ws/src
cd raven2_ws/src
git clone https://github.com/uw-biorobotics/raven2.git
cd ..
catkin_make

source ~/raven2_ws/devel/setup.bash
```
### **Step 2: Setup AMBF Simulator**  

Ensure that you have the **[AMBF simulator (with updated RAVEN2 model)](https://github.com/Athenachc/ambf)** installed and running on your system.

### **Step 3: Setup AMBF Matcap Plugin**  

Ensure that you have the **[AMBF Matcap plugin (with RAVEN2 model)](https://github.com/Athenachc/ambf_matcap_plugin)** installed and running on your system.


### **Step 4: Run AMBF RAVEN-II**

Launch the AMBF simulator with the updated RAVEN-II model (without matcap) by executing:

```bash
~/ambf/bin/lin-x86_64/ambf_simulator -a "~/ambf/ambf_models/meshes/blender_afmb/raven_2/raven_straight.yaml"
```

Launch the AMBF simulator with the updated RAVEN-II model (with matcap) by executing:

```bash
~/ambf/bin/lin-x86_64/ambf_simulator -a "~/ambf_matcap_plugin/raven2/raven_straight_matcap.yaml"
```

If everything is set up correctly, you should see the **AMBF RAVEN-II (with/without matcap)** running.

### **Step 5: Add an Alias for Convenience (Optional)**

To simplify launching the simulator, add two alias in your `.bashrc` file:

```bash
# Open ~/.bashrc in a text editor
nano ~/.bashrc  

# Add the following alias at the end of the file:
alias ambf-raven2="~/ambf/bin/lin-x86_64/ambf_simulator -a '/home/athena/ambf/ambf_models/meshes/blender_afmb/raven_2/raven_straight.yaml'"

alias ambf-raven2-matcap="~/ambf/bin/lin-x86_64/ambf_simulator -a '/home/athena/ambf_matcap_plugin/raven2/raven_straight.yaml'"

# Save and exit (Ctrl + O, then Enter, then Ctrl + x)
```

Reload the `.bashrc` file to apply changes:

```bash
source ~/.bashrc
```

Now, you can start **AMBF RAVEN-II** anytime by simply running:
```bash
roscore
```
And then open a new terminal and run: 

```bash
ambf-raven2 # without AMBF matcap plugin
```

Or 

```bash
ambf-raven2-matcap # with AMBF matcap plugin
```

### **Step 7: Run the RAVEN-II AMBF CRTK Framework**

Once AMBF is running, open a new terminal and start the **RAVEN-II framework** by executing:

```bash
python3 ~/raven2-ambf-crtk/main.py
```

At this point, the left arm (yellow one) of the **AMBF RAVEN-II** should be homed.

## **2. Keyboard Controller**

You can control the **RAVEN-II arm** using a keyboard. Follow these steps:

### **Start AMBF RAVEN-II and the Framework**

Ensure that the RAVEN-II's left arm is homed by the above steps.

### **Run the Keyboard Controller**

Now, start the keyboard control interface:

```bash
python3 ~/raven2-ambf-crtk/keyboard_controller.py
```

### **Notes:**

- The current version **only allows one key input at a time** for stability.
- To modify velocity settings, edit the file:

    ```bash
    ~/raven2-ambf-crtk/keyboard_controller.py
    ```

    At the beginning of this script, you can adjust the **velocity variables** for each joint.

## **3. Run RAVEN2 dataset**
### Download RAVEN2 dataset
[Download here](https://datadryad.org/dataset/doi:10.5061/dryad.tqjq2bw84)

### Locate dataset file
- Edit your desired dataset file location and filename in `sim_test.py`
- For instance:
```
# Import data from dataset
file_location = '/home/athena/Downloads/doi_10_5061_dryad_tqjq2bw84__v20241114/record_1_different_directions'
file_name = 'data_record_x_03.csv'
```

### Run dataset
Ensure that the RAVEN-II's left arm is homed by the above steps.

Then, open a new terminal and run:
```
python3 ~/raven2-ambf-crtk/sim_test.py
```

## Troubleshooting

1. Solve "ImportError: dynamic module does not define module export function (PyInit__tf2)"
   
    ```bash
    sudo apt update
    sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
        
    mkdir -p ~/catkin_ws/src; cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    wstool init
    wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
    wstool up
    rosdep install --from-paths src --ignore-src -y -r
        
    catkin_make --cmake-args \
                -DCMAKE_BUILD_TYPE=Release \
                -DPYTHON_EXECUTABLE=/usr/bin/python3 \
                -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
                -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
    # Check the 3 folders: 1) /usr/bin; 2) /usr/include; 3) /usr/lib to see whether the filenames are correct.
    # If your files' address are not same as mine, plz replace by your own.
        
    source ~/catkin_ws/devel/setup.bash
    ```
