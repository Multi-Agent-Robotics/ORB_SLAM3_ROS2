# ORB-SLAM3 Experiments

To run the experiments, you need to install and build the following library: https://github.com/Multi-Agent-Robotics/ORB_SLAM3.
The modification is on the `dev` branch of the library. So you need to checkout to that branch.
Note to test with different sizes of the circular buffer, you need to modify the source code of the library. Modify `./include/Tracking.h` it contains a member variable called `outlier_memory` with the type `CircularBuffer<T, N>` where `N` is the buffer size.

The script below tests the ORB-SLAM3 with the following datasets:
- [Dynavins](https://github.com/url-kaist/dynaVINS)
- [VIODE](https://github.com/kminoda/VIODE)

Follow the instructions at each dataset repository to download the datasets.

The script will run the launch files in `./orbslam3_bringup/launch/` and save the output to `./dynavins/` and `./viode/` directories.
You have to open each launch file and edit the absolute path to the place where you downloaded the datasets.

```fish
set -l green (set_color green)
set -l yellow (set_color yellow)
set -l red (set_color red)
set -l reset (set_color reset)

set -l N 5 # number of runs per launch file
set -l CIRCULAR_BUFFER_SIZE none # use none for build with no modification to 'Tracking::Relocalization()'

set -l dynavins_launch_files \
    orbslam3-dynavins-dataset.launch.py \
    orbslam3-stereo-dynavins-dataset.launch.py

set -l viode_launch_files \
    orbslam3-viode-dataset.launch.py \
    orbslam3-stereo-viode-dataset.launch.py

set -l our_launch_files \
    orbslam3-zed-svo-recording.launch.py \
    orbslam3-stereo-zed-svo-recording.launch.py

for ds in dynavins viode
    set -l dataset_launch_files (printf "%s_launch_files" $ds)
    echo "dataset_launch_files: $dataset_launch_files"
    for lf in $$dataset_launch_files
        for i in (seq $N)
            set -l name (string split --fields 1 . $lf)
            set -l log_file $ds/$name/$CIRCULAR_BUFFER_SIZE/$i.log
            mkdir -p (path dirname $log_file)
            test -f $log_file; and begin
                printf "%sLog file exists already: %s%s\n" $yellow $log_file $reset
                printf "Skipping...\n"
                continue
            end
            printf "[%d/%d] %s%s%s\n" $i $N $green $log_file $reset
            ros2 launch orbslam3_bringup $lf 2>&1 >$log_file
            humantime $CMD_DURATION
            pkill stereo
            pkill stereo-intertial
            pkill rviz2
            sleep 15s # just to be safe, that we reclaim the memory and not run out of disk space
        end
    end
end
```

To test with [our dataset](https://drive.google.com/drive/folders/1d_DGQ7RR32jnzah5OBjWXJKaYWcl07C1)

You need to run each test manually. The command to do so is:

```bash
ros2 launch orbslam3_bringup orbslam3-zed-svo-recording.launch.py
ros2 launch orbslam3_bringup orbslam3-stereo-zed-svo-recording.launch.py
```

## Baseline Experiments

To run the baseline experiments, you need to rebuild the library as it is on the `main` branch. Once you have done that. You can repeat the steps in the section above.
