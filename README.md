# cs393r_starter

## Using this Repo
You will be using a duplicate of this repo for all development in the class. This repo provides starter code for the ROS subscriptions, publishers and control loops you will be using.

### Prerequisites

Please refer to the [UT AUTOmata reference manual]() for instructions on setting up the dependencies:
* [ROS](http://wiki.ros.org/ROS/Installation)
* [amrl_msgs](https://github.com/ut-amrl/amrl_msgs)
* [amrl_maps](https://github.com/ut-amrl/amrl_maps)
* [ut_automata](https://github.com/ut-amrl/ut_automata)

### Duplicate the Repo
1. Make sure you're logged into your GitHub account.
2. Create a new repo with the same name under your GitHub account. Dont initialize that with anything.
3. `git clone <this repository url>` (found in the upper right)
4. `cd <cloned_repo>`
5. `git push --mirror <your new repository url>`

### Clone and Build
1. `git clone <your repository url>` (found in the upper right)
2. After you have cloned the repo, add it to your ROS path by adding the following line to the end of `~/.bashrc`:   
    ```
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:[Local path to your repo]
    ```
    For example:
    ```
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/joydeepb/cs393r_starter
    ```
3. Reload `.bashrc` so that the path is updated:
    ```
    source ~/.bashrc
    ```
4. `cd <cloned_repo>`
5. `make -j`
