## ROS/NEPTUS Integration Tutorial
This is the tutorial that was given in BTS 2019.
The tutorial slides can be found [here](https://docs.google.com/presentation/d/1YbstQzWH-KX7bxNNlNfiHhSm0vZiyre15n5FZqtvWO0/edit?usp=sharing)

## Running the simulation
This package was tested and ran under Ubuntu 16.04 and ROS Kinetic.

### Clone this package and install everything needed
Clone this repo then move into the bts_tutorial package and let it install all the things needed.
```
git clone https://github.com/smarc-project/smarc_scenarios.git
cd bts_tutorial
rosinstall bts_tutorial.rosinstall
catkin_make install
```

Install Neptus and set it up to use lolo-auv. See [here](https://github.com/smarc-project/imc_ros_bridge)

### Start running things
Start Neptus: `./neptus.sh`
Within Neptus, open the IMC Monitor by double clicking its icon. Then start IMC comms by clicking the small imc button in the IMC Comm. Monitor window. This acts as a way to see messages to and from Neptus.

Run each of the following commands in new terminals.

Run `roscore`, having a separate terminal running roscore makes restarts faster when needed.

Start up Gazebo with the biograd coast model loaded: `roslaunch bts_tutorial biograd_coats.launch`.

Put Lolo into Gazebo: `roslaunch bts_tutorial auv_sim.launch`. You can now follow the AUV. On the left panel of Gazebo, under Models you can find `lolo_auv_1`. Right click, follow. In order to fix the visuals, right click lolo's name -> view -> transparent. Do this twice. This is a [bug in Gazebo](https://bitbucket.org/osrf/gazebo/issues/404/collada-texture-transparency-bug) with transparent textures.

Run the navigation stack: `roslaunch bts_tutorial navigation.launch`

Run the imc bridge: `roslaunch bts_tutorial bridge.launch`

At this point, you should see `LOLO_AUV` pop up with id 00:05 under IMC Comm. Monitor in Neptus. This means we have a connection to the AUV running in Gazebo.

This is what you should be seeing:
![Neptus and Gazebo simulator showing Lolo](LINKTO1.PNGHERE "Neptus and Gazebo")

### Making and executing a plan
From here, it is mostly about using Neptus. 

To make a simple waypoint-based plan, go to Neptus->Systems(top bar)->LOLO_AUV. This opens a new small window showing a small picture of lolo. Click the Console button in this window. The console window is where most of the planning happens.

Navigate to Biograd in the map, you can look at the terrain in Gazebo to roughly see where the boundaries of the simulation are. Essentially, the area in front of the hotel is where you should plan a mission. 

Once the view loads the map from the internet, click the small red P button on the left bar. Select LOLO_AUV from the drop down list and press ok. This creates an empty plan for you to place waypoints in. You can check that you are in the plan creation mode by checking that there is a second panel on the right that says "No maneuver selected" on top. The small P symbol on the left bar should also have a darker background.

In the plan creation mode, right clicking on the map shows a menu of available maneuvers. We will only use two types here: Goto and CoverArea. The Goto maneuver is just that, a waypoint for the vehicle to go to. Once you add one of these, the right panel will show the properties of the maneuver. You can change the depth of the waypoint here. The CoverArea maneuver is a Goto maneuver that also acts as a toggle for the MBES. Essentially when lolo's current waypoint is a CoverArea, it will toggle the MBES. This behaviour is entirely defined in the behaviour tree being used. 

Play with the waypoints, drag them around, make new ones, etc. Once you are happy with your plan, click save on the right panel, near the buttom. Give your plan a name and click ok. This will move you out of the planning view and back into the default view. You can now see your new plan on the right panel. 

To send the plan to the lolo in gazebo, first make sure that lolo is selected on the top-right corner drop down menu. Then make sure the plan is selected as well. You should see the plan you made in the map. If all is well, clicking the blue arrow near the top of the right panel will send the plan to lolo. Since lolo does not wait for an extra command to start executing, it will immediately start moving in simulation. 

At this point, lolo will start sending back feedback to Neptus. Its location will start updating in the map and the text fields near the top of the right panel will start showing some info. 

### Monitoring the mission
You can use Rviz at the same time to monitor different topics. 
TODO: This part.
TODO: Pictures and video.




