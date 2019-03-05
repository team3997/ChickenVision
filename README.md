<h2> Welcome to Chicken Vision </h2>
<img src="https://github.com/team3997/ChickenVision/blob/master/StickerVision.svg"/>
To tune vision, use GRIP (http://wpiroboticsprojects.github.io/GRIP/#/).
<h2> Credit </h2>
You don't have to give credit, but if you do, it is greatly appreciated if you tell us how Chicken Vision has helped you and your team: https://docs.google.com/spreadsheets/d/1YWcWk0oOwUUU_g2qIJem4bmJQUaB20VSqDUPqsKFyJk/edit?usp=sharing
Tweet <b>#ChickenVision</b>! Let's try to get it trending!
 <br/>
  <br/>Also, huge thanks to Team 3216 who seriously improved README and really went the extra mile in producing excellent instructions.

# Requirements
* Raspberry Pi running WPILib image (https://github.com/wpilibsuite/FRCVision-pi-gen/releases)
* Bright, small (60mm), green LED Ring to surround camera (You can use different colors, but must change HSV Threshold)
* Microsoft Lifecam HD 3000
* (Optional) USB WiFi Adapter for Raspberry Pi or use Ethernet.
* Camera mounted in horizontal center of robot. Code can be updated to support offsets, but I currently do not know how to do that
* For Windows: Download NI Driver Station (http://www.ni.com/download/labview-for-frc-18.0/7841/en/)
* Chicken Vision Code (https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/ChickenVision.py)
* FRC Update Suite (http://download.ni.com/support/softlib/first/frc/FileAttachments/FRCUpdateSuite_2019.2.0.zip) (extract and install)

# Tuning For Reflective Tape
<br>

## Using GRIP

### Finding an IP
Once GRIP is downloaded, open it. Afterwards, click sources and click IP Camera. To find the IP of the raspberry pi, use a network scanner such as FING for android (https://play.google.com/store/apps/details?id=com.overlook.android.fing&hl=en_US). Once you have found the IP, put that in the box. Sometimes, the host name works, making it so that you can use frcvision.local/ rather than the IP.

### Configuring GRIP
From the operation palate (The right side of GRIP) (picture below) 
<br> 
![GRIP palate](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/palate.PNG) 
<br> 
Drag the according image processing functions tho the area below (ex. Blur, Filter Lines). Put the functions in order of the picture below how to connect the different functions (next two pictures). To connect different functions, click and drag one of the small empty circles 
<br> 
![Draggable Dot](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/emptyDrag.PNG) 
<br>
And drag it to another function, it should look like this (MAKE SURE THAT YOU DO ONE OUTPUT OF A FUNCTION TO THE INPUT OF ANOTHER FUNCTION)
<br>
![completed function](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/draggedComplete.PNG)
<br>
![GRIP image demo picture](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/GRIP.PNG?raw=true)

After You have set up all of the functions, it's time to apply them. Simply drag the knobs of the values (ex. H, S, and V) according to the picture below. You can change them if needed if the example doesn't work. You also may need to change the raspberry pi's settings (https://github.com/MRT3216/MRT3216-2019-DeepSpace/wiki/FRC-2019-Vision#configuring-the-raspberry-pi).
<br>
![GRIP image demo picture](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/GRIP.PNG?raw=true)
<br>
Finally, Click the eye icon (on all the functions) (picture below) to show the functions being applied
<br>
![eye icon from GRIP](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/eye.PNG)
<br>

## What HSV stands for
H = Hue
<br>
S = Saturation
<br>
V = Variable/Brightness

## Changing Chicken to Detect Vision Tape
When you are done setting the values from the steps above, click the tools tab in GRIP and click export code. Change the language to Python and set the Pipeline class name to GripPipeline. Then set a save location that you can access later. Lastly, change the Module Name to whatever you want. It should look like this.
<br>
![Export Code](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/codeGen.PNG)
<br>

Once you've exported the code, pull up chicken vision and the GRIP python file you made side by side.
<br>
![Code side by side](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/codesidebyside.PNG)
<br>

In the Chicken Vision code, you have to change the HSV Values. Below is where you change it. (Line 176 and Line 177)
<br>
![chicken vision HSV Values](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/HSVChicken.PNG)
<br>
To change those values, you need the GRIP file's HSV Values. Below is where you get that. (Lines 21, 22 and 23) 
<br>
![GRIP HSV Values](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/HSVGRIP.PNG)

GRIP and Chicken Vision set HSV differently. For GRIP, it's an array. Ex. hue = [low_value, high_value]. For Chicken Vision it's different. Ex. 
<br>
lower_green = np.array([low_hue_value])
<br>
upper_green = np.array([high_hue_value])
<br>
So in more complicated terms, it goes like this for Chicken Vision.
<br>
lower_green = np.array([low_GRIP_hue, low_GRIP_saturation, low_GRIP_value])
<br>
upper_green = np.array([high_GRIP_hue, high_GRIP_saturation, high_GRIP_value])
<br>

Once you are done with that, if you had a different blur that isn't 1 (MUST BE ODD!), change it on line 172.
<br>
![Blur Set](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/blur.PNG)
<br>

The next thing to do is upload it to the Raspberry Pi.
# The Raspberry Pi Image

## Configuring the raspberry pi

Go to the IP of the raspberry pi in a web browser (If you lost it, refer back to https://github.com/MRT3216/MRT3216-2019-DeepSpace/wiki/FRC-2019-Vision#finding-an-ip). After that, follow the picture below.
<br>
![Upload the code](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/OpenChicken.PNG)

After that go to the raspberry pi's IP:1881 (or frcvision.local/). You should see the camera stream and a whole bunch of settings you can change (picture below). Change your settings accordingly.
<br>
![camera settings](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/optionsCamera.PNG)
<br>
When you have done so, go to the IP of your Raspberry Pi and go to the Vision Settings tab (picture below). Click on camera one and some settings should appear below.
<br>
![Camera JSON](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/optionsCameralocal.PNG)
<br>
Click the Copy Source Config From Camera button, and you're done with the Raspberry Pi!
<br>

## Shuffle Board Configuration
After that, open the FRC Shuffleboard that you downloaded in the requirements section and plug in a Ethernet cable into the Raspberry Pi from the RoboRio (https://github.com/MRT3216/MRT3216-2019-DeepSpace/wiki/FRC-2019-Vision#requirements).

Then open the left tab under where it says Configuration (top left corner) (it also should have a close application X to the left of configuration). Drag the arrows to the right (respective distance) (image below).
<br>
![arrow to pull tab open](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/arrows.PNG)
<br>

To view the camera stream, click on the CameraServer dropdown and drag the text stream to the dashboard and violala! Your video stream is in Shuffle Board! (picture below).
<br>
![CameraServer tab](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/shuffboardCameraServer.PNG)
<br>

To view Network Tables, Click the Network Tables tab and under ChickenVision dropdown are the values being outputted by the Chicken Vision Python program (picture below).
<br>
![Network Tables Dropdown](https://github.com/MRT3216/MRT3216-2019-DeepSpace/blob/master/Vision/example_photos/shuffboardNetworkTables.PNG)
<br>

## Network Tables

In the code, you can send out network table values Ex. networkTable.putNumber("VideoTimestamp", timestamp). You can also put other types of data such as a boolaen and other types of data.


<h2>Functionality/Features</h3>
<ul>
  <li>Sanity checks: Filters out contours whose rays form a V, only recognizes targets whose contours are adjacent</li>
  <li>Returns the angle (in degrees) of closest target for easy integration for robot program (Gyro)</li>
  <li>If angle is to two targets are the same, it picks the left target. You can change this in code</li>
  <li>Pre-calculated (but sub-optimal) built in HSV Threshold range</li>
  <li>Should be plug-and-play</li>
  <li>All contours have green shapes around them along with a white dot and vertical line running through their center point</li>
  <li>Targets have vertical blue line in between contours. Yaw is calculated from that x coordinate. There should only be one blue line (one Target) at a time.</li>
  <li>Rounded yaw (horizontal angle from camera) is displayed in large white font at the top of the screen</li>
  <li>Team 254's explanations linked in comments of angle calculation functions</li>
</ul>

<h2>Resources and Links</h2>
<ul>
  <li><a href="https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf"> Microsoft Lifecam Datasheet</a></li>
  <li><a href="http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html"> Calculating horizontal field of view</a></li>
  <li><a href="https://www.team254.com/documents/vision-control/"> Calculating focal length and yaw</a></li>
 
</ul>


