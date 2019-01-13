# ChickenVision
![Alt text](https://cdn.discordapp.com/attachments/518873567065276416/533489000762310667/githubimage.png?raw=true "Example")

<h2>Requirements</h2>
<ul>
  <li>Raspberry Pi running <a href="https://wpilib.screenstepslive.com/s/currentCS/m/85074/l/1027241-using-the-raspberry-pi-for-frc">WPILib image</a> </li>
  <li>Green LED Ring to surround camera (You can use different colors, but must change HSV Threshold)</li>
  <li>Microsoft Lifecam HD 3000 (You can use a different camera, but must update diagonal field of view <b><i>line 32</i></b>)</li>
  <li>(Optional) USB WiFi Adapter for Raspberry Pi. This allows Pi to wirelessly send stream over home network for testing </li> 
  <li> Camera mounted in horizontal center of robot. Code can be updated to support offsets, but I currently do not know how to do that</li>
</ul>

<h2>Configuration</h2>
<h3>Things you may need to configure</h3>
<ul>
  <li>Exposure. You will need to adjust the exposure based on LED light ring. The lower you can set this value while still tracking vision targets, the better. My current setup is 21</li>
  <li>HSV Threshold. Depending on LED ring and exposure level, may need to adjust HSV Threshold (<b><i>line 56-57</i></b>). If using GRIP to find values:  Adjust HSV Sliders until targets are white
    <img src="https://cdn.discordapp.com/attachments/533795907448340480/533796001639563292/GripExample.png"/>
</li>
  
</ul>

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
