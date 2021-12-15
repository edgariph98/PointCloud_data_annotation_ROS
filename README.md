

<div id="top"></div>

<!-- PROJECT LOGO -->

<div align="center">

<a href="https://github.tamu.edu/trevor-ledbetter/lidar-data-annotation-interface/">

</a>

<h3 align="center">LiDAR Data Annotation Interface</h3>

<p align="center">

A RQT plugin that is able to load rosbags that contains LiDAR data and label annotations.


<br />

<br />

<a href="https://drive.google.com/file/d/10pa8xkL4puEllZvWJ3kM9zKbwSS3qQzC/view?usp=sharing">View Demo</a> · <a href="https://github.tamu.edu/trevor-ledbetter/lidar-data-annotation-interface/issues">Report Bug</a> · <a href="https://github.tamu.edu/trevor-ledbetter/lidar-data-annotation-interface/issues">Request Feature</a>

</p>

</div>

<!-- TABLE OF CONTENTS -->

<details>

<summary>Table of Contents</summary>

<ol>

<li>

<a href="#about-the-project">About The Project</a>

</li>

<li>

<a href="#getting-started">Getting Started</a>

<ul>

<li><a href="#prerequisites">Prerequisites</a></li>

<li><a href="#installation">Installation</a></li>

</ul>

</li>

<li><a href="#usage">Usage</a>
<ul>
<li><a href="#loading-a-rosbag">Loading a rosbag</a></li>
<li><a href="#navigating-through-the-point-cloud-data">Navigating through the Point Cloud data</a></li>
<li><a href="#changing-the-current-frame">Changing the current frame</a></li>
<li><a href="#making-an-annotation-group">Making an annotation group</a></li>
<li><a href="#making-an-annotation">Making an annotation</a></li>
<li><a href="#selecting-an-annotation">Selecting an annotation</a></li>
<li><a href="#deleting-an-annotation">Deleting an annotation</a></li>
<li><a href="#exporting-annotations">Exporting annotations</a></li>
</ul>
</li>

<li><a href="#roadmap">Roadmap</a></li>

<li><a href="#contributing">Contributing</a></li>

<li><a href="#contact">Contact</a></li>

</ol>

</details>

<!-- ABOUT THE PROJECT -->

##  About The Project

![](https://lh5.googleusercontent.com/TiBSUOrXbN5Q0VTLVCmiGoCnghEN1cO6JDCMUCrQqR28L9ZqG19oJ2IEAas5wkRbakyLiudGYHgU_XbMyBOj0XeeYGsUY8bpmnrp0ChVwSpCzt3mNT1B-4egd6u1TRUo9lefjb3u)
Users working to develop autonomous driving vehicles require labelled LiDAR to train and test machine learning models. The process of annotating this point cloud data is slow and difficult without a tool dedicated to labeling LiDAR data. Because of this, our goal was to create an application that can be used to annotate LiDAR data quickly and efficiently, accepting input in a standardized rosbag format that is already used, and putting the annotated data into a structured form that can be read by machine learning modules.

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- GETTING STARTED -->

##  Getting Started

This is an example of how you may give instructions on setting up your project locally.

To get a local copy up and running follow these simple example steps.

###  Prerequisites

* ROS (Noetic or Melodic)

###  Installation

No hardware installation is necessary for this product.

_Since the main repository is a whole workspace, we can follow these steps to create a catkin workspace where our application can live._ 

To install our software we need to perform the following steps:

1.  Clone the repo from git
    
2.  `cd lidar-data-annotation-interface`
    
3.  `chmod +x ws_setup.sh`
    
4.  `./ws_setup.sh`
    
5.  `source ~/.bashrc`
    
These steps perform the installation of catkin tools, and makes a build using the command `catkin_make` of all the packages under `src/` directory to compile our application.

 _(Alternatively)_ For someone familiar with ROS, the source packages needed for our application can be found under the `src/` directory of our repository. We can clone the packages `rqt_mypkg` that contains the application, together with `annotation_msgs` package and place them into desired ROS or Catkin workspace, and build our packages there through tools like `catkin_make`


<p align="right">(<a href="#top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->

##  Usage


To launch our application once installed (after building packages, and or performed the previously workspace set-up):
1. In a separate terminal, launch running `roscore`
2. Use `rqt --standalone rqt_mypkg` in the terminal to launch the plugin

### Loading a rosbag:
1.  Click the “File” button in the top left corner
2.  Select “Load rosbag” from the dropdown menu
3.  On the window that appears, select the “Browse” button, and select the rosbag you wish to load from the file explorer
4.  On the “Load rosbag” window, the submit button should no longer be greyed out, as seen below:
![](https://lh5.googleusercontent.com/uWgHEOEK0w2T-qbE3BepQ_J7VuVLWLnEZt-bas77L6YG3OQMvemgyOrw8P2MF-6gzVML-1v_XxFANwB_JyNwvoEJUVdb2JfQKlRLeMJhUq_eGQqHDWcQGrsS4eyVKuHdkcgKq_Vt)
5.  If the path to the file is correct, and a LiDAR topic has been selected, click the submit button.
6.  If you are loading a rosbag that already contains annotations and annotation groups two additional dropdown menus will appear. These dropdowns allow you to choose which topic to load the annotations and annotation groups from if there are multiple.

### Navigating through the Point Cloud data:
![](https://lh4.googleusercontent.com/uAgd1JQrCj0Huyh5jkZUU12wlukUH1rgYcXqaD27bw4v64w7P77nil37Wk1iBqUUh9RbBgjZy8jwxr2jkB0P5ixkPav2XDhDkTi3FQYwoyRnqD96BeXN3o0tCWOierdWYlk5JAmM)
-   While using the Interaction Tool
	-   To rotate the view, left click on the point cloud data window (seen above), and move the mouse.
    
	-   To move the view laterally, hold down shift, left click on the point cloud data window, and move the mouse.
    
	-   To zoom in and out, either use your mouse wheel, or right click and move the mouse up to zoom in, and down to zoom out.
    
-   While using the Select Point Cloud Tool
    
	-   To rotate the view, hold down ALT and left click on the point cloud data window (seen above), and move the mouse.
    
	-   To move the view laterally, hold down SHIFT and ALT, left click on the point cloud data window, and move the mouse.
    
	-   To zoom in and out, hold down ALT and either use your mouse wheel, or right click and move the mouse up to zoom in, and down to zoom out.

### Changing the current frame:

-   To change which frame you are looking at, use the “Previous” and “Next” buttons below the visualization frame to move one frame forwards or backwards.
    
-   Alternatively, click and drag the marker on the blue timeline shown above to rapidly move through frames.
    ![](https://lh6.googleusercontent.com/uQFFRTCMIsAW8WnUGl18jpjG8rFPNJqe6PlHLD8cYeQFPU14ihHSmCMNpMA1_nxuGu6L7e7-zouYQ8om3iwy_AOM65xSBR0AaIgldrgR7fZc1Qf0h45sNyoc0EnIe6SeU6Rw_zxa)
-   Finally, you can manually input the frame number into the frame input box shown in the bottom right of the picture above.

### Making an annotation group:

1.  After a rosbag has been loaded the “Annotation Group” dropdown menu will appear in the top left of the screen. Additionally, the “Create Group” button will appear in the annotation list window in the top right corner of the screen.
    
2.  Either:

	-  Click on the “Annotation Group” menu in the top left, and select “Create annotation group” from the dropdown.
    
	-  Click the “Create Group” button near the top right corner of the user interface.
    
3.  In the popup window that appears, choose a name for the annotation group and enter it into the “Annotation group name” textbox.
    
4.  Click the “Browse Colors” button to open a color selection dialog.
    
5.  Click the color that you want, then click the “Select” button.
    
6.  Click “Submit”.

### Deleting an annotation group:
![](https://lh3.googleusercontent.com/cvY3bOMwarNV8iGCuw93cPYgNJ8zxjvNmzJNEbSHoBQ4VCa4DZWoA2mt-gxtHlssiI-lcMC-xns9x6cEt0ejF4jna0jhi1gBKFGPHxW-UOXN9CrlyFtqRiZuQMn6AmFfSymQslzT) 
1.  Either:
	-  Click on the “Annotation Group” menu in the top left, and select “Delete annotation group” from the dropdown.  
	- Click the “Delete Group” button in the top right corner of the user interface.
    
2.  In the popup window that appears select the annotation group that you want to delete from the dropdown menu.
    
3.  Click “Delete”.
    
4.  A warning dialog will appear, click “Yes” if you are sure you want to delete this annotation group and all annotations that belong to it.
    
### Making an annotation:

1.  First, click on the “Select Point Cloud Data Tool” button in the top left.![](https://lh4.googleusercontent.com/6qGyAg5xQXbLn4NiVqXkAGvHOsNh-lejKx037q-nKwgAjnhdrmpqjNTUMUI4PauU0ZYW6iWAqme4LifuVqxjI2pP7P8nNdKE1DO6jogZM3JmX229wogoi961KiUurKzTi_AH_5IJ)
    
2.  Then, click and drag over the point cloud data you wish to annotate.
    
	-  It helps to orient your view so that there are no unwanted points behind the points you select.
    
	-  You can add additional points to the selection by holding down SHIFT and selecting more points.
    
	- Similarly, you can deselect points by holding down CTRL and selecting the points you want to remove.
    

3.  On the lower right side of the screen, in the Create Annotation window, add a label (optional), and select an annotation group to assign this annotation to from the “Annotation group” dropdown menu.
    
4.  Select the “Create” button to finalize the annotation.

	-  If you want to clear the annotation, select the “Cancel” button instead.
    
	-  You can also press “c” on the keyboard to cancel an annotation.
    

### Selecting an annotation:

-   Using the annotation list window (top left of the screen)
    ![](https://lh5.googleusercontent.com/l-GS5MSgbCZLWAdkH9MW0egDfXTQE-RJIacPEmFlyMYkCo3jS1Xl60wF2Ad8n9YiKOBqr2_ceci52NBTB3SmQU1S3ufbe64MwybZXH6Kt-P9Nmp-t1hKMqfiJqia8uN6zAqClHE6)
	-  In the annotation list window expand the annotation group that your annotation belongs to by double clicking on the group.
    
	- Click on the annotation’s label (or id if no label was specified). Once selected this annotation is sent to the Annotation Details window in the window below.
    
	-  In the Annotation Details window select the “Delete” button.
    

  

-   Using the RVIZ display window![](https://lh5.googleusercontent.com/fkf7ltNzvZ2XBErZcoJS9v7Y9L2He5_r8jTGqOOq-iHiKNebr7UohvArLHJK87RrZ0F7dCJ9Iz0-Eq8ESI8gtfa0qQERP0nDXKKzlTx0H-rC_E3JOkGTE-GetAJSKFfoqvv05xyE)
	-  Select the Interaction tool.
	    
	-  Left click on an annotation. Once selected this annotation is sent to the Annotation Details window in the bottom right of the user interface.
	    
	-  Click the “Delete” button
    

### Deleting an annotation:

-   Using the annotation details window:
	- Select the annotation you want to delete using one of the methods above.
	-  Click the “Delete” button in the bottom right corner or the annotation details window.![](https://lh5.googleusercontent.com/42XXp9DyDEbCShtDcbO2HV4Hv5SDf42aQ7jZ7EIDUKB8SgUdvZczzfYXlq3xByLiICxV1pvYolwtEXOJEm_sNkapaFGmBA6-B5gQxmq1BhLCkdUI6_IHUrjvtDTEkyRrZwZzw26Q)
-   Using the RVIZ display window:
	-  Select the Interaction tool.    
	-  Right click on the annotation you want to delete.![](https://lh5.googleusercontent.com/fkf7ltNzvZ2XBErZcoJS9v7Y9L2He5_r8jTGqOOq-iHiKNebr7UohvArLHJK87RrZ0F7dCJ9Iz0-Eq8ESI8gtfa0qQERP0nDXKKzlTx0H-rC_E3JOkGTE-GetAJSKFfoqvv05xyE)
    
	-  Select the “Delete” button that appears in the window.
    

  

### Exporting annotations:
![](https://lh6.googleusercontent.com/_NuAY2g9BTb5sCluL5cjStdUcj9pGfnfX-QsDO6UifH5Y1_5sArEDaQplrPtOfxeoIZGnAMA0COY1-eebDmbtCffv4d3IKshD2r5wZZLyTfUsPDkzwcYiV8Bi2xBSJrtjmJdOcMM)

1.  Click the “File” dropdown menu in the top left corner of the user interface.
    
2.  Select “Export annotation” from the dropdown menu.
    
3.  Click the “Browse…” button.
    
4.  Use the file explorer window to select a location and filename.

	-  The filename is defaulted to output but you can choose whichever name you want by changing this name in the top of the file explorer window.
   
5.  Choose an annotation topic name, the default is “/annotations”.
    
6.  Click “Submit”.
    
7.  Notes
	- You are not able to export annotations into the same rosbag where the original point cloud and/or annotations. A new rosbag name and file is needed to load the new annotations together with the pointcloud and their associated frames.

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ROADMAP -->

##  Roadmap

See the [open issues](https://github.tamu.edu/trevor-ledbetter/lidar-data-annotation-interface/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTRIBUTING -->

##  Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".

1. Fork the Project

2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)

3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)

4. Push to the Branch (`git push origin feature/AmazingFeature`)

5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTACT -->

##  Contact

The 12th Unmanned Team - http://autodrive.tamu.edu/index.html

Project Link: [https://github.tamu.edu/trevor-ledbetter/lidar-data-annotation-interface](https://github.tamu.edu/trevor-ledbetter/lidar-data-annotation-interface)

<p align="right">(<a href="#top">back to top</a>)</p>

