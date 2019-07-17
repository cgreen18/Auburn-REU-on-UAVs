The flexx_point_cloud_stitch_app.mlapp is to be opened using the MATLAB app editor. 

This app is what you will use to choose .rrf and .txt files associated with stitching together a point cloud. This app is meant to act as a high level implementation.

![alt text](https://github.com/cgreen18/Auburn-REU-on-UAVs/blob/master/MATLAB/Cloud_stitching/Stitch_app_pic.PNG "Stitching App")
## Opening Files
The two buttons at the top right open a file browser where the user can choose their .rrf and .txt files one at a time.
The calibration edit field is to specify how much empty space there was before the recording actually starting (this is a user determined value), most will probably set it between 0 and 1.

## Choose Stitching Method
The Nav_ICP Simultaneous check box gives the user the ability to change between running a script that weights the navigation transforms and ICP transforms and applies them together, or (unchecked) applies 100% navigation, and then 100% ICP. 

In situations when the navigation data is very well processed and filtered (in other words, good nav data), un-checking this may correct the stitch even more. 

If you want to reply completely on the computer, check the Nav_ICP Simultaneous check box, and set the slider to 0% nav trust. 

## Processing 
Before clicking the Process button, make sure Refesh Data is checked only if you are gathering the data for the first time. Else, if you just want to change the start and stop frames, uncheck refresh data. 

## Viewing Section
In the viewing section, this is all pretty self explanatory, Color lets you set the color of the point cloud. Plot Shift lets you shift the plot either rotationally (degrees) or translationally (meters). Viewer sets the camera position and is only effective when the movie box is checked. 

Save cloud will save the current global variable ptCloudScene, which you can find inside the code view, to the workspace. This variable holds the actual point clouds scene. 

### Movie
Though not completely finished, the movie checkbox will allow a couple of options for creating a movie like feel for observing the point cloud. When checked, the movie will play the option specified by the radio button under the Viewer tab. 
