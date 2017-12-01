## ros_img_processor
Just a simple template node receivng an image and doing something. Links to OpenCV and ROS wrapped.

## Image processing has been modified:

1. Search circles in the image with the Hough transformation.
2. Draw found circle.
3. From camera_info message from usb_cam gets and saves the intrinsec calibration matrix (k).
4. Puts the circle center detected in the camer world coordinates by using 
	d=k¹*u where d is the direction ray and u is the circle center.
5. Draw a vector from circle center to the calculated ray direction
6. Publish ray direction topic "center_ray_direction"

## How to run the code
In a terminal window, type:
```sh
$ roslaunch ros_img_processor ros_img_processor.launch
```

## Tip
Check your webcam encodings (yuyv,mjpeg,...) and set them accordingly at file launch/usb_camera.launch
