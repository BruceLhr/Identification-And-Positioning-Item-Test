# Identification-And-Positioning-Item-Test
This's a test Item, not full item. May be have full item in the future.

In order to catch mark object with a robot gripper, the full item have camera calibration, moving object detection, object identification and object location.
The full item may be use R-CNN instead of the current Moving object detection method and Object identification method, and find a new object location method integrate in R-CNN.

This test item is base on Opencv:
Camera calibration is ZhengYou Zhang camera calibration.
Moving object detection is background substract.
Object identification is Neural networks with HSV model.
Object location is SURF.

Camera calibration will detect a corner of first pic as origin of robot gripper, store the direction difference betwenn the first pic and the second pic as robot x-axis.
![image](https://github.com/BruceLhr/Item-pic/blob/master/item%20test-pic/pic1.png?raw=true)

Background substract need to use GMM in order to reduce noise.
![image](https://github.com/BruceLhr/Item-pic/blob/master/item%20test-pic/pic2.png?raw=true)

Neural networks with HSV model is very simple to configure. But if most of two object have same color, this way is not value. Reader can use other features to train Neural networks.
![image](https://github.com/BruceLhr/Item-pic/blob/master/item%20test-pic/pic3.png?raw=true)

Object location use SURF,it is beacause i can find any other way to achieve this part. SURF is not a good way to location a keypoint of object, bucause the object need to be placed on a flat surface.
![image](https://github.com/BruceLhr/Item-pic/blob/master/item%20test-pic/pic4.png?raw=true)

I very like the cvui. It can very quickly to design a ui. It's use!
![image](https://github.com/BruceLhr/Item-pic/blob/master/item%20test-pic/pic5.png?raw=true)

It record some full item progress.
Three thread：cvui，camera catch, image process.
New mode: classify object by size.
New distance calculation: input camera parameter by user.

